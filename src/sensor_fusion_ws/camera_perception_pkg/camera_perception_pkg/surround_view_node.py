"""Compose up to four camera images into a configurable bird's-eye view."""

from typing import Dict, Optional, Sequence

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


CAMERAS = ('front', 'rear', 'left', 'right')


class SurroundViewNode(Node):
    """Warp, mask and feather-blend camera images on one ground-plane canvas."""

    def __init__(self) -> None:
        super().__init__('surround_view_node')
        self.declare_parameter('output_topic', '/surround_view/image')
        # 캔버스는 "차를 가운데 둔 지면 좌표계"다. dst_points가 이 픽셀 좌표로
        # 저장되므로, 한 번 정하면 바꾸지 않는 게 좋다 - 크기를 바꾸는 순간 찍어둔
        # 네 점이 전부 어긋난다. 전방 카메라를 나중에 붙일 계획이면 지금부터 차 앞쪽
        # 여백까지 포함된 크기로 잡아두고, 당장 안 쓰는 영역은 view_rect로 잘라낸다.
        self.declare_parameter('canvas_width', 800)
        self.declare_parameter('canvas_height', 1000)
        # 캔버스에서 실제로 내보낼 사각형 [x1, y1, x2, y2]. 잘라내는 게 아니라 애초에
        # 이 영역만 계산한다 - 안 쓰는 영역은 워핑도 블렌딩도 안 하므로 후방만 볼 때는
        # 그만큼 빨라진다. 기본값은 캔버스 전체이고, 그러려면 캔버스 크기를 먼저 알아야
        # 해서 여기서 한 번 읽는다(기본값을 빈 리스트로 두면 rclpy가 BYTE_ARRAY로
        # 추론해서 정수 네 개를 넣는 순간 타입 에러로 죽는다).
        canvas_width = int(self.get_parameter('canvas_width').value)
        canvas_height = int(self.get_parameter('canvas_height').value)
        self.declare_parameter('view_rect', [0, 0, canvas_width, canvas_height])
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_frame_age', 0.5)
        self.declare_parameter('show_preview', True)
        self.declare_parameter('window_name', 'Surround View')
        self.declare_parameter('blend_feather_px', 35)
        # average: 겹치는 곳을 가중 평균 -> 이음매가 부드럽지만 두 영상이 다 흐려진다.
        # priority: blend_priority가 높은 카메라가 겹치는 영역을 차지하고, 낮은 쪽은
        #   그 카메라가 안 덮는 바깥만 채운다. 광각 왜곡이 심한 측면 카메라가 선명한
        #   후방 영상 위에 번지는 걸 막으려면 이쪽을 쓴다.
        self.declare_parameter('blend_mode', 'average')
        self.declare_parameter('draw_vehicle', True)
        self.declare_parameter('vehicle_rect', [310, 370, 490, 630])
        self.declare_parameter('vehicle_label', 'FRONT')
        self.declare_parameter('vehicle_color_bgr', [55, 55, 55])
        # 각 카메라가 캔버스의 어디를 덮는지 윤곽선으로 표시(정렬 맞출 때만 켠다).
        self.declare_parameter('draw_camera_outlines', False)

        defaults = {
            'front': ([0.18, 0.55, 0.82, 0.55, 1.0, 1.0, 0.0, 1.0],
                      [310.0, 370.0, 490.0, 370.0, 720.0, 0.0, 80.0, 0.0]),
            'rear': ([0.18, 0.55, 0.82, 0.55, 1.0, 1.0, 0.0, 1.0],
                     [490.0, 630.0, 310.0, 630.0, 80.0, 999.0, 720.0, 999.0]),
            'left': ([0.18, 0.55, 0.82, 0.55, 1.0, 1.0, 0.0, 1.0],
                     [310.0, 630.0, 310.0, 370.0, 0.0, 80.0, 0.0, 920.0]),
            'right': ([0.18, 0.55, 0.82, 0.55, 1.0, 1.0, 0.0, 1.0],
                      [490.0, 370.0, 490.0, 630.0, 799.0, 920.0, 799.0, 80.0]),
        }
        for name in CAMERAS:
            self.declare_parameter(f'{name}.enabled', name in ('front', 'rear'))
            self.declare_parameter(f'{name}.topic', f'/{name}/image_raw')
            self.declare_parameter(f'{name}.rotate_deg', 0)
            self.declare_parameter(f'{name}.src_points', defaults[name][0])
            self.declare_parameter(f'{name}.dst_points', defaults[name][1])
            self.declare_parameter(f'{name}.normalized_src_points', True)
            # 값이 클수록 위에 그려진다(blend_mode가 priority일 때만 의미 있음).
            self.declare_parameter(f'{name}.blend_priority', 10)

        self.width = canvas_width
        self.height = canvas_height
        self.view_rect = [int(v) for v in self.get_parameter('view_rect').value]
        self.rate = float(self.get_parameter('publish_rate').value)
        self.max_age = float(self.get_parameter('max_frame_age').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.feather = int(self.get_parameter('blend_feather_px').value)
        self.blend_mode = str(self.get_parameter('blend_mode').value)
        self.draw_vehicle = bool(self.get_parameter('draw_vehicle').value)
        self.vehicle_rect = [int(v) for v in self.get_parameter('vehicle_rect').value]
        self.vehicle_label = str(self.get_parameter('vehicle_label').value)
        self.draw_outlines = bool(self.get_parameter('draw_camera_outlines').value)
        self.vehicle_color = tuple(
            int(v) for v in self.get_parameter('vehicle_color_bgr').value)
        self._validate_global_parameters()

        view_x1, view_y1, view_x2, view_y2 = self.view_rect
        self.view_w = view_x2 - view_x1
        self.view_h = view_y2 - view_y1
        # 캔버스 좌표를 잘라낸 출력 좌표로 옮기는 평행이동. 호모그래피에 미리 곱해두면
        # 워핑 자체가 잘라낸 크기로만 일어난다 - 안 쓰는 영역은 계산조차 안 한다.
        self.view_shift = np.array([[1.0, 0.0, -view_x1],
                                    [0.0, 1.0, -view_y1],
                                    [0.0, 0.0, 1.0]], dtype=np.float64)

        self.bridge = CvBridge()
        self.frames: Dict[str, np.ndarray] = {}
        self.frame_times: Dict[str, float] = {}
        self.config = {}
        # rclpy Node already exposes a read-only ``subscriptions`` property.
        # Keep our references under a private, non-conflicting name.
        self._camera_subscriptions = []
        # 호모그래피와 마스크는 입력 해상도가 그대로면 매 프레임 같은 값이다.
        # 특히 깃털 처리용 GaussianBlur가 800x600에서 무겁다 - 카메라 3대분을 매번
        # 다시 계산하면 20Hz 목표가 6Hz까지 떨어진다(Jetson 실측). 크기별로 캐시한다.
        self._warp_cache = {}
        # 블렌딩 가중치도 마스크에만 의존한다. 다만 카메라 한 대가 끊기면 남은
        # 조합으로 다시 정규화해야 해서, 살아있는 카메라 조합별로 캐시한다.
        self._weight_cache = {}
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        for name in CAMERAS:
            config = {
                'enabled': bool(self.get_parameter(f'{name}.enabled').value),
                'topic': str(self.get_parameter(f'{name}.topic').value),
                'rotate': int(self.get_parameter(f'{name}.rotate_deg').value),
                'src': np.asarray(self.get_parameter(f'{name}.src_points').value,
                                  dtype=np.float32).reshape(-1, 2),
                'dst': np.asarray(self.get_parameter(f'{name}.dst_points').value,
                                  dtype=np.float32).reshape(-1, 2),
                'normalized': bool(self.get_parameter(
                    f'{name}.normalized_src_points').value),
                'priority': int(self.get_parameter(f'{name}.blend_priority').value),
            }
            self._validate_camera(name, config)
            self.config[name] = config
            if config['enabled']:
                callback = lambda msg, camera=name: self._image_callback(camera, msg)
                self._camera_subscriptions.append(self.create_subscription(
                    Image, config['topic'], callback, qos))
                self.get_logger().info(f'{name}: subscribing to {config["topic"]}')

        self.publisher = self.create_publisher(
            Image, str(self.get_parameter('output_topic').value), qos)
        self.timer = self.create_timer(1.0 / self.rate, self._compose_and_publish)

    def _validate_global_parameters(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.rate <= 0:
            raise ValueError('Canvas dimensions and publish_rate must be positive.')
        if self.max_age <= 0 or self.feather < 0:
            raise ValueError('max_frame_age must be positive and feather non-negative.')
        if self.blend_mode not in ('average', 'priority'):
            raise ValueError("blend_mode must be 'average' or 'priority'.")
        x1, y1, x2, y2 = self.vehicle_rect
        if not (0 <= x1 < x2 <= self.width and 0 <= y1 < y2 <= self.height):
            raise ValueError('vehicle_rect must be inside the output canvas.')
        if len(self.view_rect) != 4:
            raise ValueError('view_rect needs four values [x1, y1, x2, y2].')
        vx1, vy1, vx2, vy2 = self.view_rect
        if not (0 <= vx1 < vx2 <= self.width and 0 <= vy1 < vy2 <= self.height):
            raise ValueError('view_rect must be inside the output canvas.')


    @staticmethod
    def _validate_camera(name: str, config: dict) -> None:
        if config['src'].shape != (4, 2) or config['dst'].shape != (4, 2):
            raise ValueError(f'{name} src_points/dst_points need four x,y pairs.')
        if config['rotate'] not in (0, 90, 180, 270):
            raise ValueError(f'{name}.rotate_deg must be 0, 90, 180 or 270.')
        if abs(cv2.contourArea(config['dst'])) < 1.0:
            raise ValueError(f'{name}.dst_points form an empty polygon.')

    def _image_callback(self, name: str, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'{name} conversion failed: {error}')
            return
        rotate = self.config[name]['rotate']
        if rotate:
            rotate_codes = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                            270: cv2.ROTATE_90_COUNTERCLOCKWISE}
            frame = cv2.rotate(frame, rotate_codes[rotate])
        self.frames[name] = frame
        self.frame_times[name] = self.get_clock().now().nanoseconds / 1e9

    def _warp_constants(self, name: str, shape):
        """Homography and blend mask for one camera at one input resolution."""
        cached = self._warp_cache.get(name)
        if cached is not None and cached[0] == shape:
            return cached[1], cached[2]

        config = self.config[name]
        src = config['src'].copy()
        if config['normalized']:
            src *= np.array([shape[1] - 1, shape[0] - 1], np.float32)
        matrix = self.view_shift @ cv2.getPerspectiveTransform(src, config['dst'])

        source_mask = np.zeros(shape, dtype=np.uint8)
        cv2.fillConvexPoly(source_mask, np.rint(src).astype(np.int32), 255)
        mask = cv2.warpPerspective(source_mask, matrix, (self.view_w, self.view_h),
                                   flags=cv2.INTER_NEAREST)
        if self.feather:
            mask = cv2.GaussianBlur(mask, (0, 0), self.feather / 3.0)
        mask = mask.astype(np.float32) / 255.0

        self._warp_cache[name] = (shape, matrix, mask)
        self._weight_cache.clear()      # 마스크가 바뀌면 가중치도 다시 구해야 한다
        self.get_logger().info(
            f'{name}: warp built for {shape[1]}x{shape[0]} input '
            f'-> {self.view_w}x{self.view_h} view')
        return matrix, mask

    def _warp(self, name: str, frame: np.ndarray):
        matrix, mask = self._warp_constants(name, frame.shape[:2])
        warped = cv2.warpPerspective(frame, matrix, (self.view_w, self.view_h))
        return warped, mask

    @staticmethod
    def blend_weights(masks, mode: str):
        """Per-pixel weights that sum to 1 wherever at least one camera covers.

        마스크만 있으면 정해지는 값이라 영상과 무관하다 - 그래서 한 번 구해두고
        프레임마다 재사용한다. average는 겹치는 곳을 그냥 평균내고, priority는
        나중 레이어(우선순위 높은 쪽)가 앞 레이어를 밀어낸다.

        priority에서 누적 알파로 나누는 이유: 안 나누면 깃털 구간의 알파가 1보다
        작아서 가장 아래 레이어의 바깥 테두리가 검게 죽는다. 나눠주면 아무도 안
        덮은 곳만 검게 남고 테두리는 원래 밝기를 유지한다.
        """
        weights = []
        total = np.zeros(masks[0].shape, dtype=np.float32) if masks else None
        for mask in masks:
            if mode == 'priority':
                inverse = 1.0 - mask
                for index in range(len(weights)):
                    weights[index] = weights[index] * inverse
                total = total * inverse + mask
            else:
                total = total + mask
            weights.append(mask.astype(np.float32, copy=True))

        divisor = np.maximum(total, 1e-6)
        # 3채널로 미리 펴 두면 가중합에서 브로드캐스트 비용이 사라진다.
        return [np.repeat((weight / divisor)[:, :, None], 3, axis=2)
                for weight in weights]

    @staticmethod
    def apply_weights(images, weights, width: int, height: int) -> np.ndarray:
        """Weighted sum of the warped images; weights must already be normalised."""
        result = np.zeros((height, width, 3), dtype=np.float32)
        for image, weight in zip(images, weights):
            result += image * weight
        # 가중치 합이 1이라 넘칠 일은 없지만 부동소수 오차가 255를 넘기면
        # uint8로 감기면서 밝은 픽셀이 까맣게 뒤집힌다.
        np.minimum(result, 255.0, out=result)
        return result.astype(np.uint8)

    @classmethod
    def compose_layers(cls, layers, width: int, height: int) -> np.ndarray:
        """Weighted blend, exposed separately so it can be unit-tested."""
        images = [image for image, _ in layers]
        return cls.apply_weights(
            images, cls.blend_weights([mask for _, mask in layers], 'average'),
            width, height)

    @classmethod
    def composite_layers(cls, layers, width: int, height: int) -> np.ndarray:
        """Alpha-over in order: the last layer wins wherever it is opaque."""
        images = [image for image, _ in layers]
        return cls.apply_weights(
            images, cls.blend_weights([mask for _, mask in layers], 'priority'),
            width, height)

    def _weights_for(self, active):
        key = tuple(active)
        weights = self._weight_cache.get(key)
        if weights is None:
            masks = [self._warp_cache[name][2] for name in active]
            weights = self.blend_weights(masks, self.blend_mode)
            self._weight_cache[key] = weights
            self.get_logger().info(
                f'blend weights built for {self.blend_mode}: {", ".join(active)}')
        return weights

    def _draw_outlines(self, canvas: np.ndarray, active) -> None:
        offset = np.array(self.view_rect[:2], dtype=np.float32)
        for name in active:
            polygon = np.rint(self.config[name]['dst'] - offset).astype(np.int32)
            cv2.polylines(canvas, [polygon], True, (0, 200, 255), 1)

    def _draw_vehicle(self, canvas: np.ndarray) -> None:
        # vehicle_rect는 캔버스 좌표라 잘라낸 출력 좌표로 옮겨야 한다.
        # 잘린 영역 밖으로 나가는 부분은 cv2가 알아서 클리핑한다.
        shift_x, shift_y = self.view_rect[0], self.view_rect[1]
        x1, y1, x2, y2 = [v - s for v, s in
                          zip(self.vehicle_rect, (shift_x, shift_y, shift_x, shift_y))]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), self.vehicle_color, -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (210, 210, 210), 3)
        inset = max(5, (x2 - x1) // 6)
        cv2.rectangle(canvas, (x1 + inset, y1 + inset),
                      (x2 - inset, y2 - inset), (25, 25, 25), -1)
        cv2.putText(canvas, self.vehicle_label, (x1 + 8, y1 + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    def _compose_and_publish(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        ready = []
        for name in CAMERAS:
            if not self.config[name]['enabled'] or name not in self.frames:
                continue
            if now - self.frame_times[name] > self.max_age:
                continue
            ready.append(name)
        if not ready:
            return

        # priority 모드는 낮은 우선순위부터 깔고 높은 쪽을 위에 덮는다.
        if self.blend_mode == 'priority':
            ready.sort(key=lambda name: self.config[name]['priority'])
        active = ready
        warped = [self._warp(name, self.frames[name])[0] for name in active]
        canvas = self.apply_weights(warped, self._weights_for(active),
                                    self.view_w, self.view_h)
        if self.draw_outlines:
            self._draw_outlines(canvas, active)
        if self.draw_vehicle:
            self._draw_vehicle(canvas)
        cv2.putText(canvas, ' | '.join(active), (12, self.view_h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        message = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'surround_view'
        self.publisher.publish(message)
        if self.show_preview:
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_preview:
            cv2.destroyWindow(self.window_name)
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = SurroundViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
