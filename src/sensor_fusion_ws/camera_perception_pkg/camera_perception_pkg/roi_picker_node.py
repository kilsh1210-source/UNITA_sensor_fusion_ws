"""Click the ground ROI of each surround camera and see the bird's-eye result live.

surround_view_node는 카메라마다 두 쌍의 값을 필요로 한다.
  - src_points: 카메라 영상 안에서 지면 사다리꼴 네 점
  - dst_points: 그 네 점이 top-down 캔버스의 어디에 놓이는지
README에도 적혀 있듯 지금 params.yaml에 들어있는 값은 placeholder라, 실제로 달아놓은
상태에서는 의미 있는 그림이 안 나온다. 그렇다고 자를 들고 바닥을 재서 픽셀을 손으로
적어 넣는 것도 번거롭다.

이 노드는 그 두 값을 눈으로 보면서 찍는 도구다. 왼쪽에 카메라 원본, 오른쪽에 top-down
캔버스를 띄우고, 바닥의 같은 지점을 양쪽 패널에서 같은 순서로 클릭하면 그 자리에서
워핑 결과가 캔버스에 겹쳐 나온다. 카메라를 바꿔가며 같은 바닥 마커를 찍으면 좌/우
카메라가 후방 카메라 위에 정확히 포개진다 - 이게 여러 카메라를 맞추는 유일하게 확실한
방법이다(좌/우가 후방을 비스듬히 보고 있어 겹치는 영역이 넓기 때문에 더 그렇다).

s키를 누르면 params.yaml의 surround_view_node 섹션에서 해당 줄만 골라 치환한다.
YAML을 통째로 다시 쓰면 그 파일에 잔뜩 달린 주석이 전부 날아가서, 줄 단위 치환을 쓴다.

화면 글자가 전부 영어인 건 취향이 아니라 제약이다 - cv2.putText의 Hershey 폰트에는
한글 글리프가 없어서 한글을 넣으면 물음표만 찍힌다.
"""

import os
import re
import shutil
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


# 점 순서는 "사각형을 한 바퀴 도는" 순서면 무엇이든 된다. 두 패널에서 같은 순서로만
# 찍으면 대응이 맞기 때문이다. 다만 순서가 꼬여서 자기교차 사각형이 되면
# getPerspectiveTransform이 뒤집힌 워핑을 내놓으므로 볼록성만 검사한다.
POINT_COLORS = ((70, 220, 70), (70, 220, 220), (70, 150, 255), (230, 130, 70))


class RoiPickerNode(Node):
    """Two-pane picker: camera image on the left, top-down canvas on the right."""

    def __init__(self) -> None:
        super().__init__('roi_picker_node')
        self.declare_parameter(
            'topics', ['/camera_2/image_raw', '/camera_3/image_raw',
                       '/camera_4/image_raw'])
        # surround_view_node의 슬롯 이름과 같아야 한다 - 저장할 때 이 이름으로 줄을 찾는다.
        self.declare_parameter('names', ['rear', 'left', 'right'])
        self.declare_parameter('priorities', [30, 10, 10])
        # 빈 값이면 설치된 sensor_fusion_bringup/config/params.yaml을 쓴다.
        self.declare_parameter('params_file', '')
        self.declare_parameter('window_name', 'Surround ROI Picker')
        # 두 패널이 가로로 붙으므로 창 너비는 대략 pane_height * 2.6이 된다.
        # 340이면 906x456 - 이 장비 화면(1024x768)에 들어간다. 더 큰 모니터에서는
        # 올릴수록 점을 정확히 찍을 수 있다(창을 늘려도 클릭 좌표 해상도는 이 값이 정한다).
        self.declare_parameter('pane_height', 340)
        self.declare_parameter('grid_step_px', 50)
        # 확인 모드에서 카메라 패널을 숨기고 캔버스만 크게 볼 때의 높이.
        self.declare_parameter('canvas_only_height', 560)
        # blink 모드가 한 쪽을 보여주는 렌더 프레임 수(15Hz 기준 6이면 약 2.5Hz 교대).
        self.declare_parameter('blink_frames', 6)
        self.declare_parameter('refresh_rate', 15.0)
        self.declare_parameter('max_frame_age', 1.5)
        # params.yaml을 못 읽었을 때만 쓰이는 폴백.
        self.declare_parameter('canvas_width', 800)
        self.declare_parameter('canvas_height', 1000)
        self.declare_parameter('vehicle_rect', [310, 370, 490, 630])
        # 기본값은 캔버스 전체. 빈 리스트로 두면 rclpy가 BYTE_ARRAY로 추론해서
        # 정수 네 개짜리 값을 넣는 순간 타입 에러가 난다.
        self.declare_parameter(
            'view_rect', [0, 0, int(self.get_parameter('canvas_width').value),
                          int(self.get_parameter('canvas_height').value)])

        self.topics = [str(t) for t in self.get_parameter('topics').value]
        self.names = [str(n) for n in self.get_parameter('names').value]
        self.priorities = [int(p) for p in self.get_parameter('priorities').value]
        self.window_name = str(self.get_parameter('window_name').value)
        self.pane_height = int(self.get_parameter('pane_height').value)
        self.grid_step = int(self.get_parameter('grid_step_px').value)
        self.canvas_only_height = int(self.get_parameter('canvas_only_height').value)
        self.blink_frames = max(1, int(self.get_parameter('blink_frames').value))
        self.rate = float(self.get_parameter('refresh_rate').value)
        self.max_age = float(self.get_parameter('max_frame_age').value)

        if len(self.topics) != len(self.names):
            raise ValueError('topics and names must have the same length.')
        if not self.topics:
            raise ValueError('topics must list at least one camera.')
        if len(self.priorities) != len(self.names):
            raise ValueError('priorities must have one entry per camera name.')
        if self.pane_height < 120 or self.grid_step < 5 or self.rate <= 0:
            raise ValueError('pane_height, grid_step_px and refresh_rate are invalid.')

        self.params_file = self._resolve_params_file()
        surround = self._load_surround_section()
        self.canvas_w = int(surround.get(
            'canvas_width', self.get_parameter('canvas_width').value))
        self.canvas_h = int(surround.get(
            'canvas_height', self.get_parameter('canvas_height').value))
        self.vehicle_rect = [int(v) for v in surround.get(
            'vehicle_rect', self.get_parameter('vehicle_rect').value)]
        if self.canvas_w <= 0 or self.canvas_h <= 0:
            raise ValueError('canvas_width/canvas_height must be positive.')

        # dst_points는 언제나 캔버스 전체 좌표로 저장한다(나중에 전방 카메라를 붙여
        # 전체 서라운드로 넓혀도 찍어둔 점이 그대로 유효해야 하므로). 화면에는
        # surround_view_node가 실제로 내보내는 영역만 크게 띄운다 - 지금은 후방 쪽.
        view = [int(v) for v in surround.get(
            'view_rect', self.get_parameter('view_rect').value)]
        self.view_rect = view if len(view) == 4 else [0, 0, self.canvas_w,
                                                      self.canvas_h]
        vx1, vy1, vx2, vy2 = self.view_rect
        if not (0 <= vx1 < vx2 <= self.canvas_w and 0 <= vy1 < vy2 <= self.canvas_h):
            raise ValueError('view_rect must be inside the canvas.')
        self.show_full = False
        # 겹치는 영역을 눈으로 확인하는 방법. 기본 stack은 우선순위가 높은 카메라가
        # 낮은 쪽을 덮어써서, 정작 겹치는 곳에서 얼마나 어긋났는지 안 보인다.
        #   diff  - 겹치는 영역을 |현재 카메라 - 나머지|로 표시. 맞으면 까맣다.
        #   blink - 현재 카메라만 <-> 나머지만 을 번갈아. 어긋나면 화면이 튄다.
        self.check_mode = 'stack'
        self.canvas_only = False
        self._blink_tick = 0

        # 이미 저장돼 있는 값을 시작점으로 불러온다 - 처음부터 다시 찍지 않고
        # 어긋난 점 하나만 끌어다 고칠 수 있어야 실전에서 쓸 만하다.
        self.src: Dict[str, List[List[float]]] = {}
        self.dst: Dict[str, List[List[float]]] = {}
        self._preload(surround)

        self.active = 0
        self.show_warp = True
        self.show_grid = True
        self.status = 'loaded %s' % os.path.basename(self.params_file)
        self.status_bad = False
        # 마우스 콜백이 클릭 좌표를 되돌리려면 마지막으로 그린 배치를 알아야 한다.
        self._layout = {'pane_a_w': 640, 'pane_b_w': 640, 'pane_a_h': 340,
                        'pane_b_h': 340, 'frame_w': 640, 'frame_h': 480}
        # 마지막으로 건드린 점 - h/j/k/l로 1픽셀씩 미세조정할 대상.
        # 패널이 화면에 맞춰 축소돼 있어서 클릭만으로는 원본 1px 정밀도가 안 나온다.
        self._last_edit = None
        # r(전체 지우기)/a(dst 새로 깔기) 직전 상태. 공들여 찍은 네 점이 키 하나
        # 잘못 눌러 날아가면 처음부터 다시 찍어야 해서 되돌릴 구멍을 하나 둔다.
        self._undo = None

        self.bridge = CvBridge()
        self.frames: Dict[str, np.ndarray] = {}
        self.frame_times: Dict[str, float] = {}
        # rclpy Node가 subscriptions 속성을 이미 갖고 있어서 다른 이름을 쓴다.
        self._camera_subscriptions = []
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        for topic in self.topics:
            callback = lambda msg, key=topic: self._image_callback(key, msg)
            self._camera_subscriptions.append(
                self.create_subscription(Image, topic, callback, qos))
            self.get_logger().info(f'subscribing to {topic}')

        # WINDOW_NORMAL이라야 창을 마우스로 늘릴 수 있지만, 대신 처음에 400x300
        # 기본 크기로 뜬다. 실제 배치 폭은 첫 프레임을 받아봐야 알기 때문에
        # 첫 렌더에서 한 번만 맞춰준다.
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self._on_mouse)
        self._window_sized = False
        self.timer = self.create_timer(1.0 / self.rate, self._render)
        self.done = False
        self.get_logger().info(f'params file: {self.params_file}')

    # ------------------------------------------------------------------ params

    def _resolve_params_file(self) -> str:
        path = str(self.get_parameter('params_file').value)
        if not path:
            path = os.path.join(
                get_package_share_directory('sensor_fusion_bringup'),
                'config', 'params.yaml')
        # --symlink-install이라 share 아래 파일은 src를 가리키는 심볼릭 링크다.
        # 실제 경로로 풀어야 저장이 원본 파일에 반영된다.
        return os.path.realpath(path)

    def _load_surround_section(self) -> dict:
        try:
            with open(self.params_file, 'r', encoding='utf-8') as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as error:
            self.get_logger().warning(
                f'cannot read {self.params_file} ({error}); using node defaults')
            return {}
        return (document.get('surround_view_node') or {}).get('ros__parameters') or {}

    def _preload(self, surround: dict) -> None:
        for index, name in enumerate(self.names):
            src = surround.get(f'{name}.src_points')
            dst = surround.get(f'{name}.dst_points')
            normalized = bool(surround.get(f'{name}.normalized_src_points', True))
            self.src[name] = []
            self.dst[name] = []
            if isinstance(src, (list, tuple)) and len(src) == 8:
                # 저장은 정규화 좌표지만 화면에서는 픽셀로 다뤄야 한다. 프레임 크기는
                # 아직 모르니 정규화 상태로 들고 있다가 첫 프레임에서 픽셀로 편다.
                self.src[name] = [[float(src[i]), float(src[i + 1])]
                                  for i in range(0, 8, 2)]
                if not normalized:
                    # 픽셀로 저장돼 있으면 그대로 쓰면 된다.
                    self._src_is_normalized = getattr(self, '_src_is_normalized', {})
                    self._src_is_normalized[name] = False
            if isinstance(dst, (list, tuple)) and len(dst) == 8:
                self.dst[name] = [[float(dst[i]), float(dst[i + 1])]
                                  for i in range(0, 8, 2)]
        self._normalized_pending = {
            name: bool(self.src[name]) for name in self.names}

    def _expand_pending(self, name: str, width: int, height: int) -> None:
        """Turn preloaded normalized src points into pixels once the size is known."""
        if not self._normalized_pending.get(name):
            return
        self._normalized_pending[name] = False
        stored_normalized = getattr(self, '_src_is_normalized', {}).get(name, True)
        if not stored_normalized:
            return
        self.src[name] = [[x * (width - 1), y * (height - 1)]
                          for x, y in self.src[name]]

    # -------------------------------------------------------------------- ROS

    def _image_callback(self, topic: str, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'{topic} conversion failed: {error}')
            return
        self.frames[topic] = frame
        self.frame_times[topic] = self.get_clock().now().nanoseconds / 1e9

    def _live_frame(self, index: int) -> Optional[np.ndarray]:
        topic = self.topics[index]
        frame = self.frames.get(topic)
        if frame is None:
            return None
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.frame_times.get(topic, 0.0) > self.max_age:
            return None
        return frame

    # ------------------------------------------------------------------ mouse

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            return
        pane_a_w = self._layout['pane_a_w']
        pane_b_w = self._layout['pane_b_w']
        name = self.names[self.active]

        if 0 < x < pane_a_w:
            if y >= self._layout['pane_a_h']:
                return                               # 하단 안내 바 - 클릭 무시
            points = self.src[name]
            self._expand_pending(name, self._layout['frame_w'],
                                 self._layout['frame_h'])
            point = [x * self._layout['frame_w'] / pane_a_w,
                     y * self._layout['frame_h'] / self._layout['pane_a_h']]
            pane = 'src'
        elif pane_a_w <= x < pane_a_w + pane_b_w:
            if y >= self._layout['pane_b_h']:
                return
            points = self.dst[name]
            # 화면에는 view_rect 영역만 그려져 있어도, 저장은 항상 캔버스 전체 좌표다.
            region_x1, region_y1, region_x2, region_y2 = self._region()
            point = [region_x1 + (x - pane_a_w) * (region_x2 - region_x1) / pane_b_w,
                     region_y1 + y * (region_y2 - region_y1)
                     / self._layout['pane_b_h']]
            pane = 'dst'
        else:
            return

        if event == cv2.EVENT_RBUTTONDOWN:
            if points:
                points.pop()
                self._last_edit = None
                self._set_status(f'{name}.{pane}: removed point {len(points) + 1}')
            return

        if len(points) < 4:
            points.append(point)
            self._last_edit = (pane, len(points) - 1)
            self._set_status(f'{name}.{pane}: point {len(points)}/4')
        else:
            # 네 점이 다 찍힌 뒤의 클릭은 "가장 가까운 점 옮기기"로 해석한다.
            # 다시 리셋하고 네 번 찍게 만들면 미세조정이 지옥이 된다.
            distances = [(p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2
                         for p in points]
            moved = int(np.argmin(distances))
            points[moved] = point
            self._last_edit = (pane, moved)
            self._set_status(f'{name}.{pane}: moved point {moved + 1}')

    def _set_status(self, text: str, bad: bool = False) -> None:
        self.status = text
        self.status_bad = bad

    # ---------------------------------------------------------------- drawing

    def _region(self):
        """Canvas rectangle currently drawn in the right-hand pane."""
        return ([0, 0, self.canvas_w, self.canvas_h] if self.show_full
                else self.view_rect)

    def _homography(self, name: str):
        if len(self.src[name]) != 4 or len(self.dst[name]) != 4:
            return None
        src = np.asarray(self.src[name], dtype=np.float32)
        dst = np.asarray(self.dst[name], dtype=np.float32)
        if abs(cv2.contourArea(src)) < 1.0 or abs(cv2.contourArea(dst)) < 1.0:
            return None
        try:
            matrix = cv2.getPerspectiveTransform(src, dst)
        except cv2.error:
            return None
        # 보이는 영역만 워핑하도록 평행이동을 미리 곱해둔다(surround_view_node와 동일).
        region_x1, region_y1 = self._region()[:2]
        shift = np.array([[1.0, 0.0, -region_x1],
                          [0.0, 1.0, -region_y1],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
        return shift @ matrix

    def _warp_layer(self, index: int, width: int, height: int):
        """Warped image and coverage mask of one camera, or None if not ready."""
        name = self.names[index]
        frame = self._live_frame(index)
        matrix = self._homography(name)
        if frame is None or matrix is None:
            return None
        warped = cv2.warpPerspective(frame, matrix, (width, height))
        source_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(
            source_mask, np.rint(np.asarray(self.src[name])).astype(np.int32), 255)
        mask = cv2.warpPerspective(source_mask, matrix, (width, height),
                                   flags=cv2.INTER_NEAREST)
        return warped, mask > 0

    @staticmethod
    def _stack(canvas, layers) -> None:
        for warped, mask in layers:
            canvas[mask] = warped[mask]

    def _draw_warps(self, canvas: np.ndarray, width: int, height: int) -> None:
        """Paint the camera warps using the current overlap-checking mode."""
        # 낮은 우선순위부터 덮어써야 후방(우선순위 높음)이 겹치는 부분을 차지한다.
        # surround_view_node의 priority 블렌딩과 같은 순서다.
        order = sorted(range(len(self.names)), key=lambda i: self.priorities[i])
        layers = {index: self._warp_layer(index, width, height) for index in order}
        # 프레임이 아직 안 왔거나 점이 덜 찍힌 카메라는 빠진다 - 아래에서 order로
        # 다시 훑을 때 빠진 키를 그대로 찾으면 KeyError가 난다.
        ready = [index for index in order if layers.get(index) is not None]
        if not ready:
            return

        active = layers.get(self.active)
        others = [layers[index] for index in ready if index != self.active]

        if self.check_mode == 'stack' or active is None or not others:
            self._stack(canvas, [layers[index] for index in ready])
            return

        other_canvas = np.zeros_like(canvas)
        other_mask = np.zeros(canvas.shape[:2], dtype=bool)
        for warped, mask in others:
            other_canvas[mask] = warped[mask]
            other_mask |= mask
        active_warp, active_mask = active

        if self.check_mode == 'blink':
            # 두 장을 번갈아 보여주면 어긋난 만큼 무늬가 튄다. 정지 화면 두 장을
            # 나란히 보는 것보다 사람 눈이 훨씬 민감하게 잡아낸다.
            self._blink_tick += 1
            if (self._blink_tick // self.blink_frames) % 2:
                canvas[active_mask] = active_warp[active_mask]
            else:
                canvas[other_mask] = other_canvas[other_mask]
            return

        # diff: 겹치는 곳은 차이만, 나머지는 어둡게 깔아 배경으로 둔다.
        overlap = active_mask & other_mask
        canvas[other_mask] = (other_canvas[other_mask] * 0.35).astype(np.uint8)
        only_active = active_mask & ~other_mask
        canvas[only_active] = (active_warp[only_active] * 0.35).astype(np.uint8)
        if overlap.any():
            # 2배로 키워야 몇 픽셀짜리 어긋남도 눈에 띈다. 맞으면 까맣게 남는다.
            difference = cv2.absdiff(active_warp, other_canvas)
            canvas[overlap] = np.minimum(
                difference[overlap].astype(np.int32) * 2, 255).astype(np.uint8)

    def _canvas(self) -> np.ndarray:
        region_x1, region_y1, region_x2, region_y2 = self._region()
        width, height = region_x2 - region_x1, region_y2 - region_y1
        canvas = np.full((height, width, 3), 30, dtype=np.uint8)

        if self.show_warp:
            self._draw_warps(canvas, width, height)

        if self.show_grid:
            for x in range(region_x1 - region_x1 % self.grid_step, region_x2,
                           self.grid_step):
                cv2.line(canvas, (x - region_x1, 0), (x - region_x1, height - 1),
                         (70, 70, 70), 1)
            for y in range(region_y1 - region_y1 % self.grid_step, region_y2,
                           self.grid_step):
                cv2.line(canvas, (0, y - region_y1), (width - 1, y - region_y1),
                         (70, 70, 70), 1)

        x1, y1, x2, y2 = [v - s for v, s in zip(
            self.vehicle_rect, (region_x1, region_y1, region_x1, region_y1))]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (55, 55, 55), -1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (200, 200, 200), 2)
        cv2.putText(canvas, 'CAR', (x1 + 10, y2 - 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (230, 230, 230), 2)

        # 전체 캔버스를 볼 때는 실제 출력 영역(view_rect)을 표시해준다.
        if self.show_full:
            cv2.rectangle(canvas, (self.view_rect[0], self.view_rect[1]),
                          (self.view_rect[2] - 1, self.view_rect[3] - 1),
                          (0, 200, 255), 2)

        # 현재 카메라가 아닌 카메라의 사각형도 흐리게 그려서 서로 어떻게 겹치는지 보여준다.
        offset = np.array([region_x1, region_y1], dtype=np.float32)
        for index, name in enumerate(self.names):
            if len(self.dst[name]) != 4:
                continue
            polygon = np.rint(np.asarray(self.dst[name]) - offset).astype(np.int32)
            color = (110, 110, 110) if index != self.active else (255, 255, 255)
            cv2.polylines(canvas, [polygon], True, color,
                          2 if index == self.active else 1)
        return canvas

    def _draw_points(self, pane: np.ndarray, points: Sequence[Sequence[float]],
                     scale_x: float, scale_y: float,
                     offset: Tuple[float, float] = (0.0, 0.0)) -> None:
        scaled = [(int(round((x - offset[0]) * scale_x)),
                   int(round((y - offset[1]) * scale_y)))
                  for x, y in points]
        if len(scaled) == 4:
            cv2.polylines(pane, [np.asarray(scaled, dtype=np.int32)], True,
                          (255, 255, 255), 2)
        for index, (x, y) in enumerate(scaled):
            color = POINT_COLORS[index % len(POINT_COLORS)]
            cv2.circle(pane, (x, y), 7, (0, 0, 0), -1)
            cv2.circle(pane, (x, y), 6, color, -1)
            cv2.putText(pane, str(index + 1), (x + 9, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(pane, str(index + 1), (x + 9, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    def _info_bar(self, width: int) -> np.ndarray:
        name = self.names[self.active]
        bar = np.full((128, width, 3), 22, dtype=np.uint8)
        tabs = ' '.join(f'[{i + 1}]{n}{"*" if i == self.active else ""}'
                        for i, n in enumerate(self.names))
        counts = ' '.join(f'{n}:{len(self.src[n])}/{len(self.dst[n])}'
                          for n in self.names)
        status, status_bad = self.status, self.status_bad
        if len(self.src[name]) == 4 and not cv2.isContourConvex(
                np.rint(np.asarray(self.src[name])).astype(np.int32)):
            status = 'src quad is self-crossing - reorder the points'
            status_bad = True
        lines = (
            (f'{tabs}   src/dst {counts}', (90, 200, 255)),
            ('L-click add/move   R-click undo   h j k l nudge 1px',
             (200, 200, 200)),
            ('[d]check-mode [x]big-canvas [w]warp [g]grid [f]full-canvas',
             (90, 200, 255)),
            ('[1-3]cam [Tab]next [r]reset [z]undo [a]auto-dst [s]SAVE [q]quit',
             (200, 200, 200)),
            (status, (80, 80, 255) if status_bad else (120, 230, 120)),
        )
        for index, (line, color) in enumerate(lines):
            cv2.putText(bar, line, (10, 22 + index * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        return bar

    def _render(self) -> None:
        name = self.names[self.active]
        region_x1, region_y1, region_x2, region_y2 = self._region()
        region_w, region_h = region_x2 - region_x1, region_y2 - region_y1

        # 겹침을 확인할 때는 카메라 패널을 접고 캔버스만 크게 본다. 1024x768 화면에서
        # 두 패널을 나란히 놓으면 캔버스가 너무 작아서 몇 픽셀 어긋남이 안 보인다.
        pane_b_h = self.canvas_only_height if self.canvas_only else self.pane_height
        pane_b_w = max(1, int(round(region_w * pane_b_h / region_h)))
        pane_b = cv2.resize(self._canvas(), (pane_b_w, pane_b_h))
        self._draw_points(pane_b, self.dst[name], pane_b_w / region_w,
                          pane_b_h / region_h, (region_x1, region_y1))
        label = (f'canvas {self.canvas_w}x{self.canvas_h}' if self.show_full
                 else f'view {region_x1},{region_y1}-{region_x2},{region_y2}')
        self._caption(pane_b, f'{label}   [{self.check_mode}]')

        if self.canvas_only:
            self._layout.update({'pane_a_w': 0, 'pane_b_w': pane_b_w,
                                 'pane_b_h': pane_b_h})
            top = pane_b
        else:
            frame = self._live_frame(self.active)
            if frame is None:
                frame = np.full((self._layout['frame_h'],
                                 self._layout['frame_w'], 3), 45, dtype=np.uint8)
                cv2.putText(frame, 'NO SIGNAL', (40, frame.shape[0] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (120, 120, 120), 3)
            else:
                self._layout['frame_h'], self._layout['frame_w'] = frame.shape[:2]
                self._expand_pending(name, frame.shape[1], frame.shape[0])

            frame_h, frame_w = frame.shape[:2]
            pane_a_w = max(1, int(round(frame_w * pane_b_h / frame_h)))
            pane_a = cv2.resize(frame, (pane_a_w, pane_b_h))
            self._draw_points(pane_a, self.src[name], pane_a_w / frame_w,
                              pane_b_h / frame_h)
            self._caption(pane_a, f'{name}  {self.topics[self.active]}')
            self._layout.update({'pane_a_w': pane_a_w, 'pane_a_h': pane_b_h,
                                 'pane_b_w': pane_b_w, 'pane_b_h': pane_b_h})
            top = np.hstack((pane_a, pane_b))

        view = np.vstack((top, self._info_bar(top.shape[1])))
        if not self._window_sized:
            cv2.resizeWindow(self.window_name, view.shape[1], view.shape[0])
            self._window_sized = True
        cv2.imshow(self.window_name, view)
        self._handle_key(cv2.waitKey(1) & 0xFF)

    @staticmethod
    def _caption(pane: np.ndarray, text: str) -> None:
        cv2.putText(pane, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4)
        cv2.putText(pane, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1)

    # ---------------------------------------------------------------- keyboard

    def _handle_key(self, key: int) -> None:
        if key in (255, -1):
            return
        name = self.names[self.active]
        if key in (ord('q'), 27):
            self.done = True
        elif key == 9:                                   # Tab
            self.active = (self.active + 1) % len(self.names)
        elif ord('1') <= key <= ord('9'):
            index = key - ord('1')
            if index < len(self.names):
                self.active = index
        elif key == ord('r'):
            self._snapshot(name)
            self.src[name] = []
            self.dst[name] = []
            self._last_edit = None
            self._set_status(f'{name}: cleared - [z] to undo')
        elif key == ord('z'):
            self._undo_last()
        elif key == ord('w'):
            self.show_warp = not self.show_warp
            self._set_status(f'warp preview {"on" if self.show_warp else "off"}')
        elif key == ord('g'):
            self.show_grid = not self.show_grid
        elif key == ord('d'):
            modes = ('stack', 'diff', 'blink')
            self.check_mode = modes[(modes.index(self.check_mode) + 1) % len(modes)]
            self._set_status({
                'stack': 'stack: priority order, overlap hidden by the winner',
                'diff': 'diff: overlap shows |active - others|, black = aligned',
                'blink': 'blink: active <-> others, jumping = misaligned',
            }[self.check_mode])
        elif key == ord('x'):
            self.canvas_only = not self.canvas_only
            self._window_sized = False
            self._set_status('canvas only' if self.canvas_only
                             else 'camera + canvas')
        elif key == ord('f'):
            # 전체 캔버스 <-> 실제 출력 영역. 폭이 달라지므로 창 크기도 다시 맞춘다.
            self.show_full = not self.show_full
            self._window_sized = False
            self._set_status('showing '
                             + ('full canvas' if self.show_full else 'view_rect'))
        elif key == ord('a'):
            self._snapshot(name)
            self._auto_dst(name)
        elif key in (ord('h'), ord('j'), ord('k'), ord('l')):
            self._nudge(name, key)
        elif key == ord('s'):
            self._save()

    def _snapshot(self, name: str) -> None:
        """Remember one camera's points so the next destructive key can be undone."""
        self._undo = (name, [list(p) for p in self.src[name]],
                      [list(p) for p in self.dst[name]])

    def _undo_last(self) -> None:
        if self._undo is None:
            self._set_status('nothing to undo', bad=True)
            return
        name, src, dst = self._undo
        # 되돌리기 자체도 되돌릴 수 있게 현재 상태와 맞바꾼다.
        self._undo = (name, [list(p) for p in self.src[name]],
                      [list(p) for p in self.dst[name]])
        self.src[name], self.dst[name] = src, dst
        self._last_edit = None
        self.active = self.names.index(name)
        self._set_status(f'{name}: restored {len(src)} src / {len(dst)} dst points')

    def _nudge(self, name: str, key: int) -> None:
        """Move the last touched point by one source pixel."""
        if self._last_edit is None:
            self._set_status('nothing to nudge - click a point first', bad=True)
            return
        pane, index = self._last_edit
        points = self.src[name] if pane == 'src' else self.dst[name]
        if index >= len(points):
            self._last_edit = None
            return
        step = {ord('h'): (-1, 0), ord('l'): (1, 0),
                ord('k'): (0, -1), ord('j'): (0, 1)}[key]
        points[index][0] += step[0]
        points[index][1] += step[1]
        self._set_status(f'{name}.{pane} point {index + 1} -> '
                         f'({points[index][0]:.0f}, {points[index][1]:.0f})')

    def _auto_dst(self, name: str) -> None:
        """Drop a starter destination quad so there is something to drag around."""
        margin = 40
        region_x1, region_y1, region_x2, region_y2 = self._region()
        thirds = (region_x2 - region_x1) / 3.0
        index = self.names.index(name)
        left_x = region_x1 + margin + index * (thirds - margin / 2)
        right_x = min(region_x2 - margin, left_x + thirds)
        top_y = max(self.vehicle_rect[3], region_y1 + margin)
        bottom_y = region_y2 - margin
        self.dst[name] = [[left_x, bottom_y], [right_x, bottom_y],
                          [right_x, top_y], [left_x, top_y]]
        self._set_status(f'{name}.dst: starter quad placed - drag the points')

    # ------------------------------------------------------------------ saving

    @staticmethod
    def _format(values: Sequence[float], digits: int) -> str:
        return '[' + ', '.join(f'{v:.{digits}f}' for v in values) + ']'

    def _save(self) -> None:
        replacements = {}
        saved = []
        for name in self.names:
            if len(self.src[name]) != 4 or len(self.dst[name]) != 4:
                continue
            width, height = self._layout['frame_w'], self._layout['frame_h']
            normalized = [v for x, y in self.src[name]
                          for v in (x / max(width - 1, 1), y / max(height - 1, 1))]
            flat_dst = [v for point in self.dst[name] for v in point]
            replacements[f'{name}.src_points'] = self._format(normalized, 4)
            replacements[f'{name}.dst_points'] = self._format(flat_dst, 1)
            replacements[f'{name}.normalized_src_points'] = 'true'
            saved.append(name)

        if not replacements:
            self._set_status('nothing to save - each camera needs 4 src + 4 dst',
                             bad=True)
            return

        try:
            with open(self.params_file, 'r', encoding='utf-8') as handle:
                lines = handle.readlines()
        except OSError as error:
            self._set_status(f'cannot read params.yaml: {error}', bad=True)
            return

        # YAML을 파싱해서 다시 쓰면 주석이 전부 날아간다. 해당 키의 줄만 치환한다.
        remaining = dict(replacements)
        for index, line in enumerate(lines):
            match = re.match(r'^(\s*)([a-z_]+\.[a-z_]+):\s', line)
            if match and match.group(2) in remaining:
                key = match.group(2)
                lines[index] = f'{match.group(1)}{key}: {remaining.pop(key)}\n'
        if remaining:
            self._set_status(
                'missing keys in params.yaml: ' + ', '.join(sorted(remaining)),
                bad=True)
            return

        try:
            shutil.copyfile(self.params_file, self.params_file + '.bak')
            with open(self.params_file, 'w', encoding='utf-8') as handle:
                handle.writelines(lines)
        except OSError as error:
            self._set_status(f'cannot write params.yaml: {error}', bad=True)
            return

        snippet = '\n'.join(f'    {key}: {value}'
                            for key, value in sorted(replacements.items()))
        self.get_logger().info(
            f'saved {", ".join(saved)} to {self.params_file}\n{snippet}')
        self._set_status(f'saved {", ".join(saved)} -> {self.params_file} '
                         f'(backup: params.yaml.bak)')

    def destroy_node(self) -> bool:
        cv2.destroyWindow(self.window_name)
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = RoiPickerNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
