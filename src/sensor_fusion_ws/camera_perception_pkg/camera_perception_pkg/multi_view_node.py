"""Tile several camera topics into a single window (no warping, no stitching).

surround_view_node와 달리 지면 투영/블렌딩을 하지 않는다. 카메라가 살아있는지,
어느 포트가 어느 화면인지 눈으로 확인하려는 용도라 원본을 그대로 격자로 붙인다.
프레임이 안 오는 카메라는 회색 타일에 NO SIGNAL로 표시해서, 창이 아예 안 뜨는 것과
카메라 하나만 죽은 것을 구분할 수 있게 했다(USB 전원 문제로 개별 카메라가 수시로
떨어지는 이력이 있어서 이 구분이 중요하다).
"""

from typing import Dict, List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class MultiViewNode(Node):
    """Subscribe to N image topics and publish/show them as one tiled canvas."""

    def __init__(self) -> None:
        super().__init__('multi_view_node')
        self.declare_parameter('topics', [
            '/camera_2/image_raw', '/camera_3/image_raw', '/camera_4/image_raw'])
        self.declare_parameter('labels', ['camera_2', 'camera_3', 'camera_4'])
        self.declare_parameter('output_topic', '/multi_view/image')
        self.declare_parameter('window_name', 'Multi Camera View')
        self.declare_parameter('show_preview', True)
        self.declare_parameter('resizable_window', True)
        self.declare_parameter('tile_width', 640)
        self.declare_parameter('tile_height', 480)
        self.declare_parameter('columns', 0)      # 0이면 카메라 수에 맞춰 자동
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_frame_age', 1.0)

        self.topics: List[str] = [str(t) for t in self.get_parameter('topics').value]
        labels = [str(l) for l in self.get_parameter('labels').value]
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)
        self.resizable = bool(self.get_parameter('resizable_window').value)
        self.tile_w = int(self.get_parameter('tile_width').value)
        self.tile_h = int(self.get_parameter('tile_height').value)
        self.columns = int(self.get_parameter('columns').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.max_age = float(self.get_parameter('max_frame_age').value)

        if not self.topics:
            raise ValueError('topics must list at least one image topic.')
        if self.tile_w <= 0 or self.tile_h <= 0 or self.rate <= 0:
            raise ValueError('tile_width, tile_height and publish_rate must be positive.')
        if self.max_age <= 0:
            raise ValueError('max_frame_age must be positive.')
        if self.columns < 0:
            raise ValueError('columns must be zero (auto) or positive.')

        # 라벨이 모자라면 토픽 이름으로 채운다 - 개수 안 맞다고 죽을 이유는 없다.
        self.labels = [labels[i] if i < len(labels) else self.topics[i]
                       for i in range(len(self.topics))]
        if self.columns == 0:
            self.columns = len(self.topics) if len(self.topics) <= 3 else 2

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

        if self.show_preview and self.resizable:
            # imshow의 기본값은 WINDOW_AUTOSIZE라 창이 캔버스 크기에 고정돼서 마우스로
            # 못 늘린다. WINDOW_NORMAL이어야 모서리를 끌어 조절할 수 있다.
            # 처음 뜨는 크기는 타일 배치 그대로 잡아준다.
            rows = -(-len(self.topics) // self.columns)   # 올림 나눗셈
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.window_name, self.tile_w * self.columns, self.tile_h * rows)

        self.publisher = self.create_publisher(Image, self.output_topic, qos)
        self.timer = self.create_timer(1.0 / self.rate, self._compose_and_publish)

    def _image_callback(self, topic: str, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().error(f'{topic} conversion failed: {error}')
            return
        self.frames[topic] = frame
        self.frame_times[topic] = self.get_clock().now().nanoseconds / 1e9

    def _tile(self, topic: str, label: str, now: float) -> np.ndarray:
        frame = self.frames.get(topic)
        age = now - self.frame_times.get(topic, 0.0)
        live = frame is not None and age <= self.max_age

        if live:
            tile = cv2.resize(frame, (self.tile_w, self.tile_h))
            status, color = f'{label}', (60, 220, 60)
        else:
            tile = np.full((self.tile_h, self.tile_w, 3), 40, dtype=np.uint8)
            reason = 'NO SIGNAL' if frame is None else f'STALE {age:.1f}s'
            status, color = f'{label}  [{reason}]', (60, 60, 220)
            cv2.putText(tile, reason, (self.tile_w // 2 - 90, self.tile_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2)

        cv2.rectangle(tile, (0, 0), (self.tile_w - 1, self.tile_h - 1), color, 2)
        cv2.putText(tile, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(tile, status, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return tile

    def _compose_and_publish(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        tiles = [self._tile(topic, label, now)
                 for topic, label in zip(self.topics, self.labels)]

        # 마지막 줄이 비면 빈 타일로 채워야 hstack이 된다.
        rows = []
        for start in range(0, len(tiles), self.columns):
            row = tiles[start:start + self.columns]
            while len(row) < self.columns:
                row.append(np.zeros((self.tile_h, self.tile_w, 3), dtype=np.uint8))
            rows.append(np.hstack(row))
        canvas = np.vstack(rows)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8'))
        if self.show_preview:
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if self.show_preview:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
