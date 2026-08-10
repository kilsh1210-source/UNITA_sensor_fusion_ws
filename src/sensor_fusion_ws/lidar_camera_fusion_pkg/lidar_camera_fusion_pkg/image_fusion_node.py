#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import cv2
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_msgs.msg import Header

from interfaces_pkg.msg import DetectionArray


WINDOW_NAME = 'Fusion Visualizer'

# 숫자 키: 주 화면 전환 / q,w,e,r 키: 보조 화면(분할뷰) 전환
MODE_KEYS = {
    ord('1'): 'raw',
    ord('2'): 'lidar',
    ord('3'): 'boxes',
    ord('4'): 'bev',
}
SECONDARY_KEYS = {
    ord('q'): 'raw',
    ord('w'): 'lidar',
    ord('e'): 'boxes',
    ord('r'): 'bev',
}
MODE_LABELS = {
    'raw': 'Raw Camera',
    'lidar': 'LiDAR Points',
    'boxes': 'Bounding Boxes',
    'bev': "Bird's Eye View (TODO)",
}


class FusionVisualizerNode(Node):
    """
    - 동기화(message_filters) 제거: 이미지 콜백이 오면 무조건 화면 출력
    - 최신 LaserScan / DetectionArray가 있으면 오버레이
    - QoS는 sensor_data(BEST_EFFORT)로 고정 -> 카메라/라이다와 호환성 최대화
    - 창 1개 + 숫자키(1~4)로 화면 모드 전환 (raw / lidar / boxes / bev)
    - v: 분할뷰 토글, q/w/e/r: 분할뷰의 보조 화면 선택
    """

    def __init__(self):
        super().__init__('fusion_visualizer_node')

        # -------------------------
        # Topics
        # -------------------------
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('det_topic', '/detections')

        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('annotated_topic', '/fusion/annotated_image')
        self.declare_parameter('display', True)
        self.declare_parameter('window_width', 960)
        self.declare_parameter('window_height', 720)

        # -------------------------
        # Intrinsic
        # -------------------------
        self.declare_parameter('fx', 565.529459)
        self.declare_parameter('fy', 566.767111)
        self.declare_parameter('cx', 337.983746)
        self.declare_parameter('cy', 290.095566)

        # -------------------------
        # Extrinsic (LiDAR -> Camera)
        # -------------------------
        self.declare_parameter('cam_x_offset', 0.0)
        self.declare_parameter('cam_height', 0.032)
        self.declare_parameter('use_urdf_extrinsic', True)
        self.declare_parameter('lidar_frame_id', 'laser')
        self.declare_parameter('camera_frame_id', 'camera_link')

        # -------------------------
        # LiDAR filtering / projection
        # -------------------------
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('min_cam_z', 0.1)

        self.declare_parameter('enable_fov_filter', True)
        self.declare_parameter('cam_fov_deg', 55.0)
        self.declare_parameter('front_angle_deg', 180.0)

        self.declare_parameter('point_stride', 1)       # 라이다 점 샘플링 간격 (1이면 전부 표시)
        self.declare_parameter('draw_all_points', True) # 카메라 이미지 위에 라이다 포인트를 전부 표시할지 여부
        self.declare_parameter('display_mode', 'boxes')  # 시작 화면 모드: raw / lidar / boxes / bev (창에서 숫자키 1~4로 전환)
        self.declare_parameter('distance_method', 'center')  # min / p20 / median / center
        self.declare_parameter('distance_tolerance', 0.6)    # 거리 오차 허용 범위 [m]

        # 최신 데이터 유효 시간(초): 너무 오래된 scan/det는 무시
        self.declare_parameter('max_age_scan', 0.5)
        # CPU에서 2개 모델을 순차 추론하다 보니 프레임당 지연이 들쭉날쭉할 수 있음.
        # 너무 짧으면 추론이 살짝 느려질 때마다 박스가 깜빡이므로 여유 있게 잡음.
        self.declare_parameter('max_age_det', 1.5)

        # Load params
        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.det_topic = self.get_parameter('det_topic').value

        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.display = bool(self.get_parameter('display').value)
        self.window_width = int(self.get_parameter('window_width').value)
        self.window_height = int(self.get_parameter('window_height').value)

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        cx = float(self.get_parameter('cx').value)
        cy = float(self.get_parameter('cy').value)
        self.K = np.array([[fx, 0.0, cx],
                           [0.0, fy, cy],
                           [0.0, 0.0, 1.0]], dtype=np.float64)

        self.use_urdf_extrinsic = bool(self.get_parameter('use_urdf_extrinsic').value)
        self.lidar_frame_id = str(self.get_parameter('lidar_frame_id').value)
        self.camera_frame_id = str(self.get_parameter('camera_frame_id').value)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

        dist = float(self.get_parameter('cam_x_offset').value)
        height = float(self.get_parameter('cam_height').value)
        self.extrinsic_mat = self._init_extrinsic(dist, height, self.front_angle_deg)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.min_cam_z = float(self.get_parameter('min_cam_z').value)

        self.enable_fov_filter = bool(self.get_parameter('enable_fov_filter').value)
        self.fov_deg = float(self.get_parameter('cam_fov_deg').value)
        self.fov_center_rad = math.radians(self.front_angle_deg)

        self.point_stride = int(self.get_parameter('point_stride').value)
        self.draw_all_points = bool(self.get_parameter('draw_all_points').value)
        self.mode = str(self.get_parameter('display_mode').value).lower().strip()
        if self.mode not in MODE_LABELS:
            self.get_logger().warn(f"Unknown display_mode '{self.mode}', falling back to 'boxes'")
            self.mode = 'boxes'
        self.secondary = 'lidar' if self.mode != 'lidar' else 'boxes'
        self.split_view = False
        self.distance_method = str(self.get_parameter('distance_method').value).lower().strip()
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)

        self.max_age_scan = float(self.get_parameter('max_age_scan').value)
        self.max_age_det = float(self.get_parameter('max_age_det').value)

        self.bridge = CvBridge()

        # 최신 메시지 버퍼
        self.last_scan: Optional[LaserScan] = None
        self.last_scan_time = None

        self.last_det: Optional[DetectionArray] = None
        self.last_det_time = None

        self.last_img_time = None

        # Subscribers (QoS: sensor_data로 고정)
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(DetectionArray, self.det_topic, self.det_cb, 10)  # det는 reliable여도 수신 가능

        # Publisher
        if self.publish_annotated:
            self.pub_img = self.create_publisher(Image, self.annotated_topic, 10)
        else:
            self.pub_img = None

        if self.display:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW_NAME, self.window_width, self.window_height)

        # 디버그 타이머: 데이터 미수신 상태를 주기적으로 알려줌
        self.create_timer(1.0, self.debug_timer)

        self.get_logger().info(
            "1:raw 2:lidar 3:boxes 4:bev (주화면) | q/w/e/r: 보조화면 | "
            "v: 분할보기 토글 - 창을 클릭한 뒤 키 입력\n"
            f"FusionVisualizerNode started.\n"
            f"  image_topic={self.image_topic}\n"
            f"  scan_topic={self.scan_topic}\n"
            f"  det_topic={self.det_topic}\n"
            f"  publish_annotated={self.publish_annotated} ({self.annotated_topic})\n"
        )

    def _try_update_extrinsic_from_tf(self) -> None:
        if not self.use_urdf_extrinsic:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame_id,
                self.lidar_frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        rot_mat = self._quaternion_to_matrix(quat)
        t_vec = np.array([translation.x, translation.y, translation.z], dtype=np.float64)

        ext = np.eye(4, dtype=np.float64)
        ext[:3, :3] = rot_mat
        ext[:3, 3] = t_vec
        self.extrinsic_mat = ext

    @staticmethod
    def _quaternion_to_matrix(q) -> np.ndarray:
        x, y, z, w = q
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    @staticmethod
    def _init_extrinsic(dist: float, height: float, front_angle_deg: float) -> np.ndarray:
        t_vec = np.array([0.0, height, dist], dtype=np.float64).reshape(3, 1)

        R_axis_swap = np.array([
            [0.0, -1.0, 0.0],
            [0.0,  0.0, -1.0],
            [1.0,  0.0, 0.0]
        ], dtype=np.float64)

        # front_angle_deg 값 하나로 yaw 회전을 생성 (FOV 필터와 동일한 기준 공유)
        theta = math.radians(front_angle_deg)
        R_yaw = np.array([
            [math.cos(theta), -math.sin(theta), 0.0],
            [math.sin(theta),  math.cos(theta), 0.0],
            [0.0,              0.0,             1.0]
        ], dtype=np.float64)

        R = R_axis_swap @ R_yaw

        ext = np.eye(4, dtype=np.float64)
        ext[:3, :3] = R
        ext[:3, 3] = t_vec.flatten()
        return ext

    def debug_timer(self):
        now = self.get_clock().now()
        def age(t):
            if t is None:
                return None
            return (now - t).nanoseconds / 1e9

        img_age = age(self.last_img_time)
        scan_age = age(self.last_scan_time)
        det_age = age(self.last_det_time)

        # 이미지가 아예 안 들어오면 검정 화면 고정입니다.
        if img_age is None or img_age > 1.0:
            self.get_logger().warn(
                f"[No Image] image_topic='{self.image_topic}' is not arriving. "
                f"scan_age={scan_age}, det_age={det_age}"
            )

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def det_cb(self, msg: DetectionArray):
        # yolov8_node는 탐지가 하나도 없는 프레임에도 빈 DetectionArray를 계속 발행한다.
        # 여기서 그걸 그대로 반영하면 박스가 매 프레임 깜빡이므로, 빈 메시지는 무시하고
        # max_age_det 타임아웃이 지났을 때만 박스가 사라지게 한다.
        if len(msg.detections) == 0:
            return
        self.last_det = msg
        self.last_det_time = self.get_clock().now()

    def image_cb(self, img_msg: Image):
        self.last_img_time = self.get_clock().now()

        # 1) image to cv2
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"imgmsg_to_cv2 failed: {e}")
            return

        h, w = img.shape[:2]

        self._try_update_extrinsic_from_tf()

        # 2) 최신 scan/det가 유효한지 확인
        now = self.get_clock().now()

        scan_ok = False
        det_ok = False

        if self.last_scan is not None and self.last_scan_time is not None:
            scan_age = (now - self.last_scan_time).nanoseconds / 1e9
            scan_ok = (scan_age <= self.max_age_scan)

        if self.last_det is not None and self.last_det_time is not None:
            det_age = (now - self.last_det_time).nanoseconds / 1e9
            det_ok = (det_age <= self.max_age_det)

        # 3) scan -> projected points (lidar/boxes 모드에서 사용)
        needed_modes = {self.mode} | ({self.secondary} if self.split_view else set())

        u_pix = np.array([], dtype=np.int32)
        v_pix = np.array([], dtype=np.int32)
        ranges = np.array([], dtype=np.float64)

        if scan_ok and (needed_modes & {'lidar', 'boxes'}):
            u_pix, v_pix, ranges = self.project_scan_to_image(self.last_scan, w, h)

        # 4) 화면 모드별 프레임 생성 (분할뷰면 주+보조 2장, 아니면 주화면 1장)
        display = self._render_mode(self.mode, img, w, h, scan_ok, det_ok, u_pix, v_pix, ranges)

        if self.split_view:
            secondary_frame = self._render_mode(
                self.secondary, img, w, h, scan_ok, det_ok, u_pix, v_pix, ranges)
            display = cv2.hconcat([display, secondary_frame])

        # 5) show / publish
        if self.display:
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(1) & 0xFF
            if key in MODE_KEYS:
                self.mode = MODE_KEYS[key]
                self.get_logger().info(f'주화면: {self.mode}')
            elif key in SECONDARY_KEYS:
                self.secondary = SECONDARY_KEYS[key]
                self.get_logger().info(f'보조화면: {self.secondary}')
            elif key == ord('v'):
                self.split_view = not self.split_view
                self.get_logger().info(f'분할보기: {self.split_view}')

        if self.pub_img is not None:
            out_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            out_msg.header = Header()
            out_msg.header.stamp = img_msg.header.stamp
            out_msg.header.frame_id = img_msg.header.frame_id
            self.pub_img.publish(out_msg)

    def _render_mode(self, mode, img, w, h, scan_ok, det_ok, u_pix, v_pix, ranges) -> np.ndarray:
        if mode == 'raw':
            frame = img.copy()

        elif mode == 'lidar':
            frame = img.copy()
            if scan_ok:
                self.draw_projected_points(frame, u_pix, v_pix)

        elif mode == 'boxes':
            frame = img.copy()
            if det_ok:
                for det in self.last_det.detections:
                    class_name = getattr(det, 'class_name', 'Unknown')
                    score = float(getattr(det, 'score', 0.0))

                    bbox = det.bbox
                    box_cx = float(bbox.center.position.x)
                    box_cy = float(bbox.center.position.y)
                    bw = float(bbox.size.x)
                    bh = float(bbox.size.y)

                    x1 = int(box_cx - bw / 2.0)
                    y1 = int(box_cy - bh / 2.0)
                    x2 = int(box_cx + bw / 2.0)
                    y2 = int(box_cy + bh / 2.0)

                    x1c = max(0, min(w - 1, x1))
                    y1c = max(0, min(h - 1, y1))
                    x2c = max(0, min(w - 1, x2))
                    y2c = max(0, min(h - 1, y2))

                    dist_m, best_uv = (None, None)
                    if scan_ok and len(ranges) > 0:
                        dist_m, best_uv = self.estimate_distance_in_bbox(
                            u_pix, v_pix, ranges, x1c, y1c, x2c, y2c
                        )

                    color = (0, 255, 0)

                    if dist_m is None:
                        text = f"{class_name} {score:.2f}  dist:N/A"
                    else:
                        text = f"{class_name} {score:.2f}  dist:{dist_m:.2f}m"

                    self._draw_box_with_label(frame, x1c, y1c, x2c, y2c, color, text, best_uv)

        else:  # bev - 추후 구현 전까지는 자리만
            frame = self._bev_placeholder(img.shape)

        cv2.putText(frame, MODE_LABELS[mode], (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    @staticmethod
    def _bev_placeholder(shape) -> np.ndarray:
        h, w = shape[:2]
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(frame, "Bird's Eye View - TODO", (30, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        return frame

    def draw_projected_points(self, img, u_pix, v_pix):
        if len(u_pix) == 0:
            return

        stride = max(1, self.point_stride)
        for idx in range(0, len(u_pix), stride):
            if not self.draw_all_points and idx != 0:
                continue

            u = int(u_pix[idx])
            v = int(v_pix[idx])
            cv2.circle(img, (u, v), 2, (0, 0, 255), -1)

    def _draw_box_with_label(self, img, x1, y1, x2, y2, color, text, best_uv):
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        label_x, label_y = self._compute_distance_label_position(
            x1, y1, x2, y2, text_size[0], text_size[1], img.shape[1], img.shape[0]
        )

        cv2.rectangle(
            img,
            (label_x - 2, label_y - text_size[1] - 2),
            (label_x + text_size[0] + 2, label_y + 2),
            (0, 0, 0),
            -1,
        )
        cv2.putText(img, text, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        if best_uv is not None:
            cv2.circle(img, best_uv, 4, (255, 255, 255), -1)

    def _compute_distance_label_position(self, x1, y1, x2, y2, text_w, text_h, img_w, img_h):
        margin_x = 6
        margin_y = 8

        # label_y is the text baseline; the background box spans
        # [label_y - text_h - 2, label_y + 2], so offset by text_h to keep
        # the label inside the bbox (below its top edge) instead of above it.
        label_y = y1 + margin_y + text_h + 2
        label_y = max(text_h + 2, min(label_y, img_h - 3))

        label_x = x2 - text_w - margin_x
        label_x = max(margin_x, min(label_x, img_w - text_w - margin_x))

        return label_x, label_y

    def project_scan_to_image(self, scan_msg: LaserScan, img_w: int, img_h: int):
        ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
        n = ranges.shape[0]
        angles = scan_msg.angle_min + np.arange(n, dtype=np.float64) * scan_msg.angle_increment

        finite = np.isfinite(ranges)
        valid = finite & (ranges >= self.min_range) & (ranges <= min(float(scan_msg.range_max), self.max_range))

        if self.enable_fov_filter:
            half_fov = math.radians(self.fov_deg / 2.0)
            angle_diff = np.abs(np.arctan2(np.sin(angles - self.fov_center_rad), np.cos(angles - self.fov_center_rad)))
            valid = valid & (angle_diff <= half_fov)

        if np.count_nonzero(valid) == 0:
            return np.array([], dtype=np.int32), np.array([], dtype=np.int32), np.array([], dtype=np.float64)

        ranges_v = ranges[valid]
        angles_v = angles[valid]

        x = ranges_v * np.cos(angles_v)
        y = ranges_v * np.sin(angles_v)
        z = np.zeros_like(x)
        ones = np.ones_like(x)

        pts_lidar = np.vstack([x, y, z, ones])  # 4xN
        pts_cam = self.extrinsic_mat @ pts_lidar

        front = pts_cam[2, :] > self.min_cam_z
        pts_cam = pts_cam[:, front]
        ranges_v = ranges_v[front]

        if pts_cam.shape[1] == 0:
            return np.array([], dtype=np.int32), np.array([], dtype=np.int32), np.array([], dtype=np.float64)

        pix = self.K @ pts_cam[:3, :]
        u = pix[0, :] / pix[2, :]
        v = pix[1, :] / pix[2, :]

        u_i = u.astype(np.int32)
        v_i = v.astype(np.int32)

        inside = (u_i >= 0) & (u_i < img_w) & (v_i >= 0) & (v_i < img_h)
        return u_i[inside], v_i[inside], ranges_v[inside]

    def estimate_distance_in_bbox(self, u, v, ranges, x1, y1, x2, y2) -> Tuple[Optional[float], Optional[Tuple[int, int]]]:
        mask = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        if np.count_nonzero(mask) == 0:
            return None, None

        r = ranges[mask]
        uu = u[mask]
        vv = v[mask]

        filtered = self._filter_front_points(r, uu, vv, x1, y1, x2, y2)
        if filtered is None:
            return None, None

        candidate_r, candidate_u, candidate_v = filtered
        if len(candidate_r) == 0:
            return None, None

        if self.distance_method == 'min':
            idx = int(np.argmin(candidate_r))
            dist = float(candidate_r[idx])
            best_uv = (int(candidate_u[idx]), int(candidate_v[idx]))
            return dist, best_uv

        if self.distance_method == 'median':
            dist = float(np.median(candidate_r))
            idx = int(np.argmin(np.abs(candidate_r - dist)))
            best_uv = (int(candidate_u[idx]), int(candidate_v[idx]))
            return dist, best_uv

        if self.distance_method == 'p20':
            dist = float(np.percentile(candidate_r, 20))
            idx = int(np.argmin(np.abs(candidate_r - dist)))
            best_uv = (int(candidate_u[idx]), int(candidate_v[idx]))
            return dist, best_uv

        # center: bbox 중심에 가깝고, 앞쪽(더 짧은 거리) 포인트들을 우선적으로 사용
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        center_deltas = np.sqrt((candidate_u - cx) ** 2 + (candidate_v - cy) ** 2)
        center_order = np.argsort(center_deltas)
        sorted_r = candidate_r[center_order]
        sorted_u = candidate_u[center_order]
        sorted_v = candidate_v[center_order]

        # 오차 허용 범위 안에 있는 포인트들을 묶어서 평균 사용
        if len(sorted_r) >= 1:
            base_dist = float(sorted_r[0])
            tolerance = max(self.distance_tolerance, 1e-3)
            close_mask = np.abs(sorted_r - base_dist) <= tolerance
            if np.count_nonzero(close_mask) == 0:
                close_mask = np.ones_like(sorted_r, dtype=bool)

            selected_r = sorted_r[close_mask]
            selected_u = sorted_u[close_mask]
            selected_v = sorted_v[close_mask]

            if len(selected_r) >= 1:
                dist = float(np.mean(selected_r))
                best_uv = (int(np.mean(selected_u)), int(np.mean(selected_v)))
                return dist, best_uv

        idx = int(np.argmin(sorted_r))
        dist = float(sorted_r[idx])
        best_uv = (int(sorted_u[idx]), int(sorted_v[idx]))
        return dist, best_uv

    def _filter_front_points(self, r, u, v, x1, y1, x2, y2):
        if len(r) == 0:
            return None

        valid_mask = np.isfinite(r) & (r >= self.min_range) & (r <= self.max_range)
        if np.count_nonzero(valid_mask) == 0:
            return None

        r_valid = r[valid_mask]
        u_valid = u[valid_mask]
        v_valid = v[valid_mask]

        if len(r_valid) == 0:
            return None

        # 박스 중심 기준으로 앞쪽에 있는 포인트만 남김
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        center_deltas = np.sqrt((u_valid - cx) ** 2 + (v_valid - cy) ** 2)
        center_order = np.argsort(center_deltas)

        ordered_r = r_valid[center_order]
        ordered_u = u_valid[center_order]
        ordered_v = v_valid[center_order]

        # 가까운 포인트를 우선적으로 사용하되, 너무 멀리 있는 배경 포인트는 제외
        base_dist = float(np.min(ordered_r))
        tolerance = max(self.distance_tolerance, 1e-3)
        close_mask = np.abs(ordered_r - base_dist) <= tolerance
        if np.count_nonzero(close_mask) == 0:
            close_mask = np.ones_like(ordered_r, dtype=bool)

        return ordered_r[close_mask], ordered_u[close_mask], ordered_v[close_mask]


def main(args=None):
    rclpy.init(args=args)
    node = FusionVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.display:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()