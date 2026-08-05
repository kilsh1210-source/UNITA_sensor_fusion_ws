#!/usr/bin/env python3
"""
클러스터별 L-shape(사각형) fitting 노드.

각 라이다 클러스터의 (x,y) 포인트에 대해 각도를 회전시켜가며 두 축에 투영했을 때
포인트들이 사각형의 두 변에 가장 가깝게 붙는 각도를 탐색하는 방식(search-based
rectangle fitting, closeness criterion)으로 방향(heading)과 폭/길이를 추정한다.
"""
import math
import os
import subprocess
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from lidar_cluster_pkg.clustering_utils import (
    Cluster,
    euclidean_sequential_clustering,
    filter_front_fov,
    hsv_to_rgba,
    scan_to_points,
)


class RectFit:
    __slots__ = ("cx", "cy", "yaw", "long_extent", "short_extent")

    def __init__(self, cx: float, cy: float, yaw: float, long_extent: float, short_extent: float):
        self.cx = cx
        self.cy = cy
        self.yaw = yaw
        self.long_extent = long_extent
        self.short_extent = short_extent


def fit_l_shape(points: List[Tuple[float, float]], theta_resolution_deg: float) -> Optional[RectFit]:
    """
    0~90도 범위를 theta_resolution_deg 간격으로 회전시키며, 각 각도에서 두 후보 변에
    포인트들이 얼마나 가깝게 붙는지(closeness criterion)를 비용으로 계산해 최소가 되는
    각도를 찾는다. 사각형은 90도 대칭이므로 0~90도만 탐색하면 충분하다.
    """
    if len(points) < 3:
        return None

    best_theta = 0.0
    best_cost = float("inf")
    best_bounds = None  # (c1_min, c1_max, c2_min, c2_max)

    n_steps = max(1, int(round(90.0 / theta_resolution_deg)))
    for i in range(n_steps):
        theta = math.radians(i * theta_resolution_deg)
        c = math.cos(theta)
        s = math.sin(theta)

        c1_vals = [x * c + y * s for (x, y) in points]
        c2_vals = [-x * s + y * c for (x, y) in points]
        c1_min, c1_max = min(c1_vals), max(c1_vals)
        c2_min, c2_max = min(c2_vals), max(c2_vals)

        cost = 0.0
        for c1v, c2v in zip(c1_vals, c2_vals):
            d1 = min(c1_max - c1v, c1v - c1_min)
            d2 = min(c2_max - c2v, c2v - c2_min)
            cost += min(d1, d2)

        if cost < best_cost:
            best_cost = cost
            best_theta = theta
            best_bounds = (c1_min, c1_max, c2_min, c2_max)

    c1_min, c1_max, c2_min, c2_max = best_bounds
    c = math.cos(best_theta)
    s = math.sin(best_theta)

    width = c1_max - c1_min   # e1(=best_theta 방향) 축 폭
    length = c2_max - c2_min  # e2(=best_theta+90도 방향) 축 폭
    center_c1 = (c1_min + c1_max) / 2.0
    center_c2 = (c2_min + c2_max) / 2.0
    cx = center_c1 * c - center_c2 * s
    cy = center_c1 * s + center_c2 * c

    # 더 긴 변을 heading(로컬 x축)으로 삼는다.
    if width >= length:
        yaw = best_theta
        long_extent, short_extent = width, length
    else:
        yaw = best_theta + math.pi / 2.0
        long_extent, short_extent = length, width

    return RectFit(cx=cx, cy=cy, yaw=yaw, long_extent=long_extent, short_extent=short_extent)


def rect_corners(fit: RectFit) -> List[Tuple[float, float]]:
    hl = fit.long_extent / 2.0
    hs = fit.short_extent / 2.0
    c = math.cos(fit.yaw)
    s = math.sin(fit.yaw)
    local = [(-hl, -hs), (hl, -hs), (hl, hs), (-hl, hs)]
    return [(fit.cx + lx * c - ly * s, fit.cy + lx * s + ly * c) for (lx, ly) in local]


class LShapeNode(Node):
    def __init__(self):
        super().__init__('l_shape_node')

        # ---------- Parameters ----------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/l_shape_boxes')
        self.declare_parameter('frame_id', 'laser')

        # 클러스터링 파라미터 (scan_cluster_node와 동일한 기본값)
        self.declare_parameter('cluster_tolerance', 0.12)
        self.declare_parameter('min_cluster_size', 6)
        self.declare_parameter('max_cluster_size', 400)
        self.declare_parameter('max_range', 6.0)

        # 전방 FOV 필터 (라이다가 물리적으로 180도 돌려 장착된 경우 등을 보정)
        self.declare_parameter('front_angle_deg', 180.0)
        self.declare_parameter('fov_deg', 150.0)

        # L-shape fitting 파라미터
        self.declare_parameter('theta_resolution_deg', 1.0)
        self.declare_parameter('min_points_for_fit', 4)

        # 시각화 파라미터
        self.declare_parameter('rect_line_width', 0.02)
        self.declare_parameter('show_heading_arrow', True)
        self.declare_parameter('arrow_length', 0.3)
        self.declare_parameter('show_text', True)
        self.declare_parameter('text_size', 0.15)

        # RViz2 자동 실행
        self.declare_parameter('launch_rviz', True)
        self.declare_parameter('rviz_config', self._default_rviz_config_path())

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.cluster_tolerance = float(self.get_parameter('cluster_tolerance').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.max_cluster_size = int(self.get_parameter('max_cluster_size').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)

        self.theta_resolution_deg = float(self.get_parameter('theta_resolution_deg').value)
        self.min_points_for_fit = int(self.get_parameter('min_points_for_fit').value)

        self.rect_line_width = float(self.get_parameter('rect_line_width').value)
        self.show_heading_arrow = bool(self.get_parameter('show_heading_arrow').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)
        self.show_text = bool(self.get_parameter('show_text').value)
        self.text_size = float(self.get_parameter('text_size').value)

        self.launch_rviz = bool(self.get_parameter('launch_rviz').value)
        self.rviz_config = self.get_parameter('rviz_config').get_parameter_value().string_value

        # ---------- ROS interfaces ----------
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.rviz_process = None
        if self.launch_rviz:
            self._start_rviz()

        self.get_logger().info(
            f"LShapeNode started. subscribe={self.scan_topic}, publish={self.marker_topic}, frame_id={self.frame_id}"
        )

    @staticmethod
    def _default_rviz_config_path() -> str:
        try:
            return os.path.join(
                get_package_share_directory('lidar_cluster_pkg'), 'rviz', 'L_shape_config.rviz'
            )
        except Exception:
            return ''

    def _start_rviz(self):
        if not self.rviz_config or not os.path.isfile(self.rviz_config):
            self.get_logger().warn(
                f"RViz config를 찾을 수 없어 rviz2 자동 실행을 건너뜁니다: '{self.rviz_config}'"
            )
            return
        try:
            self.rviz_process = subprocess.Popen(['rviz2', '-d', self.rviz_config])
            self.get_logger().info(f"rviz2 실행: {self.rviz_config}")
        except FileNotFoundError:
            self.get_logger().warn("rviz2 실행파일을 찾을 수 없습니다. RViz2 설치 여부를 확인하세요.")

    def on_scan(self, msg: LaserScan):
        points = scan_to_points(msg, self.max_range)
        points = filter_front_fov(points, self.front_angle_deg, self.fov_deg)
        clusters = euclidean_sequential_clustering(
            points, self.cluster_tolerance, self.min_cluster_size, self.max_cluster_size
        )
        marker_array = self.make_markers(clusters, msg.header.stamp)
        self.pub.publish(marker_array)

    def make_markers(self, clusters: List[Cluster], stamp) -> MarkerArray:
        ma = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        for idx, cluster in enumerate(clusters):
            if len(cluster.points) < self.min_points_for_fit:
                continue

            fit = fit_l_shape(cluster.points, self.theta_resolution_deg)
            if fit is None:
                continue

            color = hsv_to_rgba(idx / max(1, len(clusters)))
            corners = rect_corners(fit)

            # 1) 사각형 외곽선
            m_rect = Marker()
            m_rect.header.frame_id = self.frame_id
            m_rect.header.stamp = stamp
            m_rect.ns = "l_shape_rect"
            m_rect.id = idx
            m_rect.type = Marker.LINE_STRIP
            m_rect.action = Marker.ADD
            m_rect.scale.x = self.rect_line_width
            m_rect.color = color
            for (x, y) in corners + [corners[0]]:
                m_rect.points.append(Point(x=float(x), y=float(y), z=0.0))
            ma.markers.append(m_rect)

            # 2) heading 화살표 (긴 변 방향)
            if self.show_heading_arrow:
                m_arrow = Marker()
                m_arrow.header.frame_id = self.frame_id
                m_arrow.header.stamp = stamp
                m_arrow.ns = "l_shape_heading"
                m_arrow.id = idx
                m_arrow.type = Marker.ARROW
                m_arrow.action = Marker.ADD
                m_arrow.pose.position.x = float(fit.cx)
                m_arrow.pose.position.y = float(fit.cy)
                m_arrow.pose.position.z = 0.0
                m_arrow.pose.orientation.z = math.sin(fit.yaw / 2.0)
                m_arrow.pose.orientation.w = math.cos(fit.yaw / 2.0)
                m_arrow.scale.x = self.arrow_length
                m_arrow.scale.y = 0.03
                m_arrow.scale.z = 0.03
                m_arrow.color = color
                ma.markers.append(m_arrow)

            # 3) 크기 텍스트
            if self.show_text:
                m_text = Marker()
                m_text.header.frame_id = self.frame_id
                m_text.header.stamp = stamp
                m_text.ns = "l_shape_text"
                m_text.id = idx
                m_text.type = Marker.TEXT_VIEW_FACING
                m_text.action = Marker.ADD
                m_text.pose.position.x = float(fit.cx)
                m_text.pose.position.y = float(fit.cy)
                m_text.pose.position.z = 0.25
                m_text.pose.orientation.w = 1.0
                m_text.scale.z = self.text_size
                m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                m_text.text = (
                    f"#{idx} {fit.long_extent:.2f}x{fit.short_extent:.2f}m "
                    f"yaw={math.degrees(fit.yaw):.0f}"
                )
                ma.markers.append(m_text)

        return ma


def main(args=None):
    rclpy.init(args=args)
    node = LShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.rviz_process is not None and node.rviz_process.poll() is None:
            node.rviz_process.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
