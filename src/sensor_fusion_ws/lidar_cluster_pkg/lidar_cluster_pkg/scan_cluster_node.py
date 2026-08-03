#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


@dataclass
class Cluster:
    points: List[Tuple[float, float]]  # (x, y)


def hsv_to_rgba(h: float, s: float = 0.9, v: float = 0.9, a: float = 1.0) -> ColorRGBA:
    """Simple HSV->RGBA (0<=h<1)."""
    h = h % 1.0
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i = i % 6
    if i == 0:
        r, g, b = v, t, p
    elif i == 1:
        r, g, b = q, v, p
    elif i == 2:
        r, g, b = p, v, t
    elif i == 3:
        r, g, b = p, q, v
    elif i == 4:
        r, g, b = t, p, v
    else:
        r, g, b = v, p, q
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def representative_point(points: List[Tuple[float, float]], cx: float, cy: float) -> Tuple[float, float]:
    """
    centroid (cx,cy)에 가장 가까운 '실제 포인트'를 대표점으로 선택.
    centroid가 공간에 뜨는 문제를 방지하기 위한 간단/강력한 방식.
    """
    return min(points, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)


class ScanClusterNode(Node):
    def __init__(self):
        super().__init__('scan_cluster_node')

        # ---------- Parameters ----------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', '/lidar_clusters')
        self.declare_parameter('frame_id', 'laser')

        # 클러스터링 파라미터(환경에 맞게 조정)
        self.declare_parameter('cluster_tolerance', 0.12)   # [m] 인접 포인트 연결 기준
        self.declare_parameter('min_cluster_size', 6)       # 최소 포인트 개수
        self.declare_parameter('max_cluster_size', 400)     # 최대 포인트 개수(폭주 방지)
        self.declare_parameter('max_range', 6.0)            # [m] 너무 먼 점 제외(미션 상황에 맞게)

        # 시각화 파라미터
        self.declare_parameter('point_size', 0.05)          # RViz points size
        self.declare_parameter('centroid_size', 0.10)       # centroid sphere size
        self.declare_parameter('text_size', 0.18)           # text height

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.cluster_tolerance = float(self.get_parameter('cluster_tolerance').value)
        self.min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self.max_cluster_size = int(self.get_parameter('max_cluster_size').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.point_size = float(self.get_parameter('point_size').value)
        self.centroid_size = float(self.get_parameter('centroid_size').value)
        self.text_size = float(self.get_parameter('text_size').value)

        # ---------- ROS interfaces ----------
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info(
            f"ScanClusterNode started. subscribe={self.scan_topic}, publish={self.marker_topic}, frame_id={self.frame_id}"
        )

    def on_scan(self, msg: LaserScan):
        # 1) LaserScan -> (x,y) points
        points = self.scan_to_points(msg)

        # 2) points -> clusters
        clusters = self.euclidean_sequential_clustering(points)

        # 3) clusters -> MarkerArray publish
        marker_array = self.make_markers(clusters, msg.header.stamp)
        self.pub.publish(marker_array)

    def scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        angle = msg.angle_min

        # range gating
        rmin = max(msg.range_min, 0.0)
        rmax = min(msg.range_max, self.max_range)

        for r in msg.ranges:
            if math.isfinite(r) and (rmin <= r <= rmax):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                pts.append((x, y))
            angle += msg.angle_increment

        return pts

    def euclidean_sequential_clustering(self, pts: List[Tuple[float, float]]) -> List[Cluster]:
        """
        2D LiDAR scan order 기반 간단 클러스터링.
        연속 포인트 간 거리 <= cluster_tolerance이면 같은 클러스터로 묶음.
        """
        if not pts:
            return []

        clusters: List[Cluster] = []
        current: List[Tuple[float, float]] = [pts[0]]

        def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
            dx = a[0] - b[0]
            dy = a[1] - b[1]
            return math.hypot(dx, dy)

        for i in range(1, len(pts)):
            if dist(pts[i - 1], pts[i]) <= self.cluster_tolerance:
                current.append(pts[i])
                if len(current) > self.max_cluster_size:
                    # 폭주 방지: 너무 커지면 강제 분리(현재 덩어리도 일단 클러스터로 넣음)
                    clusters.append(Cluster(points=current))
                    current = []
            else:
                if len(current) >= self.min_cluster_size:
                    clusters.append(Cluster(points=current))
                current = [pts[i]]

        # 마지막 클러스터 처리
        if len(current) >= self.min_cluster_size:
            clusters.append(Cluster(points=current))

        return clusters

    def make_markers(self, clusters: List[Cluster], stamp) -> MarkerArray:
        ma = MarkerArray()

        # 이전 마커 제거를 위해 DELETEALL을 먼저 쏴주는 방식(간단/확실)
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        for idx, c in enumerate(clusters):
            if not c.points:
                continue

            color = hsv_to_rgba(idx / max(1, len(clusters)))

            # 1) points marker
            m_points = Marker()
            m_points.header.frame_id = self.frame_id
            m_points.header.stamp = stamp
            m_points.ns = "clusters_points"
            m_points.id = idx
            m_points.type = Marker.POINTS
            m_points.action = Marker.ADD
            m_points.scale.x = self.point_size
            m_points.scale.y = self.point_size
            m_points.color = color

            for (x, y) in c.points:
                m_points.points.append(Point(x=float(x), y=float(y), z=0.0))

            ma.markers.append(m_points)

            # 2) 대표점(centroid에 가장 가까운 실제 포인트)
            cx, cy = self.centroid(c.points)
            rx, ry = representative_point(c.points, cx, cy)

            m_cent = Marker()
            m_cent.header.frame_id = self.frame_id
            m_cent.header.stamp = stamp
            m_cent.ns = "clusters_centroid"   # namespace는 그대로 두되, 실제론 대표점임
            m_cent.id = idx
            m_cent.type = Marker.SPHERE
            m_cent.action = Marker.ADD
            m_cent.pose.position.x = float(rx)
            m_cent.pose.position.y = float(ry)
            m_cent.pose.position.z = 0.0
            m_cent.pose.orientation.w = 1.0
            m_cent.scale.x = self.centroid_size
            m_cent.scale.y = self.centroid_size
            m_cent.scale.z = self.centroid_size
            m_cent.color = color
            ma.markers.append(m_cent)

            # 3) text label (원하시면 주석 해제해서 사용)
            
            dist_m = math.hypot(rx, ry)
            m_text = Marker()
            m_text.header.frame_id = self.frame_id
            m_text.header.stamp = stamp
            m_text.ns = "clusters_text"
            m_text.id = idx
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = float(rx)
            m_text.pose.position.y = float(ry)
            m_text.pose.position.z = 0.25  # 살짝 띄워서 보이게
            m_text.pose.orientation.w = 1.0
            m_text.scale.z = self.text_size
            m_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m_text.text = f"#{idx} ({dist_m:.2f}m) n={len(c.points)}"
            ma.markers.append(m_text)
            
            

        return ma

    @staticmethod
    def centroid(pts: List[Tuple[float, float]]) -> Tuple[float, float]:
        sx = 0.0
        sy = 0.0
        n = float(len(pts))
        for (x, y) in pts:
            sx += x
            sy += y
        return (sx / n, sy / n)


def main(args=None):
    rclpy.init(args=args)
    node = ScanClusterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
