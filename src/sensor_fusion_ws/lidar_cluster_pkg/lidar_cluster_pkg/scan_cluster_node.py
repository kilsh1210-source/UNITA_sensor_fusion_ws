#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from lidar_cluster_pkg.clustering_utils import (
    Cluster,
    centroid,
    euclidean_sequential_clustering,
    hsv_to_rgba,
    representative_point,
    scan_to_points,
)


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
        points = scan_to_points(msg, self.max_range)

        # 2) points -> clusters
        clusters = euclidean_sequential_clustering(
            points, self.cluster_tolerance, self.min_cluster_size, self.max_cluster_size
        )

        # 3) clusters -> MarkerArray publish
        marker_array = self.make_markers(clusters, msg.header.stamp)
        self.pub.publish(marker_array)

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
            cx, cy = centroid(c.points)
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

            # 3) text label
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
