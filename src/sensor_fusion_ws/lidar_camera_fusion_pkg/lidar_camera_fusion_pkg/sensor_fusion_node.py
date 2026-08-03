#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from cv_bridge import CvBridge

from interfaces_pkg.msg import DetectionArray


class SensorFusionNode(Node):

    def __init__(self):

        super().__init__("sensor_fusion_node")

        self.bridge = CvBridge()

        ###############################
        # Camera Parameter
        ###############################

        self.image_width = 640
        self.image_height = 480

        # 카메라 내부파라미터 (실측 캘리브레이션 값). 선형 FOV 근사 대신 정확한 핀홀 투영
        # angle = atan((pixel_x - cx) / fx) 으로 픽셀→각도 변환을 하기 위해 필요.
        # 카메라를 바꾸면 재캘리브레이션 후 launch 인자 fx/cx로 값을 갱신할 것
        self.declare_parameter("fx", 559.431712)
        self.declare_parameter("cx", 302.888725)

        # 라이다가 인식하는 정면 방향이 실제 원하는 정면과 반대일 때 보정값(도).
        # 라이다 장착 방향에 맞게 launch 인자 lidar_front_offset_deg로 
        # 조정
        # (예: 정반대면 180, 좌우만 바뀌면 부호 반전 필요)
        self.declare_parameter("lidar_front_offset_deg", 180.0)

        self.fx = self.get_parameter("fx").value
        self.cx = self.get_parameter("cx").value
        self.lidar_front_offset_deg = self.get_parameter("lidar_front_offset_deg").value

        self.get_logger().info(
            f"fx={self.fx}, cx={self.cx}, "
            f"lidar_front_offset_deg={self.lidar_front_offset_deg} deg"
        )

        ###############################
        # ROS Data
        ###############################

        self.latest_image = None
        self.latest_scan = None
        self.latest_detection = None

        self.click_point = None

        cv2.namedWindow("Sensor Fusion")
        cv2.setMouseCallback("Sensor Fusion", self.mouse_callback)

        ###############################
        # Subscriber
        ###############################

        self.image_sub = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.det_sub = self.create_subscription(
            DetectionArray,
            "/detections",
            self.detection_callback,
            10
        )

        ###############################

        self.timer = self.create_timer(
            0.03,
            self.timer_callback
        )

        self.get_logger().info("Sensor Fusion Node Started")

    #####################################################

    def image_callback(self, msg):

        self.latest_image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="passthrough"
        )

    #####################################################

    def scan_callback(self, msg):

        self.latest_scan = msg

        self.print_lidar_distances(msg)

    #####################################################

    def print_lidar_distances(self, msg):

        ranges = list(msg.ranges)

        total = len(ranges)

        if total == 0:
            return

        # 정면 (방향 보정 offset 적용)
        offset_steps = int(math.radians(self.lidar_front_offset_deg) / msg.angle_increment)
        front_idx = (total // 2 + offset_steps) % total

        # 좌우 30도
        left_idx = (front_idx + int(math.radians(30) / msg.angle_increment)) % total
        right_idx = (front_idx - int(math.radians(30) / msg.angle_increment)) % total

        front = ranges[front_idx]
        left = ranges[left_idx]
        right = ranges[right_idx]

        if math.isinf(front):
            front = -1

        if math.isinf(left):
            left = -1

        if math.isinf(right):
            right = -1

        print(
            f"Front : {front:.2f} m   "
            f"Left : {left:.2f} m   "
            f"Right : {right:.2f} m"
        )

    #####################################################

    def mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)

    #####################################################

    def detection_callback(self, msg):

        self.latest_detection = msg

    #####################################################

    def pixel_to_angle(self, pixel_x):

        # 핀홀 카메라 모델: angle = atan((pixel_x - cx) / fx)
        return math.degrees(math.atan2(pixel_x - self.cx, self.fx))

    #####################################################

    def get_distance(self, angle_deg):

        if self.latest_scan is None:
            return None

        scan = self.latest_scan

        total = len(scan.ranges)

        angle_span = scan.angle_max - scan.angle_min

        angle_rad = math.radians(angle_deg + self.lidar_front_offset_deg)

        # 라이다가 지원하는 각도 범위로 정규화(wrap)
        angle_rad = (
            (angle_rad - scan.angle_min) % angle_span
        ) + scan.angle_min

        index = int(
            (angle_rad - scan.angle_min)
            / scan.angle_increment
        )

        if index < 0:
            return None

        if index >= total:
            return None

        distance = scan.ranges[index]

        if math.isinf(distance):
            return None

        if math.isnan(distance):
            return None

        return distance

    #####################################################

    def scan_points_in_bbox(self, x1, x2, step=4):

        # bbox의 x1~x2 픽셀 범위를 훑으면서, 유효한(inf/nan 아닌) 라이다 리턴이
        # 있는 (픽셀 x, 거리) 목록을 반환. 박스 중심 레이 1개만 보는 것보다
        # 박스 폭 전체를 봐야 사람 일부가 순간적으로 빠지는 걸 방지할 수 있다.

        points = []

        x1c = max(0, int(x1))
        x2c = min(self.image_width - 1, int(x2))

        for px in range(x1c, x2c + 1, max(1, step)):

            angle = self.pixel_to_angle(px)

            distance = self.get_distance(angle)

            if distance is not None:
                points.append((px, distance))

        return points

    #####################################################

    def draw_detection(self, image, det):

        bbox = det.bbox

        cx = bbox.center.position.x
        cy = bbox.center.position.y

        w = bbox.size.x
        h = bbox.size.y

        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)

        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)

        points = self.scan_points_in_bbox(x1, x2)

        if points:
            distance = min(d for _, d in points)
        else:
            distance = None

        if distance is None:
            text = det.class_name
        else:
            text = f"{det.class_name} {distance:.2f} m"

        color = (0, 255, 0)

        cv2.rectangle(
            image,
            (x1, y1),
            (x2, y2),
            color,
            2
        )

        for px, _ in points:
            cv2.circle(
                image,
                (px, int(cy)),
                3,
                (0, 0, 255),
                -1
            )

        cv2.putText(
            image,
            text,
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

    #####################################################

    def draw_click_point(self, image, point):

        x, y = point

        angle = self.pixel_to_angle(x)

        distance = self.get_distance(angle)

        color = (0, 255, 255)

        cv2.drawMarker(
            image,
            (x, y),
            color,
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2
        )

        if distance is None:
            text = "N/A"
        else:
            text = f"{distance:.2f} m"

        cv2.putText(
            image,
            text,
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

    #####################################################

    def timer_callback(self):

        if self.latest_image is None:
            return

        image = self.latest_image.copy()

        if self.latest_detection is not None:

            for det in self.latest_detection.detections:

                self.draw_detection(image, det)

        if self.click_point is not None:

            self.draw_click_point(image, self.click_point)

        cv2.imshow(
            "Sensor Fusion",
            image
        )

        cv2.waitKey(1)

        #####################################################


def main(args=None):

    rclpy.init(args=args)

    node = SensorFusionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()