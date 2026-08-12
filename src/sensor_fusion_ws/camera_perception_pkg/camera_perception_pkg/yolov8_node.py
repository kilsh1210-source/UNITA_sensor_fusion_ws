# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from interfaces_pkg.msg import Point2D
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import Mask
from interfaces_pkg.msg import KeyPoint2D
from interfaces_pkg.msg import KeyPoint2DArray
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray

from std_srvs.srv import SetBool


class Yolov8Node(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)
        
        #---------------Variable Setting---------------
        # 딥러닝 모델 pt 파일명 작성 (launch에서 절대경로로 덮어쓰는 것을 권장)
        # 콤마(,)로 여러 개 지정하면 각 모델을 모두 돌려서 결과를 하나로 합쳐 발행함
        # (예: "best_cone.pt,car_back.pt")
        #self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("model", "best.pt")
        
        # 추론 하드웨어 선택 (cpu / gpu)
        self.declare_parameter("device", "cpu")
        #self.declare_parameter("device", "cuda:0")
        #----------------------------------------------
        
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        # ---------------- 오탐 억제 필터 ----------------
        # 멀리 있는 배경에서 아주 작은 박스로 튀는 오탐을 거르기 위한 값들.
        # 콘/드럼은 화면에서 어느 정도 크기 이상으로 보여야 실제로 주행에 의미가 있고,
        # 그보다 작은 검출은 대부분 배경 노이즈다.
        self.declare_parameter("min_box_area_px", 0)      # bbox 최소 넓이 [px^2], 0이면 끔
        self.declare_parameter("min_box_height_px", 0)    # bbox 최소 높이 [px], 0이면 끔
        # bbox 아래변이 이미지 상단 이 비율보다 위에 있으면 버림 (지평선 위 = 하늘/먼 배경).
        # 0.0이면 끔. 예: 0.35면 화면 위쪽 35% 안에서 끝나는 박스는 무시.
        self.declare_parameter("min_box_bottom_ratio", 0.0)
        # 클래스별 임계값 override. "cone:0.6,drum:0.55" 형식. 비우면 threshold 공통 적용
        self.declare_parameter("class_thresholds", "")
        self.declare_parameter("max_det", 300)
        # 필터로 버린 검출 개수를 주기적으로 로그로 남길지 여부 (튜닝용)
        self.declare_parameter("log_filtered", True)

        self.get_logger().info('Yolov8Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.iou = self.get_parameter(
            "iou").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.min_box_area_px = self.get_parameter(
            "min_box_area_px").get_parameter_value().integer_value
        self.min_box_height_px = self.get_parameter(
            "min_box_height_px").get_parameter_value().integer_value
        self.min_box_bottom_ratio = self.get_parameter(
            "min_box_bottom_ratio").get_parameter_value().double_value
        self.max_det = self.get_parameter(
            "max_det").get_parameter_value().integer_value
        self.log_filtered = self.get_parameter(
            "log_filtered").get_parameter_value().bool_value

        self.class_thresholds = self._parse_class_thresholds(
            self.get_parameter("class_thresholds").get_parameter_value().string_value)
        if self.class_thresholds:
            self.get_logger().info(f"클래스별 임계값: {self.class_thresholds}")

        self._filtered_count = 0
        self._kept_count = 0

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "detections", 10)
        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def _parse_class_thresholds(self, raw: str) -> Dict[str, float]:
        """'cone:0.6,drum:0.55' 형태를 dict로 변환."""
        table: Dict[str, float] = {}
        for chunk in raw.split(','):
            chunk = chunk.strip()
            if not chunk:
                continue
            if ':' not in chunk:
                self.get_logger().warn(
                    f"class_thresholds 형식 오류 (무시): '{chunk}' - 'name:0.6' 형식이어야 함")
                continue
            name, _, value = chunk.partition(':')
            try:
                table[name.strip()] = float(value)
            except ValueError:
                self.get_logger().warn(f"class_thresholds 값이 숫자가 아님 (무시): '{chunk}'")
        return table

    def _keep_detection(self, class_name: str, score: float,
                        bbox: BoundingBox2D, img_h: int) -> bool:
        """오탐으로 보이는 검출을 걸러낸다.

        멀리 있는 배경에서 튀는 오탐은 대체로 (1) 박스가 아주 작고 (2) 화면 위쪽
        (지평선 근처)에 뜬다는 공통점이 있어서, 크기와 위치로 상당수를 걸러낼 수 있다.
        """
        min_score = self.class_thresholds.get(class_name)
        if min_score is not None and score < min_score:
            return False

        w = bbox.size.x
        h = bbox.size.y

        if self.min_box_area_px > 0 and (w * h) < self.min_box_area_px:
            return False

        if self.min_box_height_px > 0 and h < self.min_box_height_px:
            return False

        if self.min_box_bottom_ratio > 0.0:
            bottom = bbox.center.position.y + h / 2.0
            if bottom < img_h * self.min_box_bottom_ratio:
                return False

        return True

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        model_paths = [p.strip() for p in self.model.split(',') if p.strip()]

        self.yolo_list: List[YOLO] = []
        for path in model_paths:
            try:
                yolo = YOLO(path)  # 모델 로딩
                yolo.fuse()
                self.yolo_list.append(yolo)
            except FileNotFoundError:
                self.get_logger().error(f"Error: Model file '{path}' not found!")
                return TransitionCallbackReturn.FAILURE
            except Exception as e:
                self.get_logger().error(f"Error while loading model '{path}': {str(e)}")
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f"Loaded {len(self.yolo_list)} model(s): {model_paths}")

        # subs
        self._sub = self.create_subscription(
            Image,
            "image_raw",
            self.image_cb,
            self.image_qos_profile
        )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS


    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        del self.yolo_list
        if 'cuda' in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

    def parse_hypothesis(self, results: Results, yolo: YOLO) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: Image) -> None:
        # 예전에는 여기서 매 프레임 print(msg.header)를 찍었다. launch가 output='screen'이라
        # 30 Hz로 stdout에 쏟아지면서 콜백이 느려지고, QoS depth=1이라 그만큼 프레임이 버려졌다.

        if not self.enable:
            return

        # convert image once, run every loaded model against it
        # 인코딩을 명시하지 않으면 퍼블리셔가 rgb8로 바뀌었을 때 채널이 뒤집힌 채로
        # 추론이 돌아간다(주황색 콘이 파랗게 보임). bgr8로 못박아둔다.
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_h = cv_image.shape[0]

        detections_msg = DetectionArray()
        detections_msg.header = msg.header

        dropped = 0

        for yolo in self.yolo_list:

            results = yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                iou=self.iou,
                max_det=self.max_det,
                device=self.device
            )
            results: Results = results[0].cpu()

            hypothesis = []
            boxes = []
            masks = []
            keypoints = []

            if results.boxes:
                hypothesis = self.parse_hypothesis(results, yolo)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    if not self._keep_detection(
                            hypothesis[i]["class_name"], hypothesis[i]["score"],
                            boxes[i], img_h):
                        dropped += 1
                        continue

                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            del results

        # publish merged detections from all models
        self._pub.publish(detections_msg)

        if self.log_filtered:
            self._filtered_count += dropped
            self._kept_count += len(detections_msg.detections)
            if self._filtered_count and (self._filtered_count + self._kept_count) >= 300:
                self.get_logger().info(
                    f"최근 300건 중 필터로 버린 검출 {self._filtered_count}개 "
                    f"(발행 {self._kept_count}개) - 너무 많이 버려지면 "
                    f"min_box_area_px/min_box_height_px/min_box_bottom_ratio를 낮출 것")
                self._filtered_count = 0
                self._kept_count = 0

        del cv_image


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
