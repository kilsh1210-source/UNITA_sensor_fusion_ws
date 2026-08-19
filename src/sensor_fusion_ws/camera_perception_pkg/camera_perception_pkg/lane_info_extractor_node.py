"""차선 세그멘테이션(lane_1 / lane_2) 검출 결과 -> 주행용 차선 중심점(LaneInfo).

minicar_sim(gwakminji/minicar_sim)의 camera_perception_pkg/lane_info_extractor_node.py를
가져와서, 하드코딩돼 있던 튜닝값들을 ROS 파라미터로 뺀 버전이다.
(값은 sensor_fusion_bringup/config/params.yaml 에서 관리)

동작 순서
  1. /detections 에서 lane_1 / lane_2 마스크를 받아 "내가 지금 몇 차선인지" 판단
  2. /lidar_obstacle_info(Point32: x=거리[m], y=이미지상 중심 x[px], z=감지플래그)를 보고
     장애물이 내 차선 bbox 안에 있으면 옆 차선 쪽으로 목표 오프셋을 준다
  3. 추종할 차선 마스크의 edge를 BEV로 펴고, 높이별 차선 중심 x를 뽑아 LaneInfo로 발행
"""

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray
from .lib import camera_perception_func_lib as CPFL

#---------------Constant Variables---------------
SUB_TOPIC_NAME = "/detections"
SUB_OBSTACLE_TOPIC = "/lidar_obstacle_info"        # 가장 가까운 장애물 1개 (호환용)
SUB_OBSTACLE_ARRAY_TOPIC = "/lidar_obstacle_array"  # 검출된 장애물 전부
PUB_TOPIC_NAME = "/yolov8_lane_info"
ROI_IMAGE_TOPIC_NAME = "/roi_image"
SHOW_IMAGE = True
LANE_WIDTH_PIXEL = 200      # 차선 변경 시 옆 차선까지의 BEV 픽셀 거리 (실차ws에서는 280을 씀)
AVOIDANCE_TRIGGER_DIST = 2.4  # 이 거리[m]보다 가까운 장애물만 회피 대상 (실차ws에서는 1.8)
SHIFT_SPEED = 20.0          # 프레임당 오프셋 변화량(px). 클수록 차선 변경이 급해짐
IMAGE_CENTER_X = 320
LANE_1_FAR_LEFT_THRESHOLD = 180
LANE_2_FAR_RIGHT_THRESHOLD = 460
# 차선 상태(1차선/2차선) 전환 디바운싱: 새 상태가 이만큼 연속으로 잡혀야 실제로 전환한다.
# 1이면 minicar_sim 원본과 동일(즉시 전환), 실차에서는 마스크가 튀어서 15 정도가 안정적.
LANE_CHANGE_THRESHOLD_COUNT = 15

# 회피 복귀 가드: 반대쪽 차선이 최근 이 프레임 수 안에도 여전히 확인되고 있으면
# "같은 콘이 근접 구간에서 양쪽에 걸쳐 잡히는 상황"으로 보고 복귀를 보류한다.
OBSTACLE_RECENT_FRAMES_THRESHOLD = 5

# [실측 확인된 버그 - 단발 오판 방지] 장애물-차선 겹침 래치(lane1/2_obstacle_confirmed)를
# 걸기 전에 이만큼 연속 프레임 겹침을 요구한다. 근접 구간에서 차선 박스 폭이 순간적으로
# 넓게 잡히면(마스크 불안정) 반대쪽 차선 장애물까지 단 한 프레임 겹쳐서 래치가 걸리고,
# 그 결과 실제로는 오른쪽(lane_1)에 있는 콘을 왼쪽(lane_2)으로 오판해 콘 쪽으로 회피가
# 발동한 사고가 실측됐다. 2 정도면 노이즈성 단발은 걸러지면서 진짜 장애물 반응은 거의
# 안 늦는다.
OBSTACLE_CONFIRM_FRAMES = 2

# [와리가리 방지 - 쿨다운] 차선 전환(회피 진입/복귀)이 한 번 일어나면, 그 뒤 이만큼의
# 프레임 동안은 반대 방향 전환 판단 자체를 아예 하지 않는다. shift_speed=60px/frame,
# lane_width_pixel=280px 기준 이동 자체가 약 5프레임 걸리는데, 위 RECENT_FRAMES 가드가
# 5라 이동이 끝나자마자 가드가 무력화돼(같은 콘인데도 "최근에 안 봤음"으로 오판) 곧바로
# 되돌아가는 게 실측된 와리가리의 실제 원인이었다. 쿨다운은 "같은 콘인지 판단"에 기대지
# 않고 그냥 전환 직후 일정 시간은 무조건 재판단을 막아버리는 더 확실한 방어선이다.
# 카메라/추론 프레임레이트에 따라 체감 시간이 다르니, `ros2 topic hz /detections`로 실제
# fps를 확인하고 튜닝할 것 (기본값은 대략 10fps 기준 ~1.5초).
MIN_AVOIDANCE_DWELL_FRAMES = 15

# [실측 확인된 버그 - 차선 합류] 회피로 lane_1에 가 있는 상태에서 lane_1 선이 "일시적으로
# 안 보임"이 아니라 "실제로 끝나서(=lane_2와 합류)" 사라지면, 복귀 트리거가 장애물 재발견
# 기준이라 아무 장애물도 없는 합류 지점에서는 영원히 안 풀린다. 그 상태로 final_offset_modifier
# 로직이 lane_2의 진짜 중심선을 기준 삼아버려서 "lane_2 중심 + 280px"(도로 밖)를 계속
# 타겟으로 잡고, 그 결과 오른쪽으로 튀었다 직진했다를 반복하며 차선을 이탈했다(실측 확인).
# 그래서 lane_1이 이만큼 연속 프레임 동안 안 보이고 lane_2만 보이면, 장애물 유무와
# 무관하게 무조건 오프셋을 0으로 되돌린다("합류했으니 강제로 lane_2를 홈으로 인정").
# lane_change_threshold_count와 같은 값(15, 실차 노이즈 대비 검증된 값)을 기본값으로 쓴다.
LANE_MERGE_CONFIRM_FRAMES = 15

# [근접 구간 플리커링 방지] 추종 기준선(final_tracking_class) 전환 디바운싱. 콘이 가까워질수록
# has_lane_1/has_lane_2가 프레임마다 뒤집혀서, 디바운싱 없이 즉시 전환하면 current_offset이
# 매 프레임 0<->lane_width_pixel로 떨려 실제 조향까지 떨리고 회피가 사실상 무력화된다
# (실측 확인: 회피 오프셋은 잡혀 있는데 실제로는 안 꺾여서 그대로 부딪힘). lane_change_threshold_count
# 보다는 짧게 - 이건 반응성이 더 중요한 프레임 단위 판단이라 3이면 충분히 노이즈를 거른다.
TRACK_CLASS_SWITCH_THRESHOLD = 3

# BEV 변환용 원본 이미지 좌표 4점 (좌상, 우상, 우하, 좌하) - 640x480 기준
SRC_POINTS = [154.0, 298.0, 486.0, 298.0, 614.0, 470.0, 26.0, 470.0]
ROI_CUTTING_IDX = 300       # BEV 이미지에서 아래쪽으로 잘라낼 픽셀 (차 앞쪽만 남김)
TARGET_Y_START = 5          # 타겟 포인트를 뽑을 y 시작/끝/간격 (ROI 이미지 좌표)
TARGET_Y_END = 155
TARGET_Y_STEP = 30
LANE_WIDTH_FOR_CENTER = 300  # get_lane_center가 한쪽 선만 보일 때 가정하는 차선 폭(px)
# 아래 둘은 타겟 위치를 바꾸는 값이라 기본값이 전부 '예전 동작'이다.
# LANE_WIDTH_FOR_CENTER가 이 둘이 꺼진 상태에서 실측으로 역산된 값이기 때문.
# 자세한 내용은 camera_perception_func_lib.get_lane_center()
LANE_CENTER_TILT_COMP = 0.0        # 차선 기울기(cos) 보정 강도. 0.0=없음, 1.0=완전
LANE_CENTER_FORCE_SINGLE_LINE = False  # '두 선 보임' 분기 차단 여부

# [실측 확인된 버그 - 타겟 좌표 자체의 프레임간 노이즈] get_lane_center()는 매 프레임 아무
# 기억 없이 새로 계산한다. 근접 구간에서 콘 실루엣이 차선 픽셀을 갈라놓으면 "두 선 보임"으로
# 오판(camera_perception_func_lib.get_lane_center 233번 줄)해 타겟이 행(row)당 프레임마다
# 수백 px씩 튄다 - 위에서 고친 회피 오프셋/추종선 디바운싱과는 별개로, 이게 실측된 물리적
# 와리가리(회피 오프셋은 안정적으로 유지되는데도 차가 좌우로 흔들리다 부딪힘)의 진짜 원인이었다.
# 그래서 각 행(target_point_y로 식별)의 타겟 x좌표에 프레임당 최대 변화량을 건다 - 실제
# 곡선 추종(연속적인 변화)은 그대로 따라가고, 노이즈로 인한 순간 점프만 걸러낸다.
TARGET_X_RATE_LIMIT_PX = 50.0
#----------------------------------------------

class Yolov8InfoExtractor(Node):
    def __init__(self):
        super().__init__('lane_info_extractor_node')
        self.sub_topic = self.declare_parameter('sub_detection_topic', SUB_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.sub_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_OBSTACLE_TOPIC).value
        self.sub_obstacle_array_topic = self.declare_parameter(
            'sub_lidar_obstacle_array_topic', SUB_OBSTACLE_ARRAY_TOPIC).value
        self.roi_image_topic = self.declare_parameter('roi_image_topic', ROI_IMAGE_TOPIC_NAME).value
        self.show_image = bool(self.declare_parameter('show_image', SHOW_IMAGE).value)

        self.lane_width_pixel = float(self.declare_parameter('lane_width_pixel', float(LANE_WIDTH_PIXEL)).value)
        self.avoidance_trigger_dist = float(
            self.declare_parameter('avoidance_trigger_dist', AVOIDANCE_TRIGGER_DIST).value)
        self.shift_speed = float(self.declare_parameter('shift_speed', SHIFT_SPEED).value)
        self.image_center_x = int(self.declare_parameter('image_center_x', IMAGE_CENTER_X).value)
        self.lane_1_far_left_threshold = float(
            self.declare_parameter('lane_1_far_left_threshold', float(LANE_1_FAR_LEFT_THRESHOLD)).value)
        self.lane_2_far_right_threshold = float(
            self.declare_parameter('lane_2_far_right_threshold', float(LANE_2_FAR_RIGHT_THRESHOLD)).value)
        self.lane_change_threshold_count = int(
            self.declare_parameter('lane_change_threshold_count', LANE_CHANGE_THRESHOLD_COUNT).value)
        self.obstacle_recent_frames_threshold = int(
            self.declare_parameter('obstacle_recent_frames_threshold', OBSTACLE_RECENT_FRAMES_THRESHOLD).value)
        self.obstacle_confirm_frames = int(
            self.declare_parameter('obstacle_confirm_frames', OBSTACLE_CONFIRM_FRAMES).value)
        self.min_avoidance_dwell_frames = int(
            self.declare_parameter('min_avoidance_dwell_frames', MIN_AVOIDANCE_DWELL_FRAMES).value)
        self.lane_merge_confirm_frames = int(
            self.declare_parameter('lane_merge_confirm_frames', LANE_MERGE_CONFIRM_FRAMES).value)
        self.track_class_switch_threshold = int(
            self.declare_parameter('track_class_switch_threshold', TRACK_CLASS_SWITCH_THRESHOLD).value)

        src_flat = list(self.declare_parameter('src_points', SRC_POINTS).value)
        self.src_mat = [[int(round(src_flat[i])), int(round(src_flat[i + 1]))] for i in range(0, 8, 2)]
        self.roi_cutting_idx = int(self.declare_parameter('roi_cutting_idx', ROI_CUTTING_IDX).value)
        self.target_y_start = int(self.declare_parameter('target_y_start', TARGET_Y_START).value)
        self.target_y_end = int(self.declare_parameter('target_y_end', TARGET_Y_END).value)
        self.target_y_step = int(self.declare_parameter('target_y_step', TARGET_Y_STEP).value)
        self.lane_width_for_center = int(
            self.declare_parameter('lane_width_for_center', LANE_WIDTH_FOR_CENTER).value)
        # [lane_1 전용 폭] lane_width_for_center(216)는 lane_2만 91% 추종한 실측(car_center_x
        # 기준 target_x 평균 편차)으로 역산된 값이다. 카메라가 차체 중심에서 벗어나 있어
        # (car_center_x=332≠320) 좌우가 대칭이 아니므로, lane_1(우측선) 추종 시엔 이 값이
        # 안 맞을 수 있다(실측 확인: lane_1 회피 중 직진 불안정 -> 복귀 시 벽 충돌). 음수면
        # lane_width_for_center와 동일하게 취급(아직 lane_1 실측 보정 전 기본값).
        lane1_width_param = float(
            self.declare_parameter('lane_width_for_center_lane1', -1.0).value)
        self.lane_width_for_center_lane1 = (
            int(lane1_width_param) if lane1_width_param >= 0 else self.lane_width_for_center)
        self.lane_center_tilt_comp = float(
            self.declare_parameter('lane_center_tilt_comp', LANE_CENTER_TILT_COMP).value)
        self.lane_center_force_single_line = bool(
            self.declare_parameter('lane_center_force_single_line', LANE_CENTER_FORCE_SINGLE_LINE).value)
        # [lane_1 전용 - 실측 확인] lane_1은 실선이라 한 줄의 양쪽 가장자리를 서로 다른
        # 두 선으로 착각해 매 프레임 "두 선 보임"(중점 계산) <-> "한 선"(폭 오프셋) 분기를
        # 오락가락했다. 이 분기 전환 자체가 회피 후 lane_1 직진 중 "오른쪽/직진 반복"으로
        # 보이는 떨림의 원인이었다(실측: 직선 안정 구간에서도 매 프레임 TWO_LINE으로 잡힘).
        # lane_2는 이 혼재 상태에서 이미 캘리브레이션돼 있으니 안 건드리고, lane_1만 항상
        # 폭 오프셋 방식(단일선)으로 고정한다.
        self.lane_center_force_single_line_lane1 = bool(
            self.declare_parameter('lane_center_force_single_line_lane1', True).value)
        self.target_x_rate_limit = float(
            self.declare_parameter('target_x_rate_limit_px', TARGET_X_RATE_LIMIT_PX).value)

        self.cv_bridge = CvBridge()
        self.qos_profile = qos_profile_sensor_data
        self.subscriber = self.create_subscription(DetectionArray, self.sub_topic, self.yolov8_detections_callback, self.qos_profile)
        self.obstacle_sub = self.create_subscription(Point32, self.sub_obstacle_topic, self.obstacle_callback, self.qos_profile)
        self.obstacle_array_sub = self.create_subscription(
            Polygon, self.sub_obstacle_array_topic, self.obstacle_array_callback, self.qos_profile)
        self.publisher = self.create_publisher(LaneInfo, self.pub_topic, 10)
        self.roi_image_publisher = self.create_publisher(Image, self.roi_image_topic, 10)

        # 추종할 차선을 고정한다('lane_1' | 'lane_2'). 빈 문자열이면 기존 상태머신 사용.
        # 실차에서는 한쪽 선만 보이는 구간이 대부분이라 상태머신이 흔들리고,
        # 전환될 때마다 타겟이 lane_width_pixel만큼 튄다.
        self.fixed_lane_class = str(
            self.declare_parameter('fixed_lane_class', '').value).strip()
        if self.fixed_lane_class not in ('', 'lane_1', 'lane_2'):
            self.get_logger().warn(
                f"fixed_lane_class='{self.fixed_lane_class}' 는 올바르지 않음. 자동 판단으로 진행")
            self.fixed_lane_class = ''
        if self.fixed_lane_class:
            self.get_logger().info(f"차선 추종 고정: {self.fixed_lane_class}")

        self.current_lane_state = self.fixed_lane_class or 'lane_2'
        self.current_offset = 0.0
        self.target_offset = 0.0
        # 지금 실제로 달리고 있는 차선 오프셋(0.0=홈 차선 / ±lane_width_pixel=반대 차선).
        # 매 프레임 리셋되지 않고, 그 차선에서 새 장애물을 만나야만 반대쪽으로 바뀐다
        # (자동으로 홈 차선에 복귀하지 않음). 자세한 이유는 아래
        # yolov8_detections_callback()의 회피 판단부 주석 참고.
        self.active_avoidance_offset = 0.0
        # 정적 장애물 래치: 거리 제한 없이 차선과 겹침이 한 번이라도 확인되면 True로
        # 유지되고, 회피가 실제로 발동하는 순간 리셋된다. 자세한 이유는 아래
        # yolov8_detections_callback()의 회피 판단부 주석 참고.
        self.lane1_obstacle_confirmed = False
        self.lane2_obstacle_confirmed = False
        # 래치를 걸기 전 연속 겹침 프레임 카운터(단발 오판 방지용, 위 영구 래치와는 별개).
        self.lane1_confirm_run = 0
        self.lane2_confirm_run = 0
        # "최근에 봤는지" 카운터. 복귀 판단 시 반대쪽 차선이 최근에도 여전히 확인되고
        # 있으면(=같은 콘이 근접 구간에서 양쪽에 걸쳐 잡히는 상황) 복귀를 보류하는 데
        # 쓴다. 영구 래치(lane1/2_obstacle_confirmed)를 그대로 가드로 쓰면 한 번 True가
        # 된 뒤 리셋할 유일한 경로(복귀 트리거)를 그 가드 자신이 막아버려 영영 복귀를
        # 못 하게 잠길 수 있어서, 자연 감쇠하는 별도 카운터를 둔다.
        self.lane1_frames_since_seen = 999
        self.lane2_frames_since_seen = 999
        # 전환 쿨다운 카운터. 0보다 크면 전환 판단 자체를 건너뛴다(매 프레임 1씩 감소).
        # 전환이 실제로 일어나는 순간 min_avoidance_dwell_frames로 다시 채운다.
        self.avoidance_dwell_counter = 0
        # 차선 합류 감지용: has_lane_1이 False고 has_lane_2가 True인 프레임이 연속되면
        # 증가(그 반대도 대칭으로). lane_merge_confirm_frames에 도달하면 "합류"로 보고
        # 장애물 유무와 무관하게 오프셋을 강제로 되돌린다.
        self.lane1_absent_run = 0
        self.lane2_absent_run = 0
        # 추종 기준선 전환 디바운싱용 상태. stable_tracking_class가 "실제로 지금 쓰고
        # 있는" 클래스, pending/counter는 후보가 몇 프레임 연속 같았는지 추적한다.
        self.stable_tracking_class = self.fixed_lane_class or 'lane_2'
        self.pending_tracking_class = None
        self.track_switch_counter = 0
        # 행(target_point_y)별 마지막 유효 타겟 x좌표. 프레임당 변화량 제한(rate limit)에 쓴다.
        self.smoothed_target_x = {}
        # 추종 선 전환(final_offset_modifier) 발생 시 current_offset 연속성 보정용.
        # 자세한 이유는 아래 yolov8_detections_callback()의 보정 코드 주석 참고.
        self.prev_offset_modifier = 0.0
        self.obstacle_detected = False
        self.obstacle_dist = 999.0
        self.obstacle_pixel_x = -1.0
        # 검출된 장애물 전부: [(거리[m], 중심x[px], 반폭[px]), ...]
        self.obstacles = []

        # 차선 상태 전환 디바운싱용
        self.lane_change_counter = 0
        self.potential_next_state = None

        self.get_logger().info("Method B: BBox Overlap Logic with Debouncing Ready.")

    def obstacle_callback(self, msg: Point32):
        if msg.z == 1.0:
            self.obstacle_detected = True
            self.obstacle_dist = msg.x
            self.obstacle_pixel_x = msg.y # ★ 필수
        else:
            self.obstacle_detected = False
            self.obstacle_dist = 999.0
            self.obstacle_pixel_x = -1.0

    def obstacle_array_callback(self, msg: Polygon):
        """검출된 장애물 전부. 각 점 = (x=거리[m], y=중심x[px], z=반폭[px])."""
        self.obstacles = [(float(p.x), float(p.y), float(p.z)) for p in msg.points
                          if float(p.x) >= 0.0 and float(p.y) >= 0.0]

    def yolov8_detections_callback(self, detection_msg: DetectionArray):
        if len(detection_msg.detections) == 0: return

        # 차선 정보 추출 (Localization용)
        lane_1_box = None
        lane_2_box = None
        lane_1_cx, lane_2_cx = -1, -1
        has_lane_1, has_lane_2 = False, False

        for d in detection_msg.detections:
            if d.class_name == 'lane_1':
                lane_1_cx = d.bbox.center.position.x
                lane_1_box = d # 박스 정보 저장
                has_lane_1 = True
            elif d.class_name == 'lane_2':
                lane_2_cx = d.bbox.center.position.x
                lane_2_box = d # 박스 정보 저장
                has_lane_2 = True

        # 내 차선 판단 (이번 프레임의 '임시' 상태)
        detected_state = self.current_lane_state

        # fixed_lane_class가 지정되면 상태 판단/전환을 아예 하지 않고 그 차선만 추종한다.
        # 실측(주행 20초, 191프레임): lane_2 단독 91.1%, 둘 다 8.9%, lane_1 단독 0%.
        # 이 상태에서 상태머신을 돌리면 lane_2_far_right_threshold 근처에서 판정이
        # 흔들리고, 전환될 때마다 lane_width_pixel(280)만큼 타겟이 통째로 이동한다.
        if self.fixed_lane_class:
            detected_state = self.fixed_lane_class
            self.current_lane_state = self.fixed_lane_class
            self.lane_change_counter = 0
            self.potential_next_state = None
        elif has_lane_1 and has_lane_2:
            dist_1 = abs(lane_1_cx - self.image_center_x)
            dist_2 = abs(lane_2_cx - self.image_center_x)
            detected_state = 'lane_1' if dist_1 < dist_2 else 'lane_2'
        elif has_lane_2 and not has_lane_1:
            detected_state = 'lane_1' if lane_2_cx > self.lane_2_far_right_threshold else 'lane_2'
        elif has_lane_1 and not has_lane_2:
            detected_state = 'lane_2' if lane_1_cx < self.lane_1_far_left_threshold else 'lane_1'

        # 상태 변경 디바운싱: 같은 새 상태가 연속으로 lane_change_threshold_count번 잡혀야 전환
        if detected_state != self.current_lane_state:
            if detected_state == self.potential_next_state:
                self.lane_change_counter += 1
            else:
                self.potential_next_state = detected_state
                self.lane_change_counter = 1

            if self.lane_change_counter >= self.lane_change_threshold_count:
                self.get_logger().info(f"🔄 Lane State Changed: {self.current_lane_state} -> {detected_state}")
                self.current_lane_state = detected_state
                self.lane_change_counter = 0
        else:
            self.lane_change_counter = 0
            self.potential_next_state = None

        # ---------------------------------------------------
        # 2. [방식 B] BBox Overlap Check (겹침 확인)
        # ---------------------------------------------------
        tracking_class = self.current_lane_state

        # 장애물이 어디 있는지 동적으로 판단
        obstacle_in_lane_1 = False
        obstacle_in_lane_2 = False

        # 회피 대상 거리 안에 있는 장애물 전부를 본다. 가장 가까운 하나만 보면
        # 그걸 피해 지나가 박스가 사라지는 순간 판단에서 빠져, 옆 차선에 남아있는
        # 장애물을 놓치고 그쪽으로 꺾어 들어간다.
        nearby = [(cx, half_w) for dist, cx, half_w in self.obstacles
                  if dist < self.avoidance_trigger_dist]
        if not nearby and self.obstacle_detected and self.obstacle_dist < self.avoidance_trigger_dist:
            # 배열 토픽이 안 오는 구성에서의 하위 호환
            nearby = [(self.obstacle_pixel_x, 0.0)]

        if has_lane_1:
            l1_min = lane_1_box.bbox.center.position.x - (lane_1_box.bbox.size.x / 2)
            l1_max = lane_1_box.bbox.center.position.x + (lane_1_box.bbox.size.x / 2)
        if has_lane_2:
            l2_min = lane_2_box.bbox.center.position.x - (lane_2_box.bbox.size.x / 2)
            l2_max = lane_2_box.bbox.center.position.x + (lane_2_box.bbox.size.x / 2)

        for obs_cx, obs_half_w in nearby:
            # 박스 반폭만큼 넓혀서 겹침을 본다 (중심점만 보면 큰 장애물을 흘린다)
            o_min = obs_cx - obs_half_w
            o_max = obs_cx + obs_half_w

            if has_lane_1 and o_min < l1_max and o_max > l1_min:
                obstacle_in_lane_1 = True
            if has_lane_2 and o_min < l2_max and o_max > l2_min:
                obstacle_in_lane_2 = True

            # (만약 박스가 안 잡혔다면 픽셀 기준으로 대체)
            # lane_2=좌측선, lane_1=우측선 (실측 확인됨, README/주석의 예전 가정과 반대이니 주의)
            if not has_lane_1 and not has_lane_2:
                if obs_cx < self.image_center_x: obstacle_in_lane_2 = True
                else: obstacle_in_lane_1 = True

        # [정적 장애물 -> 한 번 겹침 확인되면 계속 기억(래치)]
        # 위 obstacle_in_lane_X는 avoidance_trigger_dist 이내 장애물만 대상이라, "겹침 확인"과
        # "거리 1.0m 이내"가 반드시 같은 프레임에 동시에 참이어야 발동했다. 장애물이 가까워질수록
        # (근접 구간) YOLO 박스가 흔들려 겹침 판정만 그 순간에 실패할 수 있는데, 그러면 회피를
        # 통째로 놓치고 그대로 들이받는다. 그래서 거리 제한 없이(전 구간) 겹침을 한 번이라도
        # 확인하면 래치해두고, 그 이후엔 (겹침이 그 프레임에 다시 안 잡혀도) 거리만 임계값
        # 이내로 들어오면 무조건 발동시킨다. 래치는 실제로 회피가 발동하는 순간 리셋된다.
        seen_lane_1_this_frame = False
        seen_lane_2_this_frame = False
        for dist, obs_cx, obs_half_w in self.obstacles:
            o_min = obs_cx - obs_half_w
            o_max = obs_cx + obs_half_w
            if has_lane_1 and o_min < l1_max and o_max > l1_min:
                seen_lane_1_this_frame = True
            if has_lane_2 and o_min < l2_max and o_max > l2_min:
                seen_lane_2_this_frame = True

        # [실측 확인된 버그] 단 한 프레임의 겹침만으로 영구 래치가 걸렸다 - 근접 구간에서
        # 차선 박스 폭이 순간적으로 넓게 잡히면(마스크 불안정, 오늘 밤 여러 번 확인됨) 반대쪽
        # 차선에 있는 장애물까지 한 프레임 겹쳐서 잘못된 방향으로 회피가 발동했다(실측: 오른쪽
        # lane_1 콘을 lane_2로 오판해 오히려 콘 쪽으로 틈). 그래서 래치를 걸기 전에 연속
        # 프레임 확인을 요구한다 - 노이즈성 단발 오판은 걸러지고, 진짜 장애물은 몇 프레임
        # 안에 계속 겹치므로 반응이 크게 느려지지 않는다.
        self.lane1_confirm_run = self.lane1_confirm_run + 1 if seen_lane_1_this_frame else 0
        self.lane2_confirm_run = self.lane2_confirm_run + 1 if seen_lane_2_this_frame else 0
        if self.lane1_confirm_run >= self.obstacle_confirm_frames:
            self.lane1_obstacle_confirmed = True
        if self.lane2_confirm_run >= self.obstacle_confirm_frames:
            self.lane2_obstacle_confirmed = True

        # "최근에 봤는지" 카운터 갱신(자연 감쇠 - 복귀 가드용, 위 영구 래치와는 별개).
        # [실측 확인된 버그] has_lane_X가 False인 프레임(그 차선 선 자체가 안 잡힘 - 근접
        # 구간에서 콘이 선을 가리는 경우가 흔함)에도 이 카운터가 그냥 계속 증가했었다.
        # "그 차선을 확인했는데 장애물이 없음"과 "그 차선을 아예 확인 못 함"을 구분 못 해서,
        # 콘이 왼쪽 선을 가려 has_lane_2=False가 이어지기만 해도 lane2_frames_since_seen이
        # 계속 올라가 복귀 가드(> obstacle_recent_frames_threshold)를 통과해버렸다 - 그 결과
        # 콘 코앞(0.39m)에서 "왼쪽 차선 클리어"로 오판하고 콘이 있는 왼쪽으로 되돌아갔다.
        # 이제 그 차선의 선이 실제로 보인 프레임에서만 갱신하고, 안 보이면(정보 없음) 그대로
        # 유지한다 - "선이 안 보임"을 "장애물 없음"으로 착각하지 않는다.
        if has_lane_1:
            self.lane1_frames_since_seen = 0 if seen_lane_1_this_frame else self.lane1_frames_since_seen + 1
        if has_lane_2:
            self.lane2_frames_since_seen = 0 if seen_lane_2_this_frame else self.lane2_frames_since_seen + 1

        # 추종할 차선 마스크가 안 보이면 반대쪽 차선을 대신 추종하고 오프셋으로 보정.
        # settled 판단(아래)이 이 보정값을 반영해야 해서 회피 판단보다 먼저 계산해둔다 -
        # 예전엔 이 계산이 회피 판단 뒤에 있어서, lane_2가 안 보여 이 보정이 걸린 채로
        # 오래 지속되면(final_offset_modifier가 0이 아닌 채 고정) settled가
        # active_avoidance_offset만 보고 판단해 영원히 False로 남아 회피 판단 자체가
        # 막혔었다("반대 방향으로 도는 것처럼" 보인 원인 중 하나).
        candidate_tracking_class = tracking_class
        candidate_offset_modifier = 0.0

        # lane_2=좌측선, lane_1=우측선 (실측 확인) -> lane_1_center = lane_2_center + lane_width_pixel
        if tracking_class == 'lane_1':
            if has_lane_1: candidate_tracking_class = 'lane_1'; candidate_offset_modifier = 0.0
            elif has_lane_2: candidate_tracking_class = 'lane_2'; candidate_offset_modifier = self.lane_width_pixel
        elif tracking_class == 'lane_2':
            if has_lane_2: candidate_tracking_class = 'lane_2'; candidate_offset_modifier = 0.0
            elif has_lane_1: candidate_tracking_class = 'lane_1'; candidate_offset_modifier = -self.lane_width_pixel

        # [실측 확인된 버그 - 근접 구간 플리커링] has_lane_1/has_lane_2가 프레임마다 홱홱
        # 뒤집히면(콘이 가까워질수록 흔함) candidate_tracking_class도 매 프레임 바뀌고,
        # 그때마다 아래 연속성 보정이 current_offset을 0<->lane_width_pixel로 매 프레임
        # 튕겨서 실제 조향 타겟도 같이 떨렸다. 그 결과 motion_planner의 조향 스무딩에
        # 걸려 "회피 오프셋은 280으로 잡혀 있는데 실제로는 거의 안 꺾인 것"처럼 되어
        # 그대로 장애물에 부딪혔다(실측 확인: 콘 앞에서 회피 실패). 그래서 차선 상태
        # 전환(위 lane_change_threshold_count)과 같은 패턴으로, candidate가 이만큼
        # 연속으로 같아야만 실제로 전환한다. 확정 전에는 직전까지 안정적으로 쓰던
        # 클래스를 그대로 유지한다 - 그 클래스 박스가 이번 프레임에 없으면 아래
        # draw_edges가 그릴 게 없어 "타겟 없음"으로 안전하게 처리될 뿐, 떨리지 않는다.
        if candidate_tracking_class == self.stable_tracking_class:
            self.track_switch_counter = 0
            self.pending_tracking_class = None
        else:
            if candidate_tracking_class == self.pending_tracking_class:
                self.track_switch_counter += 1
            else:
                self.pending_tracking_class = candidate_tracking_class
                self.track_switch_counter = 1
            if self.track_switch_counter >= self.track_class_switch_threshold:
                self.stable_tracking_class = candidate_tracking_class
                self.track_switch_counter = 0
                self.pending_tracking_class = None

        if self.stable_tracking_class == candidate_tracking_class:
            final_tracking_class = candidate_tracking_class
            final_offset_modifier = candidate_offset_modifier
        else:
            final_tracking_class = self.stable_tracking_class
            final_offset_modifier = self.prev_offset_modifier

        # [버그 수정] final_offset_modifier는 "추종 선이 바뀌었을 때 그 선의 중심을
        # 원래 선 기준 좌표로 맞추는" 보정인데, current_offset(회피 등으로 서서히
        # 이동 중인 값)은 이 전환을 전혀 모른 채 이전 프레임 값을 그대로 이어받는다.
        # 예: 회피 중(current_offset≈-150, lane_2 기준)에 lane_2 선이 화면에서
        # 사라져 lane_1으로 전환되면(final_offset_modifier 0->+280), 그 프레임의
        # 최종 타겟이 lane_width_pixel(280px)만큼 그대로 더 튄다 - 옆 차선이 아니라
        # 차선 밖으로 나가던 원인이 이것이었다. final_offset_modifier가 바뀐 만큼
        # current_offset도 같이 밀어줘야 실제 화면상 타겟 위치가 끊기지 않는다.
        if final_offset_modifier != self.prev_offset_modifier:
            self.current_offset += (final_offset_modifier - self.prev_offset_modifier)
        self.prev_offset_modifier = final_offset_modifier

        # 전략 수립 — "지금 실제로 달리고 있는 차선에 장애물이 확인되면 반대쪽으로
        # 옮기고, 그 차선을 계속 유지한다"로 통일. 예전엔 회피 후 장애물이 안 보이면
        # 자동으로 원래 차선에 돌아갔는데, 이제는 안 돌아가고 옮겨간 차선을 새 홈으로
        # 삼아 계속 달리다가, 그 차선에서 또 장애물을 만나야 다시 반대쪽으로 옮긴다.
        #
        # active_avoidance_offset==0.0 이면 지금 홈 차선(current_lane_state)에 있는
        # 것이고, ±lane_width_pixel이면 반대 차선에 가 있는 것이다. 진입/복귀 둘 다
        # "지금 있는 차선에 장애물이 확인됨"이라는 같은 종류의 이벤트라 똑같이 즉시
        # 반응한다(디바운싱 없음 — 장애물 확인 자체가 이미 확실한 신호이므로).
        # 미검출(정보 없음) 프레임은 아무 판단도 안 하고 지금 상태를 그대로 유지한다.
        # [좌/우 확정] lane_2=좌측선, lane_1=우측선 (실측 확인됨). 오프셋 부호는 이미지
        # 좌표 기준: 음수=왼쪽 이동, 양수=오른쪽 이동. 예전엔 반대(lane_1=좌/lane_2=우)로
        # 가정해서 부호가 다 반대였다 - 여기서 전부 뒤집었다.
        #
        # [와리가리 방지] 차선 이동이 아직 안 끝났는데(current_offset이 목표에 아직 안
        # 닿았는데) 그 사이 검출이 흔들려서 반대쪽 겹침이 잠깐 잡히면, 이동 중간에 바로
        # 되돌아가려 해서 왔다갔다한다. 그래서 "이전 목표에 이미 도착했을 때만" 새 판단을
        # 받아들이도록 강제한다 - 이동 완료 전에는 어느 쪽 래치가 들어와도 무시.
        # [와리가리 진짜 원인] 반대쪽 래치가 "지금 그 차선에 도착한 뒤" 새로 확인된 게
        # 아니라, 옮기기 전(예전 차선에 있을 때)부터 이미 켜져 있던 오래된 값일 수 있다.
        # 그러면 도착하자마자(settled) 바로 그 묵은 래치로 즉시 되돌아가고, 원래 차선의
        # 장애물은 여전히 있으니 다시 즉시 반대로 트리거되는 식으로 계속 왕복한다.
        # 그래서 실제로 전환이 발동하는 순간, 반대쪽 래치도 같이 지워서 "도착한 뒤 새로
        # 확인된 것"만 반영하게 한다.
        settled = abs(self.current_offset - (self.active_avoidance_offset + final_offset_modifier)) < self.shift_speed
        if self.avoidance_dwell_counter > 0:
            self.avoidance_dwell_counter -= 1
        can_switch = settled and self.avoidance_dwell_counter <= 0

        # [차선 합류 감지] lane_1으로 회피해 달리다가 lane_1 선이 "일시적으로 안 보임"이
        # 아니라 "실제로 끝나서"(=lane_2와 합류) 사라지면, 아래 회피 판단부는 장애물
        # 재발견 기준이라 장애물이 없는 합류 지점에서는 영원히 안 풀린다. 그 상태로 아래
        # final_offset_modifier가 lane_2의 진짜 중심선을 기준 삼아버려서 "lane_2 중심 +
        # 280px"(도로 밖)를 계속 타겟으로 잡는다 - 오른쪽으로 튀었다 직진했다 반복하며
        # 차선을 이탈한 원인이었다(실측 확인).
        # [실측 확인된 버그] 이 카운터를 settled 여부와 무관하게 매 프레임 올렸더니, 회피가
        # 막 트리거돼 아직 lane_1에 도착도 못 한 상태(current_offset이 한창 램프 중)에서도
        # "lane_1 선이 안 보임"이 그냥 정상인데(아직 그쪽으로 덜 틀었으니 안 보이는 게
        # 당연함) 합류로 오판해 15프레임 만에 회피를 강제로 취소해버렸다 - 정작 피해야 할
        # 장애물에 가까워지는 중에 도로 원래 차선(장애물 쪽)으로 되돌아가 그대로 부딪혔다.
        # 그래서 "이미 그 차선에 도착한(settled) 뒤"부터만 부재 프레임을 센다 - 이동 중에는
        # 안 보이는 게 당연하므로 합류 판단 자체를 보류한다.
        if settled:
            self.lane1_absent_run = self.lane1_absent_run + 1 if (not has_lane_1 and has_lane_2) else 0
            self.lane2_absent_run = self.lane2_absent_run + 1 if (not has_lane_2 and has_lane_1) else 0
        else:
            self.lane1_absent_run = 0
            self.lane2_absent_run = 0
        if self.active_avoidance_offset > 0.0 and self.lane1_absent_run >= self.lane_merge_confirm_frames:
            self.active_avoidance_offset = 0.0
            self.lane1_obstacle_confirmed = False
            self.lane2_obstacle_confirmed = False
            self.lane1_confirm_run = 0
            self.lane2_confirm_run = 0
            self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
            self.get_logger().warn("🔀 Lane 1 ended (merge into Lane 2) -> forced back to Lane 2")
        elif self.active_avoidance_offset < 0.0 and self.lane2_absent_run >= self.lane_merge_confirm_frames:
            self.active_avoidance_offset = 0.0
            self.lane1_obstacle_confirmed = False
            self.lane2_obstacle_confirmed = False
            self.lane1_confirm_run = 0
            self.lane2_confirm_run = 0
            self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
            self.get_logger().warn("🔀 Lane 2 ended (merge into Lane 1) -> forced back to Lane 1")

        # [진단용 임시 로그] 와리가리 원인 확인용. 확인 끝나면 지울 것.
        if self.obstacle_detected and self.obstacle_dist < self.avoidance_trigger_dist:
            self.get_logger().info(
                f"[avoid_debug] state={self.current_lane_state} settled={settled} "
                f"dwell={self.avoidance_dwell_counter} "
                f"offset={self.active_avoidance_offset:.0f} cur={self.current_offset:.0f} "
                f"dist={self.obstacle_dist:.2f} "
                f"L1now={obstacle_in_lane_1} L1latch={self.lane1_obstacle_confirmed} "
                f"L2now={obstacle_in_lane_2} L2latch={self.lane2_obstacle_confirmed}")
        if self.current_lane_state == 'lane_2':
            if can_switch and self.obstacle_detected and self.obstacle_dist < self.avoidance_trigger_dist:
                in_lane_1_now = self.active_avoidance_offset != 0.0
                if not in_lane_1_now and self.lane2_obstacle_confirmed:
                    self.active_avoidance_offset = self.lane_width_pixel
                    self.lane2_obstacle_confirmed = False
                    self.lane1_obstacle_confirmed = False
                    self.lane1_confirm_run = 0
                    self.lane2_confirm_run = 0
                    self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
                    self.get_logger().warn(f"🚧 Obs in Lane 2 -> Move to Lane 1 (RIGHT)")
                elif (in_lane_1_now and self.lane1_obstacle_confirmed and not obstacle_in_lane_2
                      and self.lane2_frames_since_seen > self.obstacle_recent_frames_threshold):
                    # obstacle_in_lane_2가 지금도 참이면, 지금 lane_1 겹침은 새 장애물이
                    # 아니라 근접 구간에서 박스가 커져 lane_2 것이 lane_1까지 걸친
                    # "같은 콘"일 가능성이 높다(실측 확인) - 그럴 땐 복귀 안 하고 계속 유지.
                    # [실측 확인된 버그] 두 차선 선이 다 안 잡히는 초근접 구간에서는 픽셀
                    # 폴백으로만 obstacle_in_lane_2가 갱신되는데, lane2_frames_since_seen은
                    # has_lane_2가 있어야만 갱신되는 별개 경로라 이 폴백을 못 따라가고 낡은
                    # 값(우연히 threshold 초과)에 멈춰 있을 수 있다. 그러면 "지금 이 순간
                    # obstacle_in_lane_2=True"인데도(콘이 바로 거기 있는데도) 카운터만 보고
                    # 복귀해버렸다(콘 39cm 앞에서 실측됨). 그래서 카운터와 별개로 "지금 이
                    # 순간" 신호(obstacle_in_lane_2)도 직접 거부권으로 넣는다 - 카운터가
                    # 낡았어도 실시간 신호가 "있음"이면 무조건 복귀를 막는다.
                    self.active_avoidance_offset = 0.0
                    self.lane1_obstacle_confirmed = False
                    self.lane2_obstacle_confirmed = False
                    self.lane1_confirm_run = 0
                    self.lane2_confirm_run = 0
                    self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
                    self.get_logger().warn(f"🚧 Obs in Lane 1 -> Move back to Lane 2 (LEFT)")
            # else: 미검출이거나 아직 이동/쿨다운 중 -> 아무것도 안 함 (상태/래치 유지)

        elif self.current_lane_state == 'lane_1':
            if can_switch and self.obstacle_detected and self.obstacle_dist < self.avoidance_trigger_dist:
                in_lane_2_now = self.active_avoidance_offset != 0.0
                if not in_lane_2_now and self.lane1_obstacle_confirmed:
                    self.active_avoidance_offset = -self.lane_width_pixel
                    self.lane1_obstacle_confirmed = False
                    self.lane2_obstacle_confirmed = False
                    self.lane1_confirm_run = 0
                    self.lane2_confirm_run = 0
                    self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
                    self.get_logger().warn(f"🚧 Obs in Lane 1 -> Move to Lane 2 (LEFT)")
                elif (in_lane_2_now and self.lane2_obstacle_confirmed and not obstacle_in_lane_1
                      and self.lane1_frames_since_seen > self.obstacle_recent_frames_threshold):
                    # obstacle_in_lane_1이 지금도 참이면 근접 구간에서 같은 콘의 박스가
                    # 커져 걸친 것일 가능성이 높다(실측 확인) - 그럴 땐 복귀 안 함.
                    # (반대쪽과 동일한 실시간 거부권 - 위 lane_2 분기 주석 참고)
                    self.active_avoidance_offset = 0.0
                    self.lane2_obstacle_confirmed = False
                    self.lane1_obstacle_confirmed = False
                    self.lane1_confirm_run = 0
                    self.lane2_confirm_run = 0
                    self.avoidance_dwell_counter = self.min_avoidance_dwell_frames
                    self.get_logger().warn(f"🚧 Obs in Lane 2 -> Move back to Lane 1 (RIGHT)")
            # else: 미검출이거나 아직 이동/쿨다운 중 -> 아무것도 안 함 (상태/래치 유지)

        self.target_offset = self.active_avoidance_offset

        # final_offset_modifier는 위(설정 판단 이전)에서 이미 계산해뒀다 - settled 판단이
        # 이 값을 반영해야 하기 때문. 여기서는 그 값 그대로 재사용만 한다.
        real_target_offset = self.target_offset + final_offset_modifier

        if self.current_offset < real_target_offset: self.current_offset = min(self.current_offset + self.shift_speed, real_target_offset)
        elif self.current_offset > real_target_offset: self.current_offset = max(self.current_offset - self.shift_speed, real_target_offset)

        # CPFL.draw_edges()는 detections[0].mask로 이미지 크기를 잡는다. 우리 쪽 yolov8_node는
        # cone/car_back(detect 모델, 마스크 없음)과 lane_seg를 합쳐서 발행하므로, 첫 검출이
        # 콘이면 크기가 0인 이미지가 만들어진다. 마스크가 있는 차선 검출만 따로 넘긴다.
        lane_msg = DetectionArray()
        lane_msg.header = detection_msg.header
        lane_msg.detections = [d for d in detection_msg.detections
                               if d.class_name in ('lane_1', 'lane_2') and d.mask.height > 0]
        if not lane_msg.detections:
            return

        try:
            edge_image = CPFL.draw_edges(lane_msg, cls_name=final_tracking_class, color=255)
            (h, w) = (edge_image.shape[0], edge_image.shape[1])
            dst_mat = [[round(w * 0.2), round(h * 0.0)], [round(w * 0.8), round(h * 0.0)], [round(w * 0.8), h], [round(w * 0.2), h]]

            bird_image_raw = CPFL.bird_convert(edge_image, srcmat=self.src_mat, dstmat=dst_mat)
            bird_image = cv2.convertScaleAbs(bird_image_raw)
            roi_image = CPFL.roi_rectangle_below(bird_image, cutting_idx=self.roi_cutting_idx)

            if self.show_image:
                debug_img = cv2.cvtColor(roi_image, cv2.COLOR_GRAY2BGR)
                cv2.putText(debug_img, f"State: {self.current_lane_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # 차선 검출 여부 (YOLO가 이번 프레임에 lane_1/lane_2 박스를 냈는지)
                l1_color = (0, 255, 0) if has_lane_1 else (0, 0, 255)
                l2_color = (0, 255, 0) if has_lane_2 else (0, 0, 255)
                cv2.putText(debug_img, f"Lane1:{'O' if has_lane_1 else 'X'}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, l1_color, 2)
                cv2.putText(debug_img, f"Lane2:{'O' if has_lane_2 else 'X'}", (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, l2_color, 2)

                # 차선별 장애물 상태: now=이번 프레임에 실제로 겹침 확인됨,
                # latch=거리 무관하게 한 번이라도 겹침 확인돼 회피 대기 중(래치)
                o1_color = (0, 0, 255) if (obstacle_in_lane_1 or self.lane1_obstacle_confirmed) else (200, 200, 200)
                o2_color = (0, 0, 255) if (obstacle_in_lane_2 or self.lane2_obstacle_confirmed) else (200, 200, 200)
                cv2.putText(debug_img,
                            f"ObsL1 now:{'Y' if obstacle_in_lane_1 else 'N'} latch:{'Y' if self.lane1_obstacle_confirmed else 'N'}",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, o1_color, 2)
                cv2.putText(debug_img,
                            f"ObsL2 now:{'Y' if obstacle_in_lane_2 else 'N'} latch:{'Y' if self.lane2_obstacle_confirmed else 'N'}",
                            (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, o2_color, 2)
                cv2.imshow('Lane Info (ROI)', debug_img)
                cv2.waitKey(1)
        except Exception: return

        grad = CPFL.dominant_gradient(roi_image, theta_limit=70)
        target_points = []
        for target_point_y in range(self.target_y_start, self.target_y_end, self.target_y_step):
            # lane_2는 좌측선, lane_1은 우측선이다(실측 확인). 어느 쪽인지 알려주면 한쪽 선만
            # 보일 때 중심을 어느 방향으로 밀지 기울기 부호로 추측하지 않아도 된다.
            target_point_x = CPFL.get_lane_center(roi_image, detection_height=target_point_y,
                                                  detection_thickness=10, road_gradient=grad,
                                                  lane_width=(self.lane_width_for_center_lane1
                                                              if final_tracking_class == 'lane_1'
                                                              else self.lane_width_for_center),
                                                  line_side=('right' if final_tracking_class == 'lane_1'
                                                             else 'left'),
                                                  tilt_comp=self.lane_center_tilt_comp,
                                                  force_single_line=(self.lane_center_force_single_line_lane1
                                                                     if final_tracking_class == 'lane_1'
                                                                     else self.lane_center_force_single_line))
            if target_point_x != -1:
                # [프레임간 노이즈 방지] 이 행에서 직전에 유효했던 값 대비 변화량을 제한한다.
                # get_lane_center()가 매 프레임 기억 없이 새로 계산해서, 근접 구간 노이즈로
                # "두 선 보임" 오판이 나면 행당 타겟이 프레임마다 수백 px씩 튀었다(실측 확인
                # - 회피 오프셋은 안정적인데 차가 물리적으로 와리가리치다 부딪힌 원인).
                prev_x = self.smoothed_target_x.get(target_point_y)
                if prev_x is not None:
                    delta = target_point_x - prev_x
                    if delta > self.target_x_rate_limit:
                        target_point_x = prev_x + self.target_x_rate_limit
                    elif delta < -self.target_x_rate_limit:
                        target_point_x = prev_x - self.target_x_rate_limit
                self.smoothed_target_x[target_point_y] = target_point_x

                final_x = target_point_x + self.current_offset
                final_x = max(0, min(640, final_x))
            else: final_x = -1
            tp = TargetPoint(); tp.target_x = round(final_x); tp.target_y = round(target_point_y); target_points.append(tp)

        lane = LaneInfo(); lane.slope = grad; lane.target_points = target_points
        self.publisher.publish(lane)
        try: self.roi_image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(cv2.convertScaleAbs(roi_image), encoding="mono8"))
        except: pass

def main(args=None):
    rclpy.init(args=args); node = Yolov8InfoExtractor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); cv2.destroyAllWindows(); rclpy.shutdown()
if __name__ == '__main__': main()
