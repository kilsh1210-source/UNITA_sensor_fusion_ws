#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand


# --------------- Tunable Params (Top) ---------------
# Topics
SUB_PATH_TOPIC_NAME = "path_planning_result"          # 수신할 경로 토픽
PUB_TOPIC_NAME = "topic_control_signal"               # 발행할 모션 명령 토픽
SUB_DETECTION_TOPIC_NAME = "detections"               # 객체 검출 결과 토픽
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"  # 신호등 인식 토픽

# Feature toggles
USE_TRAFFIC_LIDAR_STOP = True                         # 신호등/라이다 정지 로직 사용 여부
USE_PD = True                                         # PD 보정 조향 사용 여부

# Vehicle/Image
CAR_CENTER_POINT = [320, 179]                         # 차량 기준 픽셀 좌표 (x, y)
CAR_CENTER_X = 320                                    # PD 횡오차 기준 중심 x
VEHICLE_HEADING_RAD = -1.57079632679                  # 차량 진행방향 라디안(기본 위쪽)
TIMER = 0.1                                           # 제어 주기(초), 0.1=10Hz

# Pure Pursuit
LOOKAHEAD_DISTANCE = 170.0                            # 목표점 거리(작을수록 민감, 클수록 완만)
WHEELBASE = 50.0                                      # 가상 휠베이스(클수록 조향 계산 완만)
MAX_STEER_ANGLE_RAD = 0.55                            # 조향각 정규화 기준(작을수록 출력 커짐)
MAX_STEER_CMD = 9.0                                   # 최종 조향 명령 최대 절대값

# PD
KP = 0.01                                             # 횡오차 비례 이득(즉각 조향 강도) 초기값 0.05
KD = 0.045                                             # 변화량 이득(진동 억제/선행 보정)
MAX_PD_STEER = 4.0                                    # PD 보정 최대 절대값
LOOKAHEAD_Y = 155                                     # PD가 참조할 y 라인(화면 아래쪽일수록 가까움)

# Steering smoothing
# 조향 명령은 MotionCommand.steering(int32)으로 나가면서 round()되고, serial_sender가
# max_steer_cmd로 나눠 -1.0~1.0으로 정규화한다. 즉 명령 1칸 = 전체 조향각의 1/max_steer_cmd.
# max_steer_cmd=9면 한 칸이 조향 포텐셔미터로 약 8(좌)~12(우) counts인데, 펌웨어의
# STEERING_DEADBAND가 6이라 한 칸만 바뀌어도 조향 모터가 켜졌다 꺼진다. 저속에서 같은
# 곡률을 도는 데 제어 틱이 더 많이 들어가면 이 계단이 하나씩 다 느껴진다(틱틱거림).
# 아래 두 값이 한 번의 큰 변화를 여러 틱에 나눠 내보내 그 충격을 줄인다.
STEER_SMOOTHING_ALPHA = 0.4                           # 1.0이면 스무딩 없음(원본 동작). 작을수록 부드럽고 반응이 느려짐
STEER_RATE_LIMIT = 3.0                                # 제어 틱당 허용 변화량(steer_cmd 단위). 0 이하면 제한 없음

# Speed
BASE_SPEED = 200                                      # 기본 주행 속도
MIN_SPEED = 100                                      # 최소 속도 하한
MAX_SPEED = 250                                # 최대 속도 상한
STEER_SPEED_GAIN = 12.0                               # 조향 클수록 감속시키는 계수
# ----------------------------------------------------


def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class UnitaPurePursuitNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")

        # -------------------------
        # Topic parameters
        # -------------------------
        self.sub_path_topic = self.declare_parameter("sub_path_topic", SUB_PATH_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter("pub_topic", PUB_TOPIC_NAME).value

        self.use_traffic_lidar_stop = bool(
            self.declare_parameter("use_traffic_lidar_stop", USE_TRAFFIC_LIDAR_STOP).value
        )
        self.sub_detection_topic = self.declare_parameter("sub_detection_topic", SUB_DETECTION_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter("sub_traffic_light_topic", SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.timer_period = float(self.declare_parameter("timer", TIMER).value)

        # -------------------------
        # Pure Pursuit parameters
        # -------------------------
        self.lookahead_distance = float(self.declare_parameter("lookahead_distance", LOOKAHEAD_DISTANCE).value)
        self.wheelbase = float(self.declare_parameter("wheelbase", WHEELBASE).value)
        self.max_steer_angle = float(self.declare_parameter("max_steer_angle_rad", MAX_STEER_ANGLE_RAD).value)
        self.max_steer_cmd = float(self.declare_parameter("max_steer_cmd", MAX_STEER_CMD).value)

        cp = self.declare_parameter("car_center_point", CAR_CENTER_POINT).value
        self.car_center_point = (int(cp[0]), int(cp[1]))
        self.vehicle_heading = float(self.declare_parameter("vehicle_heading_rad", VEHICLE_HEADING_RAD).value)

        # -------------------------
        # PD parameters
        # -------------------------
        self.use_pd = bool(self.declare_parameter("use_pd", USE_PD).value)
        self.prev_dx = 0.0
        self.car_center_x = int(self.declare_parameter("car_center_x", CAR_CENTER_X).value)

        self.Kp = float(self.declare_parameter("Kp", KP).value)
        self.Kd = float(self.declare_parameter("Kd", KD).value)
        self.lookahead_y = int(self.declare_parameter("lookahead_y", LOOKAHEAD_Y).value)

        self.max_steer = float(self.declare_parameter("max_steer", MAX_PD_STEER).value)

        # -------------------------
        # Steering smoothing
        # -------------------------
        self.steer_smoothing_alpha = float(
            self.declare_parameter("steer_smoothing_alpha", STEER_SMOOTHING_ALPHA).value
        )
        self.steer_rate_limit = float(self.declare_parameter("steer_rate_limit", STEER_RATE_LIMIT).value)
        # 필터 상태는 반드시 round() 전의 float으로 들고 있어야 한다.
        # 반올림된 값을 되먹이면 필터가 정수 격자에 갇혀 목표에 영영 못 닿는다.
        self.steer_state = 0.0

        self.base_speed = int(self.declare_parameter("base_speed", BASE_SPEED).value)
        self.min_speed = int(self.declare_parameter("min_speed", MIN_SPEED).value)
        self.max_speed = int(self.declare_parameter("max_speed", MAX_SPEED).value)
        self.steer_speed_gain = float(self.declare_parameter("steer_speed_gain", STEER_SPEED_GAIN).value)

        # QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Data holders
        self.path_data = None
        self.detection_data = None
        self.traffic_light_data = None
        self.lidar_data = None

        # Subscribers
        self.path_sub = self.create_subscription(
            PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile
        )

        if self.use_traffic_lidar_stop:
            self.detection_sub = self.create_subscription(
                DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile
            )
            self.traffic_light_sub = self.create_subscription(
                String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile
            )

        # Publisher
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # Timer
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # -------------------------
    # Callbacks
    # -------------------------
    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    # -------------------------
    # Helpers
    # -------------------------
    def find_lookahead_point(self, path):
        car_x, car_y = self.car_center_point

        forward_points = [p for p in path if p[1] <= car_y]
        if not forward_points:
            forward_points = path

        # 경로는 y 오름차순, 즉 "먼 쪽 -> 가까운 쪽" 순으로 들어온다.
        # 순수 추종은 차량에서 lookahead_distance 이상 떨어진 '첫' 점을 봐야 하므로
        # 가까운 쪽부터 훑어야 한다. 먼 쪽부터 훑으면 최원점이 언제나 조건을 만족해
        # lookahead_distance 값과 무관하게 그 한 점만 선택된다(경로 최대 길이는
        # 차량 y=179, 최원점 y=5로 174px라 어떤 설정값이든 첫 검사에서 통과).
        # 그 결과 곡선에서 중간 형상을 못 읽고 최원점을 직선으로 겨냥해 안쪽을 가로질렀다.
        for p in reversed(forward_points):
            dx = p[0] - car_x
            dy = p[1] - car_y
            if math.hypot(dx, dy) >= self.lookahead_distance:
                return p

        # 경로가 lookahead_distance보다 짧으면 가장 먼 점으로 대체
        return forward_points[0] if forward_points else None

    def compute_pp_steer_cmd(self, path):
        if not path:
            return 0.0
        if self.lookahead_distance <= 0.0 or self.max_steer_angle <= 0.0:
            return 0.0

        lookahead = self.find_lookahead_point(path)
        if lookahead is None:
            return 0.0

        car_x, car_y = self.car_center_point
        lx, ly = lookahead

        # 분모는 파라미터가 아니라 '실제로 고른 점까지의 거리'여야 한다.
        # 선택이 정상이면 둘이 거의 같지만, 경로가 lookahead_distance보다 짧아
        # 더 먼 점으로 대체된 경우 파라미터를 쓰면 그 비율만큼 조향이 과해진다.
        # (예전: 실거리 174px인데 분모에 120을 써서 1.45배 과조향)
        lookahead_dist = math.hypot(lx - car_x, ly - car_y)
        if lookahead_dist < 1e-6:
            return 0.0

        target_angle = math.atan2(ly - car_y, lx - car_x)
        alpha = normalize_angle(target_angle - self.vehicle_heading)

        steer_angle = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            lookahead_dist,
        )

        steer_cmd = (steer_angle / self.max_steer_angle) * self.max_steer_cmd
        steer_cmd = clamp(steer_cmd, -self.max_steer_cmd, self.max_steer_cmd)
        return float(steer_cmd)

    def compute_pd_steer_cmd(self, path):
        if not path or len(path) < 5:
            self.prev_dx = 0.0
            return 0.0

        x_target, _y_target = min(path, key=lambda p: abs(p[1] - self.lookahead_y))
        dx = float(x_target - self.car_center_x)

        steer = self.Kp * dx + self.Kd * (dx - self.prev_dx)
        self.prev_dx = dx

        steer = clamp(steer, -self.max_steer, self.max_steer)
        return float(steer)

    def smooth_steer(self, steer_cmd):
        """조향 명령을 EMA + 레이트리밋으로 완만하게 만든다.

        EMA가 큰 변화를 지수적으로 나눠 내보내고, 레이트리밋이 그 첫 틱의 크기에
        상한을 건다. 둘 다 self.steer_state(float)를 기준으로 계산한다.

        주의: 입력이 ±max_steer_cmd를 매 틱 오가는 극단적 진동이면 레이트리밋이 계속
        걸리면서 출력 평균이 0이 아닌 쪽으로 치우친다(±9 입력 -> 0~3 왕복). 그 정도로
        떨고 있으면 스무딩이 아니라 상류(경로/PD)를 봐야 한다. 실제 진폭인 ±2~4에서는
        레이트리밋이 걸리지 않아 대칭이다.
        """
        target = float(steer_cmd)

        if 0.0 < self.steer_smoothing_alpha < 1.0:
            target = (self.steer_smoothing_alpha * target
                      + (1.0 - self.steer_smoothing_alpha) * self.steer_state)

        if self.steer_rate_limit > 0.0:
            delta = clamp(target - self.steer_state, -self.steer_rate_limit, self.steer_rate_limit)
            target = self.steer_state + delta

        self.steer_state = clamp(target, -self.max_steer_cmd, self.max_steer_cmd)
        return self.steer_state

    def publish_stop(self):
        """정지 명령. 조향도 0으로 나가므로(펌웨어가 rear_pwm과 무관하게 targetSteering을
        갱신함) 필터 상태도 0으로 맞춰야 다음 출발 때 옛날 값에서 이어지지 않는다."""
        self.steer_state = 0.0
        self.prev_dx = 0.0
        self.publish_cmd(0.0, 0, 0)

    def should_stop_by_traffic(self):
        if self.traffic_light_data is None:
            return False
        if self.traffic_light_data.data != "Red":
            return False

        if self.detection_data is None:
            return True

        for detection in self.detection_data.detections:
            if getattr(detection, "class_name", "") == "traffic_light":
                y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2)
                if y_max < 150:
                    return True
        return False

    # -------------------------
    # Main loop
    # -------------------------
    def timer_callback(self):
        if not self.path_data:
            self.publish_stop()
            return

        if self.use_traffic_lidar_stop:
            if self.lidar_data is not None and self.lidar_data.data is True:
                self.publish_stop()
                return
            if self.should_stop_by_traffic():
                self.publish_stop()
                return

        steer_pp = self.compute_pp_steer_cmd(self.path_data)

        steer_pd = 0.0
        if self.use_pd:
            steer_pd = self.compute_pd_steer_cmd(self.path_data)

        steer_cmd = steer_pp + steer_pd
        steer_cmd = clamp(steer_cmd, -self.max_steer_cmd, self.max_steer_cmd)
        steer_cmd = self.smooth_steer(steer_cmd)

        speed = int(self.base_speed - self.steer_speed_gain * abs(steer_cmd))
        speed = int(clamp(speed, self.min_speed, self.max_speed))

        self.publish_cmd(steer_cmd, speed, speed)

    def publish_cmd(self, steering, left_speed, right_speed):
        msg = MotionCommand()
        msg.steering = int(round(steering))
        msg.left_speed = int(left_speed)
        msg.right_speed = int(right_speed)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnitaPurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()