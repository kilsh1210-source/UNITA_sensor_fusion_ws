import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point32, Polygon
from interfaces_pkg.msg import LaneInfo, PathPlanningResult, LatticeDebug
import numpy as np
from scipy.interpolate import CubicSpline

#---------------Variable Setting---------------
SUB_LANE_TOPIC_NAME = "yolov8_lane_info"  # lane_info_extractor 노드에서 퍼블리시하는 타겟 지점 토픽
SUB_OBSTACLE_TOPIC_NAME = "/lidar_obstacle_info"  # 가장 가까운 장애물 1개 (호환용)
SUB_OBSTACLE_ARRAY_TOPIC_NAME = "/lidar_obstacle_array"  # 검출된 장애물 전부
PUB_TOPIC_NAME = "path_planning_result"   # 경로 계획 결과 퍼블리시 토픽
PUB_LATTICE_DEBUG_TOPIC_NAME = "lattice_debug"  # 후보 경로 디버그 시각화용 퍼블리시 토픽
CAR_CENTER_POINT = (320, 179) # 이미지 상에서 차량 앞 범퍼의 중심이 위치한 픽셀 좌표

# 후보 경로 시각화 on/off 토글 (True로 바꾸면 lattice_debug 토픽 발행 시작)
ENABLE_LATTICE_DEBUG = False

# ==============================================================================
# [Lattice Planner 옵션 기본값 파라미터]
# ==============================================================================
# 1. 생성할 후보 경로(Candidate Paths)의 개수 
#    - 예: 7개 -> [최좌측, 좌측2, 좌측1, 중앙, 우측1, 우측2, 최우측]
DEFAULT_CANDIDATE_COUNT = 7

# 2. 기준 차선 중심선으로부터 오프셋을 줄 최대 좌/우 픽셀 거리 범위
#    - 예: 120.0 -> 왼쪽으로 최대 -120px, 오른쪽으로 최대 +120px 범위 내에서 후보 경로 생성
DEFAULT_MAX_LATTICE_OFFSET = 120.0

# 3. 최적 경로 선택 후, Cubic Spline 보간을 통해 최종적으로 보낼 경로 좌표 점(Waypoint)의 개수
DEFAULT_SPLINE_POINTS = 100

# 4. 장애물 패널티 기본 스케일링 계수 (추후 패널티 수식 확장용 스케일러)
DEFAULT_OBSTACLE_PENALTY_GAIN = 3.0

# 5. [가중치] 차선 중앙 유지 가중치
#    - 높을수록 평소 장애물이 없을 때 차선 중앙으로 바짝 붙으려는 성향이 강해짐
DEFAULT_LANE_CENTER_WEIGHT = 10.0

# 6. [가중치] 장애물 회피 가중치
#    - 높을수록 장애물이 나타났을 때 차선 중앙을 벗어나더라도 장애물에서 멀어지려는 성향이 강해짐
DEFAULT_OBSTACLE_WEIGHT = 20.0

# 7. [가중치] 경로 부드러움(스무딩) 가중치
#    - 높을수록 핸들을 급격하게 꺾는 경로에 감점을 주어 완만한 곡선 경로를 선호하게 됨
DEFAULT_SMOOTHNESS_WEIGHT = 10.

# 8. [조건] 장애물 회피 로직을 개입시킬 거리의 기준 (단위: 미터)
#    - LiDAR가 측정한 장애물 거리가 이 값(2.4m)보다 가까워질 때만 회피 감점을 적용
DEFAULT_OBSTACLE_DIST_THRESHOLD = 2.4

# 9. [가중치] 차로(오프셋) 변경 비용
#    - 높을수록 직전 프레임에 선택했던 오프셋에서 먼 후보일수록 감점이 커져,
#      매 프레임 후보가 바뀌며 조향이 떨리는(지그재그) 현상을 억제함
#      단, 너무 높으면 장애물이 나타났을 때도 차로 변경을 꺼려하는 성향이 강해질 수 있음
DEFAULT_PATH_CHANGE_COST = 10.05

# 10. [조건] 장애물 안전 거리 (단위: 픽셀)
#     - 후보 경로와 장애물 사이 거리가 이 값보다 가까우면 obstacle_penalty_gain을 이용해
#       추가로 큰 페널티를 부과함 (obstacle_weight의 완만한 감점과 별개로 최소 이격거리를 강제)
DEFAULT_OBSTACLE_CLEARANCE = 40.0
#----------------------------------------------


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # 파라미터 선언
        self.sub_lane_topic = self.declare_parameter('sub_lane_topic', SUB_LANE_TOPIC_NAME).value
        self.sub_obstacle_topic = self.declare_parameter('sub_obstacle_topic', SUB_OBSTACLE_TOPIC_NAME).value
        self.sub_obstacle_array_topic = self.declare_parameter(
            'sub_obstacle_array_topic', SUB_OBSTACLE_ARRAY_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.car_center_point = self.declare_parameter('car_center_point', CAR_CENTER_POINT).value

        self.candidate_count = int(self.declare_parameter('candidate_count', DEFAULT_CANDIDATE_COUNT).value)
        self.max_lattice_offset = float(self.declare_parameter('max_lattice_offset', DEFAULT_MAX_LATTICE_OFFSET).value)
        self.spline_points = int(self.declare_parameter('spline_points', DEFAULT_SPLINE_POINTS).value)
        self.obstacle_penalty_gain = float(self.declare_parameter('obstacle_penalty_gain', DEFAULT_OBSTACLE_PENALTY_GAIN).value)
        self.lane_center_weight = float(self.declare_parameter('lane_center_weight', DEFAULT_LANE_CENTER_WEIGHT).value)
        self.obstacle_weight = float(self.declare_parameter('obstacle_weight', DEFAULT_OBSTACLE_WEIGHT).value)
        self.smoothness_weight = float(self.declare_parameter('smoothness_weight', DEFAULT_SMOOTHNESS_WEIGHT).value)
        self.obstacle_dist_threshold = float(self.declare_parameter('obstacle_dist_threshold', DEFAULT_OBSTACLE_DIST_THRESHOLD).value)
        self.path_change_cost = float(self.declare_parameter('path_change_cost', DEFAULT_PATH_CHANGE_COST).value)
        self.obstacle_clearance = float(self.declare_parameter('obstacle_clearance', DEFAULT_OBSTACLE_CLEARANCE).value)

        # 후보 경로 시각화 디버그 토글
        self.enable_lattice_debug = bool(self.declare_parameter('enable_lattice_debug', ENABLE_LATTICE_DEBUG).value)
        self.pub_lattice_debug_topic = self.declare_parameter('pub_lattice_debug_topic', PUB_LATTICE_DEBUG_TOPIC_NAME).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.target_points = []
        self.obstacle_detected = False
        self.obstacle_dist = float('inf')
        self.obstacle_pixel_x = -1.0
        # 검출된 장애물 전부: [(거리[m], 중심x[px], 반폭[px]), ...]
        # 가장 가까운 하나만 보면, 그걸 피해 지나가 박스가 사라지는 순간 판단에서 빠져
        # 다음 장애물만 보고 꺾다가 직전 장애물을 들이받는다.
        self.obstacles = []
        self.prev_selected_offset = 0.0  # path_change_cost 계산용: 직전 프레임에 선택된 오프셋

        # 구독 및 퍼블리셔 설정
        self.lane_sub = self.create_subscription(LaneInfo, self.sub_lane_topic, self.lane_callback, self.qos_profile)
        self.obstacle_sub = self.create_subscription(Point32, self.sub_obstacle_topic, self.obstacle_callback, self.qos_profile)
        self.obstacle_array_sub = self.create_subscription(
            Polygon, self.sub_obstacle_array_topic, self.obstacle_array_callback, self.qos_profile)
        self.publisher = self.create_publisher(PathPlanningResult, self.pub_topic, self.qos_profile)
        self.lattice_debug_publisher = self.create_publisher(LatticeDebug, self.pub_lattice_debug_topic, self.qos_profile)

    def lane_callback(self, msg: LaneInfo):
        self.target_points = msg.target_points
        if len(self.target_points) >= 3:
            self.plan_path()

    def obstacle_callback(self, msg: Point32):
        self.obstacle_detected = bool(msg.z == 1.0)
        if self.obstacle_detected:
            self.obstacle_dist = float(msg.x)
            self.obstacle_pixel_x = float(msg.y)
        else:
            self.obstacle_dist = float('inf')
            self.obstacle_pixel_x = -1.0

    def obstacle_array_callback(self, msg: Polygon):
        """검출된 장애물 전부를 받는다. 각 점 = (x=거리[m], y=중심x[px], z=반폭[px])."""
        self.obstacles = [(float(p.x), float(p.y), float(p.z)) for p in msg.points
                          if float(p.x) >= 0.0 and float(p.y) >= 0.0]

    def plan_path(self):
        valid_points = [(tp.target_x, tp.target_y) for tp in self.target_points if tp.target_x >= 0]
        if len(valid_points) < 3:
            self.get_logger().warn("Not enough valid lane points for lattice planning")
            return

        x_points, y_points = zip(*valid_points)
        x_points = list(x_points)
        y_points = list(y_points)

        # 차량 앞 범퍼 중심을 기준 경로에 추가
        y_points.append(self.car_center_point[1])
        x_points.append(self.car_center_point[0])

        # y 기준으로 정렬하고 중복 y 제거
        sorted_points = sorted(zip(y_points, x_points), key=lambda point: point[0])
        y_points, x_points = zip(*sorted_points)
        y_points = np.array(y_points, dtype=np.float64)
        x_points = np.array(x_points, dtype=np.float64)

        unique_y, unique_idx = np.unique(y_points, return_index=True)
        y_points = unique_y
        x_points = x_points[unique_idx]

        if len(y_points) < 3:
            self.get_logger().warn("Need at least 3 unique y points for lattice planning")
            return

        candidates, offsets = self.generate_candidate_paths(x_points, y_points)
        best_index = self.select_best_candidate(candidates, offsets, x_points)
        selected_x = candidates[best_index]

        if self.enable_lattice_debug:
            self.publish_lattice_debug(candidates, y_points, best_index)

        # 선택된 경로를 스플라인으로 부드럽게 보간
        spline = CubicSpline(y_points, selected_x, bc_type='natural')
        y_new = np.linspace(min(y_points), max(y_points), self.spline_points)
        x_new = spline(y_new)

        path_msg = PathPlanningResult()
        path_msg.x_points = list(x_new)
        path_msg.y_points = list(y_new)
        self.publisher.publish(path_msg)

        self.get_logger().info(f"Lattice path selected offset {np.mean(selected_x - x_points):.1f} px")

        self.target_points.clear()

    def generate_candidate_paths(self, x_points: np.ndarray, y_points: np.ndarray):
        offsets = np.linspace(-self.max_lattice_offset, self.max_lattice_offset, self.candidate_count)
        candidates = []

        for offset in offsets:
            transition = np.linspace(0.0, offset, len(x_points))
            candidate_x = x_points + transition
            candidates.append(candidate_x)

        return candidates, offsets

    def _active_obstacles(self):
        """페널티 대상 장애물 목록. 배열 토픽이 있으면 그걸 쓰고, 없으면 단일 토픽으로 대체.

        반환: [(중심x[px], 반폭[px]), ...]
        """
        if self.obstacles:
            return [(cx, half_w) for dist, cx, half_w in self.obstacles
                    if dist < self.obstacle_dist_threshold]
        # 배열 토픽이 안 오는 구성(예전 image_fusion_node)에서의 하위 호환
        if (self.obstacle_detected and self.obstacle_dist < self.obstacle_dist_threshold
                and self.obstacle_pixel_x >= 0):
            return [(self.obstacle_pixel_x, 0.0)]
        return []

    def _obstacle_penalty(self, candidate_x):
        """후보 경로 하나에 대해, 모든 장애물의 페널티를 합산한다.

        박스 반폭을 안전거리에 더해서, 큰 장애물일수록 더 멀리 비켜가게 한다.
        """
        penalty = 0.0
        path_x = float(np.mean(candidate_x))
        for cx, half_w in self._active_obstacles():
            gap = abs(path_x - cx)
            penalty += self.obstacle_weight * max(
                0.0, 1.0 - gap / (self.max_lattice_offset * 2.0))
            clearance = self.obstacle_clearance + half_w
            if gap < clearance:
                penalty += self.obstacle_penalty_gain * (clearance - gap)
        return penalty

    def select_best_candidate(self, candidates, offsets, x_points):
        """
        [수정 이력] lane_penalty는 원래 mean(|candidate_x - mean(candidate_x)|)였다. 이건
        '차선 중앙(x_points)과의 거리'가 아니라 '후보가 자기 평균에서 흩어진 정도'를 재는
        식이라, 곡선에서 offset=0(경로가 실제 곡률을 따름, 비용 225)보다 offset=+80(경로가
        펴짐, 비용 42.5)이 더 낮은 비용으로 나와 경로가 곡선을 안쪽으로 가로질렀다.
        (path_change_cost를 10.0으로 올려 이 경로를 임시로 막아 원인을 확인했었다.)
        이제 후보를 실제 차선 중심선(x_points)과 비교하도록 고쳤다: offset=0이면 0,
        |offset|이 커질수록 커지는 정상적인 '중앙 유지' 페널티가 된다.
        """
        scores = []
        for idx, candidate_x in enumerate(candidates):
            lane_penalty = np.mean(np.abs(candidate_x - x_points)) * self.lane_center_weight

            # 검출된 장애물 전부에 대해 페널티를 합산한다. 예전에는 가장 가까운 하나만
            # 봤는데, 그러면 그걸 피해 지나가 박스가 사라지는 순간 판단에서 통째로 빠지고
            # 다음 장애물만 보고 꺾다가 직전 장애물을 들이받았다.
            obstacle_penalty = self._obstacle_penalty(candidate_x)

            smoothness_penalty = self.smoothness_weight * np.mean(np.abs(np.diff(candidate_x, n=2))) if len(candidate_x) >= 3 else 0.0

            # 직전에 선택했던 오프셋에서 멀어질수록 감점 (차로 변경 비용)
            change_penalty = self.path_change_cost * abs(offsets[idx] - self.prev_selected_offset)

            scores.append(lane_penalty + obstacle_penalty + smoothness_penalty + change_penalty)

        best_index = int(np.argmin(scores))
        self.prev_selected_offset = float(offsets[best_index])
        return best_index

    def publish_lattice_debug(self, candidates, y_points: np.ndarray, best_index: int):
        debug_msg = LatticeDebug()
        debug_msg.y_points = [float(y) for y in y_points]
        debug_msg.x_points = [float(x) for candidate_x in candidates for x in candidate_x]
        debug_msg.candidate_count = len(candidates)
        debug_msg.best_index = best_index
        self.lattice_debug_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
