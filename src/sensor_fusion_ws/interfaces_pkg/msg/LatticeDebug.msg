# Lattice 후보 경로 디버그 정보 (path_planner_node -> 시각화 노드)
# 시각화 전용 메시지로, 최종 주행 경로(path_planning_result)와는 별개로 발행됨

# 모든 후보 경로가 공통으로 사용하는 y좌표들
float64[] y_points

# 모든 후보의 x좌표를 이어붙인 1차원 배열
# candidate i의 x좌표 = x_points[i * len(y_points) : (i + 1) * len(y_points)]
float64[] x_points

# 생성된 후보 경로 개수
int32 candidate_count

# candidates 중 최종 선택된 경로의 인덱스
int32 best_index
