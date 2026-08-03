# lidar_cluster_pkg

라이다 스캔에서 물체 덩어리(클러스터)를 찾아 RViz에서 시각화하는 패키지. 카메라와는 무관하게 라이다
데이터만으로 "주변에 뭐가 몇 개 있는지" 확인하고 싶을 때 쓰는 디버깅/시각화용 도구.
`fusion_bringup.launch.py`에는 포함되어 있지 않으며, 필요할 때 따로 실행한다.

## 노드

### `scan_cluster_node`

`/scan`(`sensor_msgs/LaserScan`)의 포인트들을 스캔 순서 기준으로 순회하면서, 인접한 두 포인트 사이 거리가
`cluster_tolerance` 이하면 같은 클러스터로 묶는 단순 순차 클러스터링(sequential Euclidean clustering).

- 구독: `/scan`
- 발행: `/lidar_clusters` (`visualization_msgs/MarkerArray`) — 클러스터별 포인트, 대표점(centroid에 가장
  가까운 실제 포인트), 거리와 포인트 개수를 표시하는 텍스트 라벨
- 파라미터
  - `scan_topic` (기본 `/scan`), `marker_topic` (기본 `/lidar_clusters`), `frame_id` (기본 `laser`)
  - `cluster_tolerance`: 같은 클러스터로 묶을 인접 포인트 간 거리 기준(m), 기본 `0.12`
  - `min_cluster_size` / `max_cluster_size`: 클러스터로 인정할 최소/최대 포인트 개수, 기본 `6` / `400`
  - `max_range`: 이보다 먼 포인트는 무시(m), 기본 `6.0`
  - `point_size`, `centroid_size`, `text_size`: RViz 마커 크기

RViz에서 `MarkerArray` 디스플레이를 `/lidar_clusters` 토픽으로 추가하고 Fixed Frame을 `frame_id`(기본
`laser`)와 맞추면 클러스터가 보인다.
