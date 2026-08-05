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

### `l_shape_node`

`scan_cluster_node`와 동일한 방식으로 클러스터를 만든 뒤, 각 클러스터에 사각형(L-shape)을 피팅해서
방향(heading)과 폭/길이를 추정하는 노드. 0~90도 범위를 회전시켜가며 포인트들이 사각형의 두 변에 가장
가깝게 붙는 각도를 찾는 search-based rectangle fitting(closeness criterion) 방식을 사용한다.

- 구독: `/scan`
- 발행: `/l_shape_boxes` (`visualization_msgs/MarkerArray`) — 클러스터별 사각형 외곽선(LINE_STRIP),
  긴 변 방향을 가리키는 heading 화살표, 크기/각도 텍스트 라벨
- 파라미터
  - `scan_topic`, `marker_topic`(기본 `/l_shape_boxes`), `frame_id`: `scan_cluster_node`와 동일
  - `cluster_tolerance`, `min_cluster_size`, `max_cluster_size`, `max_range`: `scan_cluster_node`와 동일한
    기본값의 클러스터링 파라미터
  - `front_angle_deg`(기본 `180.0`) / `fov_deg`(기본 `150.0`): 라이다가 물리적으로 180도 돌려 장착된 경우
    등을 보정하기 위해, 라이다 자체 좌표계에서 `front_angle_deg` 방향을 "실제 전방"으로 보고 그 좌우
    `fov_deg/2`씩(기본 총 150도)만 클러스터링 대상으로 남긴다. `image_fusion_node`의
    `lidar_front_offset_deg`와 동일한 관례
  - `theta_resolution_deg`: 각도 탐색 간격(도), 기본 `1.0` — 작을수록 정밀하지만 계산량 증가
  - `min_points_for_fit`: 사각형 피팅에 필요한 최소 포인트 개수, 기본 `4`
  - `rect_line_width`, `show_heading_arrow`, `arrow_length`, `show_text`, `text_size`: RViz 시각화 옵션
  - `launch_rviz`(기본 `true`): 노드 시작 시 `rviz/L_shape_config.rviz` 설정으로 rviz2를 자동 실행할지 여부
  - `rviz_config`: 자동 실행할 rviz 설정 파일 경로(기본값은 패키지에 포함된 `rviz/L_shape_config.rviz`)

`launch_rviz`가 `true`(기본값)이면 노드 실행 시 `rviz/L_shape_config.rviz` 설정으로 rviz2가 자동으로
열리며, `/l_shape_boxes`(MarkerArray)와 `/scan`(LaserScan) 디스플레이가 이미 세팅되어 있다. 노드를
끄면(Ctrl+C) 같이 뜬 rviz2도 함께 종료된다. 자동 실행을 원치 않으면
`ros2 run lidar_cluster_pkg l_shape_node --ros-args -p launch_rviz:=false`로 끌 수 있다.

## launch

### `l_shape_bringup.launch.py`

rplidar 드라이버(`rplidar_node`)와 `l_shape_node`를 한번에 실행하는 launch 파일. `l_shape_node`가
`launch_rviz`(기본 `true`)로 rviz2까지 자동으로 띄우므로, 이거 하나만 실행하면 드라이버+클러스터링+시각화가
전부 뜬다.

```bash
ros2 launch lidar_cluster_pkg l_shape_bringup.launch.py
```

인자(필요할 때만 덮어쓰기): `serial_port`(기본 `/dev/ttyUSB0`), `serial_baudrate`(기본 `460800`, C1 기준),
`frame_id`(기본 `laser`), `front_angle_deg`(기본 `180.0`), `fov_deg`(기본 `150.0`),
`launch_rviz`(기본 `true`).
