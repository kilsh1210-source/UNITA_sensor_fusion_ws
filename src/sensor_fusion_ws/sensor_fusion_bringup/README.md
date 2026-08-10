# sensor_fusion_bringup

여러 서브시스템(카메라-라이다 퓨전, L-shape fitting, 추후 추가될 bird's eye view 등)을 한 번에 묶어서
실행하기 위한 최상위 launch 모음 패키지. 각 서브시스템은 자기 패키지 안에 자체 launch 파일(예:
`lidar_camera_fusion_pkg`의 `fusion_bringup.launch.py`, `lidar_cluster_pkg`의 `l_shape_bringup.launch.py`)을
그대로 가지고 있고, 이 패키지는 그것들을 조합만 한다. 새로운 서브시스템이 추가돼도 이 패키지에 새 launch
파일만 늘리면 된다.

## launch

### `full_bringup.launch.py`

`fusion_bringup.launch.py`(라이다 드라이버+카메라+YOLO+퓨전)를 통째로 include하고, 그 위에
`lidar_cluster_pkg`의 `l_shape_node`만 추가로 띄운다. `rplidar_node`는 fusion 쪽에서 한 번만 실행되고
`l_shape_node`는 같은 `/scan`을 구독만 하므로 시리얼 포트 충돌이 없다 (`l_shape_bringup.launch.py`처럼
드라이버를 따로 또 띄우면 포트 충돌로 `/scan`을 못 받으니 주의).

```bash
ros2 launch sensor_fusion_bringup full_bringup.launch.py
```

인자는 `fusion_bringup.launch.py`의 모든 인자(`serial_port`, `serial_baudrate`, `frame_id`, `device`,
`fx`, `cx`, `lidar_front_offset_deg`, `cam_num`, `display_mode`, `distance_tolerance`,
`draw_all_points`, `use_urdf_extrinsic`, `lidar_frame_id`, `camera_frame_id`)에 `fov_deg`(기본 `150.0`),
`launch_rviz`(기본 `true`)가 추가된 것과 동일하다. `lidar_front_offset_deg`는 퓨전의 좌표 변환과
`l_shape_node`의 `front_angle_deg`에 동시에 적용된다.

## config/params.yaml

위 launch 인자들의 기본값은 전부 `config/params.yaml` 하나에서 온다. 값을 바꾸고 싶으면(예: 카메라
재캘리브레이션으로 fx/cx가 바뀌었을 때) 이 파일만 고치면 되고, 필요하면 여전히 launch 인자로 그때그때
override할 수 있다(`ros2 launch sensor_fusion_bringup full_bringup.launch.py fov_deg:=120.0`).

`l_shape_node`는 이 launch 파일이 직접 실행하는 노드라 `config/params.yaml`을 통째로 로드한다 —
`cluster_tolerance`, `min_cluster_size`, `rect_line_width` 등 launch 인자로는 노출되지 않는 세부
파라미터까지 전부 여기서 관리된다. 반면 `rplidar_node`/`image_publisher_node`/`yolov8_node`/
`image_fusion_node`는 `fusion_bringup.launch.py`가 별도 패키지에서 소유하고 있어서, 이 YAML의 값은
그 launch 파일로 전달되는 인자의 "기본값"으로만 쓰인다 (전체 파라미터를 다 노출하려면
`fusion_bringup.launch.py` 자체를 손봐야 함).
