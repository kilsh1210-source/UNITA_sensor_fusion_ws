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
`fx`, `cx`, `lidar_front_offset_deg`, `cam_num`, `show_split_view`, `distance_tolerance`,
`draw_all_points`, `use_urdf_extrinsic`, `lidar_frame_id`, `camera_frame_id`)에 `fov_deg`(기본 `150.0`),
`launch_rviz`(기본 `true`)가 추가된 것과 동일하다. `lidar_front_offset_deg`는 퓨전의 좌표 변환과
`l_shape_node`의 `front_angle_deg`에 동시에 적용된다.
