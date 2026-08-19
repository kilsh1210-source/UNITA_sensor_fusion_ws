# sensor_fusion_bringup

여러 서브시스템(카메라-라이다 퓨전, bird's eye view, L-shape fitting, URDF TF 등)을 한 번에 묶어서
실행하기 위한 최상위 launch 모음 패키지. 각 서브시스템은 자기 패키지 안에 자체 launch 파일(예:
`lidar_camera_fusion_pkg`의 `fusion_bringup.launch.py`, `lidar_cluster_pkg`의 `l_shape_bringup.launch.py`,
`unita_minicar_description`의 `description.launch.py`)을 그대로 가지고 있고, 이 패키지는 그것들을
조합만 한다. 새로운 서브시스템이 추가돼도 이 패키지에 새 launch 파일만 늘리면 된다.

## launch

### `full_bringup.launch.py`

`unita_minicar_description`의 `description.launch.py`(URDF 기반 `base_link`→`laser`/`camera_link` 등
고정 TF)와 `fusion_bringup.launch.py`(라이다 드라이버+카메라+YOLO(멀티모델)+`bird_eye_node`+퓨전)를
통째로 include하고, 그 위에 `lidar_cluster_pkg`의 `l_shape_node`만 추가로 띄운다. `rplidar_node`는 fusion
쪽에서 한 번만 실행되고 `l_shape_node`는 같은 `/scan`을 구독만 하므로 시리얼 포트 충돌이 없다
(`l_shape_bringup.launch.py`처럼 드라이버를 따로 또 띄우면 포트 충돌로 `/scan`을 못 받으니 주의).

URDF TF를 먼저 띄우는 이유는 `image_fusion_node`의 `use_urdf_extrinsic`(이 launch에서는 기본 `true`)이
매 프레임 `lidar_frame_id`→`camera_frame_id` TF를 조회해서 라이다→카메라 외부파라미터로 쓰기 때문이다.

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
`bird_eye_node`/`image_fusion_node`는 `fusion_bringup.launch.py`가 별도 패키지에서 소유하고 있어서,
이 YAML의 값은 그 launch 파일로 전달되는 인자의 "기본값"으로만 쓰인다 (전체 파라미터를 다 노출하려면
`fusion_bringup.launch.py` 자체를 손봐야 함).
## 후방 서라운드 BEV (카메라 2/3/4)

후방(camera_2) / 좌측(camera_3) / 우측(camera_4) 세 대의 영상을 지면에 투영해서
top-down 한 장으로 합친다. 좌/우는 비스듬히 뒤를 보고 있어 후방과 겹치는 영역이
넓은데, 겹치는 곳은 왜곡이 적은 후방 영상이 그대로 나오도록 우선순위 블렌딩을 쓴다
(`blend_mode: priority`, `rear.blend_priority`가 가장 높음).

```bash
colcon build --packages-select camera_perception_pkg sensor_fusion_bringup --symlink-install
source install/setup.bash

# 1) 카메라 3대 (이미 multi_camera_view.launch.py로 띄웠으면 건너뛴다)
ros2 launch sensor_fusion_bringup multi_camera_view.launch.py

# 2) ROI 찍기 - 아래 "ROI 맞추기" 참고
ros2 launch sensor_fusion_bringup roi_picker.launch.py

# 3) 서라운드 BEV
ros2 launch sensor_fusion_bringup rear_surround_view.launch.py
```

두 런치 모두 카메라를 직접 켜지 않는다(`start_cameras:=false`가 기본). 이미 떠 있는
카메라 노드의 토픽에 붙는 게 보통이고, 여기서 또 열면 `/dev/video*`를 이미 잡고
있어서 `cv2.VideoCapture`가 실패한다. 카메라가 안 떠 있으면 `start_cameras:=true`.

`rear_surround_view.launch.py` 인자: `show_preview`(기본 true),
`draw_camera_outlines`(카메라별 커버리지 윤곽선, 정렬 확인용), `blend_mode`.

### 캔버스 좌표계

`surround_view_node`의 캔버스는 **차를 한가운데 둔 지면 좌표계**다(800x1000).
위쪽이 차 앞, 아래쪽이 차 뒤, x가 클수록 차의 오른쪽이고 `[310,370,490,630]`이
차량 자리다.

지금은 후방만 쓰는데도 캔버스를 앞쪽까지 잡아둔 이유는, `dst_points`가 이 캔버스의
픽셀 좌표로 저장되기 때문이다. 나중에 전방 카메라를 붙이면서 캔버스를 넓히면 힘들게
찍어둔 네 점이 전부 어긋난다. 그래서 좌표계는 처음부터 최종 형태로 고정해두고, 당장
안 쓰는 앞쪽은 `view_rect: [0, 340, 800, 1000]`로 잘라서 안 내보낸다. 잘라내는 게
아니라 **애초에 그 영역만 계산**하므로 안 쓰는 영역만큼 빨라진다.

전방 카메라를 추가할 때는 `front.enabled: true` + `view_rect: [0, 0, 800, 1000]`으로
바꾸고 전방 네 점만 찍으면 된다. 후방/좌우는 다시 안 찍어도 된다.

### ROI 맞추기 (roi_picker_node)

`params.yaml`에 들어있는 `src_points`/`dst_points` 기본값은 placeholder다. 실제 장착
상태에서는 반드시 다시 찍어야 한다.

```bash
ros2 launch sensor_fusion_bringup roi_picker.launch.py
```

왼쪽 패널은 카메라 원본, 오른쪽 패널은 캔버스다. **바닥의 같은 지점을 양쪽 패널에서
같은 순서로** 클릭하면 그 자리에서 워핑 결과가 캔버스에 겹쳐 나온다. 카메라를
바꿔가며 같은 바닥 마커를 찍으면 세 영상이 캔버스 위에서 포개진다 - 이게 여러
카메라를 정렬하는 가장 확실한 방법이다.

| 조작 | 동작 |
| --- | --- |
| 좌클릭 | 점 추가 (4개 다 찍힌 뒤에는 가장 가까운 점을 그 자리로 이동) |
| 우클릭 | 마지막 점 취소 |
| `h` `j` `k` `l` | 마지막으로 건드린 점을 1픽셀씩 이동 (패널이 축소돼 있어 클릭만으로는 부족할 때) |
| `1` `2` `3` / `Tab` | 카메라 전환 (rear / left / right) |
| `r` | 현재 카메라의 점 전부 지우기 |
| `z` | `r`/`a` 되돌리기 (한 단계, 다시 누르면 재적용) |
| `a` | dst 사각형 대충 깔아주기 (끌어다 맞추는 출발점) |
| `w` | 워핑 미리보기 on/off |
| `g` | 격자 on/off |
| `f` | 캔버스 전체 ↔ 실제 출력 영역(`view_rect`) |
| `s` | `params.yaml`에 저장 |
| `q` / ESC | 종료 |

`s`는 `params.yaml`의 해당 줄만 골라 치환한다(주석 보존). 원본은
`params.yaml.bak`으로 백업된다. 네 점이 다 찍힌 카메라만 저장되고, 사각형 순서가
꼬여서 자기교차하면 하단 상태줄에 빨간 경고가 뜬다.

점 4개 순서는 "사각형을 한 바퀴 도는" 순서면 무엇이든 된다. 두 패널에서 같은
순서로만 찍으면 대응이 맞기 때문이다.

패널 크기는 `roi_picker_node.pane_height`(기본 340)가 정한다. 이 장비 화면이
1024x768이라 340으로 잡았고, 큰 모니터를 붙이면 올릴수록 점을 정확히 찍을 수 있다.

### 성능

800x660 출력에 카메라 3대 기준 약 14~19Hz(Jetson, 다른 노드와 CPU 경합 중 실측).
호모그래피/블렌딩 마스크/가중치는 전부 입력 해상도에만 의존하므로 한 번만 계산하고
캐시한다 - 매 프레임 다시 구하면 6Hz까지 떨어진다. 카메라가 하나 끊겼다 붙으면
살아있는 조합으로 가중치를 다시 만든다.
