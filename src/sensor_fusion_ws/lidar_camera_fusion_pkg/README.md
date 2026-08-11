# lidar_camera_fusion_pkg

카메라(YOLO 검출 + `bird_eye_node`의 버드아이뷰)와 라이다(`/scan`)를 결합해서, 검출된 객체까지의 거리를
계산하고 하나의 "Fusion Visualizer" 창으로 보여주는 패키지. 방식이 다른 두 개의 퓨전 노드가 들어있고,
워크스페이스 전체를 한 번에 띄우는 launch 파일도 여기 있다.

## 노드

### `image_fusion_node` (권장 — 현재 launch 파일들이 사용하는 방식)

카메라 내부파라미터(fx, fy, cx, cy)와 라이다→카메라 외부파라미터(회전+변환)를 이용해 라이다 포인트를
이미지 평면에 3D 투영하는 방식. 외부파라미터는 `unita_minicar_description`의 URDF/TF에서 가져오거나
(`use_urdf_extrinsic:=true`, 기본값), 직접 지정한 `cam_x_offset`/`cam_height` + `front_angle_deg`로
계산할 수도 있다.

- 구독: `image_raw`(`sensor_msgs/Image`), `/scan`(`sensor_msgs/LaserScan`), `/detections`(`interfaces_pkg/DetectionArray`),
  `bird_eye/image`, `bird_eye/roi`(둘 다 `bird_eye_node`가 발행)
- `cv2.imshow`로 **"Fusion Visualizer"** 창 1개를 띄우고, 창을 클릭해 포커스를 준 뒤 키보드로 화면을 전환한다.

  | 키 | 동작 |
  |---|---|
  | `1` | 주화면: 기본 카메라 이미지 (오버레이 없음) |
  | `2` | 주화면: 라이다 포인트만 표시 |
  | `3` | 주화면: YOLO 바운딩박스 + 거리 표시 (기본 시작 모드) |
  | `4` | 주화면: 버드아이뷰 (`bird_eye_node`의 차선 검출 결과) |
  | `5` | 주화면: 버드아이뷰 ROI 원본 (버드아이뷰 변환에 쓰는 사다리꼴을 원본 위에 표시) |
  | `q`/`w`/`e`/`r`/`t` | 보조화면을 각각 raw/lidar/boxes/bev/bev_roi로 선택 |
  | `v` | 분할보기 토글 — 켜면 주화면+보조화면을 가로로 나란히 표시 |

- 주요 파라미터 (전체 목록은 `declare_parameter` 참고)
  - `fx`, `fy`, `cx`, `cy`: 카메라 내부파라미터(px). 기본값(`565.529459`, `566.767111`, `337.983746`,
    `290.095566`)은 실측 캘리브레이션 결과. 렌즈 왜곡(distortion) 보정은 적용하지 않으므로 이미지
    가장자리 쪽 투영은 오차가 있을 수 있다.
  - `use_urdf_extrinsic`(기본 `true`): `true`면 매 프레임 `lidar_frame_id`→`camera_frame_id` TF를 조회해서
    외부파라미터로 사용(`unita_minicar_description`의 `description.launch.py`가 이 TF를 발행해야 함).
    `false`면 `cam_x_offset`/`cam_height`/`front_angle_deg`로 직접 계산한 고정값을 사용.
  - `lidar_frame_id`(기본 `laser`), `camera_frame_id`(기본 `camera_link`, launch에서는
    `camera_optical_frame_tilted`로 override) — TF 조회에 쓰는 프레임 이름.
  - `front_angle_deg`(노드 자체 기본 `180.0`, launch 기본 `-180.0`): 라이다가 인식하는 "정면"과 실제
    카메라가 보는 방향이 다를 때 보정값(도). 라이다·FOV 필터·`use_urdf_extrinsic:=false`일 때의 좌표
    변환에 모두 쓰인다.
  - `cam_fov_deg`(기본 `55.0`): FOV 필터에 쓰는 카메라 수평 화각(도).
  - `display_mode`(기본 `boxes`): 시작 화면 모드(`raw`/`lidar`/`boxes`/`bev`/`bev_roi`), 실행 중엔 키로 전환.
  - `distance_method`(기본 `center`) / `distance_tolerance`(기본 `0.6`): bbox 안 라이다 포인트 중 거리를
    추정하는 방식과, 가장 가까운 포인트 기준으로 함께 묶어 평균 낼 오차 허용 범위(m).
  - `draw_all_points`(기본 `true`): 카메라 이미지 위에 라이다 포인트를 전부 그릴지 여부.

### `sensor_fusion_node` (대안 — 라이다·카메라가 동축 마운트일 때)

라이다와 카메라가 **z(높이)만 다르고 x, y는 동일한 동축(coaxial) 위치**에 마운트된다는 가정 하에,
별도의 3D 외부파라미터(extrinsic) 계산 없이 카메라 픽셀 각도만으로 라이다 거리를 찾는 단순한 방식.
현재 마운트는 동축이 아니므로(카메라가 라이다 앞쪽에 오프셋되어 있음) 어느 launch 파일도 이 노드를
쓰지 않지만, 배선을 단순화하고 싶을 때 참고용으로 남아있다.

동작 순서: bbox 중심 픽셀의 x좌표 → 카메라 수평 화각(`camera_fov`) 기준 각도로 변환 →
그 각도(+ `lidar_front_offset_deg` 보정)에 해당하는 라이다 스캔 인덱스의 거리값을 읽음.

- 구독: `image_raw` (`sensor_msgs/Image`), `/scan` (`sensor_msgs/LaserScan`), `/detections` (`interfaces_pkg/DetectionArray`)
- `cv2.imshow`로 "Sensor Fusion" 창을 띄워 bbox + 거리 텍스트를 오버레이. 화면을 클릭하면 그 지점의 거리도 표시.
- **ROS 파라미터 (재빌드 없이 launch 인자로 조정 가능)**
  - `camera_fov`: 카메라 수평 화각(도). 현재 값 `59.5399`는 실측 캘리브레이션 fx=559.431712, width=640 기준
    (`2*atan(width/(2*fx))`)으로 산출. 카메라를 바꾸면 재캘리브레이션 후 갱신할 것.
  - `lidar_front_offset_deg`: 라이다가 인식하는 "정면"과 실제 카메라가 보는 방향이 다를 때 보정값(도).
    현재 `180.0` (라이다 0도 방향이 카메라 반대쪽을 보는 마운트 기준).

## launch/fusion_bringup.launch.py

라이다 + 카메라 + YOLO(cone/drum + 차량 후면 모델) + `bird_eye_node`(버드아이뷰) + `image_fusion_node`를
한 번에 띄우는 통합 launch 파일.

```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py
```

인자: `serial_port`(기본 `/dev/ttyUSB0`), `serial_baudrate`(기본 `460800`, RPLIDAR C1 기준),
`frame_id`(기본 `laser`), `device`(YOLO/버드아이뷰 추론 디바이스, 기본 `cuda:0`), `fx`(기본 `565.529459`),
`cx`(기본 `337.983746`), `lidar_front_offset_deg`(기본 `-180.0`), `cam_num`(기본 `0`),
`display_mode`(기본 `boxes`), `draw_all_points`(기본 `true`), `distance_tolerance`(기본 `0.6`),
`use_urdf_extrinsic`(기본 `false` — `sensor_fusion_bringup`을 거치면 `config/params.yaml`의 `true`로 override됨),
`lidar_frame_id`(기본 `laser`), `camera_frame_id`(기본 `camera_optical_frame_tilted`).

YOLO는 `camera_perception_pkg/models/`의 `best_cone.pt`(콘/드럼) + `car_back.pt`(차량 후면)를 콤마로 묶어
`model` 파라미터로 넘겨서 동시에 로딩한다. `bird_eye_node`는 자체 미리보기 창(`show_preview`)을 끄고
`bird_eye/image`·`bird_eye/roi`만 발행해서 Fusion Visualizer의 4번/5번 화면으로 넘긴다.

예: 나중에 라이다 오프셋을 다시 재보정한 경우
```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py lidar_front_offset_deg:=-180.0 fx:=565.529459
```

> 이 launch 파일은 URDF TF(`unita_minicar_description`)를 자체적으로 include하지 않는다.
> `use_urdf_extrinsic:=true`로 쓰려면(=`sensor_fusion_bringup`의 `full_bringup.launch.py`처럼)
> TF를 별도로 띄워줘야 한다.
