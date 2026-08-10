# sensor_fusion_ws

카메라(YOLOv8) + 라이다(RPLIDAR C1)를 퓨전해서, 검출된 물체까지의 거리를 측정하는 ROS2(Humble) 워크스페이스.

패키지별 상세 설명은 각 패키지의 README 참고:
[`sensor_fusion_bringup`](src/sensor_fusion_ws/sensor_fusion_bringup/README.md) ·
[`interfaces_pkg`](src/sensor_fusion_ws/interfaces_pkg/README.md) ·
[`camera_perception_pkg`](src/sensor_fusion_ws/camera_perception_pkg/README.md) ·
[`lidar_camera_fusion_pkg`](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) ·
[`lidar_cluster_pkg`](src/sensor_fusion_ws/lidar_cluster_pkg/README.md) ·
[`rplidar_ros`](src/sensor_fusion_ws/rplidar_ros/README.md)

## 1. 카메라·라이다 연결 및 포트 확인

### 1-1. 연결 전

USB 포트에 아직 아무것도 꽂지 않은 상태에서 기준선을 확인해둔다.

```bash
ls /dev/video* 2>/dev/null    # 현재 잡혀있는 비디오 장치 (다른 카메라가 있으면 미리 보임)
ls /dev/ttyUSB* 2>/dev/null   # 현재 잡혀있는 USB 시리얼 장치
```

### 1-2. 웹캠 연결 후 장치 번호 확인

```bash
ls /dev/video*
```

카메라 1대에 `/dev/video0`, `/dev/video1`처럼 번호가 2개 잡히는 경우가 흔한데(영상용 노드 + 메타데이터 노드),
보통 더 작은 번호가 실제 영상 장치다. 어떤 번호가 맞는지 확실히 하려면 실제로 프레임을 읽어본다:

```bash
python3 -c "
import cv2
for i in range(4):
    cap = cv2.VideoCapture(i)
    ok, _ = cap.read()
    print(i, 'OK' if ok else 'fail')
    cap.release()
"
```

`OK`가 뜨는 가장 작은 번호를 카메라 번호로 쓰면 된다.

### 1-3. 라이다(RPLIDAR C1) 연결 후 포트 확인

```bash
ls /dev/ttyUSB*
```

연결 전엔 없다가 연결 직후 새로 나타난 번호(`/dev/ttyUSB0` 등)가 라이다다. 여러 개의 USB-시리얼 장치가
동시에 꽂혀있어서 헷갈리면:

```bash
dmesg | tail -20   # 방금 연결한 장치의 커널 로그 (ttyUSB 번호가 어떤 장치인지 보임)
```

권한도 같이 확인 (`dialout` 그룹이 없으면 포트를 열 수 없음):

```bash
groups   # 목록에 dialout 이 있어야 함
# 없으면: sudo usermod -aG dialout $USER   (실행 후 재로그인 필요)
```

### 1-4. 확인한 번호를 실행 시 반영

기본값과 다르면, **소스를 고칠 필요 없이** launch 인자로 그때그때 넘기거나
[`config/params.yaml`](src/sensor_fusion_ws/sensor_fusion_bringup/config/params.yaml)의 값을 고쳐서 영구적으로 반영할 수 있다
(단, `config/params.yaml`을 고친 뒤에는 `colcon build --packages-select sensor_fusion_bringup`을 다시 해야 `install/`에 반영됨 —
symlink 설치가 아니라 파일을 복사하는 방식이라서다).

```bash
ros2 launch sensor_fusion_bringup full_bringup.launch.py \
  cam_num:=1 \
  serial_port:=/dev/ttyUSB0
```

ROS2 Humble이 설치되어 있어야 한다 (`/opt/ros/humble`).

## 2. 빌드

```bash
cd ~/sensor_fusion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

매번 새 터미널을 열 때마다 `/opt/ros/humble/setup.bash`와 `install/setup.bash` 두 개를 순서대로 source 해야 한다.

### 2-1. CUDA 확인 (GPU 사용 시)

`yolov8_node`/`bird_eye_node`의 추론 디바이스 기본값은 `cuda:0`다 (Jetson Orin Nano 등 GPU 대상).
실행 전에 CUDA를 실제로 쓸 수 있는지 먼저 확인한다.

```bash
python3 -c "import torch; print('cuda available:', torch.cuda.is_available())"
```

`True`가 아니면:

- Jetson은 일반 `pip install torch`(x86/CUDA 데스크톱용)로는 GPU를 못 잡는다. JetPack 버전에 맞는
  NVIDIA 전용 torch/torchvision wheel(또는 그걸 반영한 `ultralytics`)을 설치해야 한다.
- GPU가 아예 없는 환경(개발 PC 등)에서는 `device:=cpu`를 launch 인자로 넘겨서 CPU로 돌리면 된다:
  ```bash
  ros2 launch sensor_fusion_bringup full_bringup.launch.py device:=cpu
  ```

## 3. 전체 파이프라인 실행

라이다 + 카메라 + YOLO + 퓨전(거리 측정) + L-shape fitting까지 한 번에 띄운다 (권장 진입점).
값들은 전부 [`config/params.yaml`](src/sensor_fusion_ws/sensor_fusion_bringup/config/params.yaml)
하나로 관리되고, 필요하면 launch 인자로 그때그때 override할 수 있다.

```bash
ros2 launch sensor_fusion_bringup full_bringup.launch.py
```

라이다·카메라·YOLO·퓨전만 필요하면 (L-shape fitting/rviz 없이) 아래처럼 한 단계 아래 launch 파일을
직접 실행해도 된다. 단 이 경우 기본값은 `config/params.yaml`이 아니라 그 파일 자체에 하드코딩된 값이다.

```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py
```

### 화면 (Fusion Visualizer)

- 정상 동작하면 `Fusion Visualizer`라는 이름의 OpenCV 창 1개가 뜬다. 창을 한 번 클릭해서
  포커스를 준 뒤 키보드로 화면 모드를 전환할 수 있다.

| 키 | 동작 |
|---|---|
| `1` | 주화면: 기본 카메라 이미지 (오버레이 없음) |
| `2` | 주화면: 라이다 포인트만 표시 |
| `3` | 주화면: YOLO 바운딩박스 + 거리 표시 (기본 시작 모드) |
| `4` | 주화면: 버드아이뷰 (`bird_eye_node`의 차선 검출 결과) |
| `5` | 주화면: 버드아이뷰 ROI 원본 (버드아이뷰 변환에 쓰는 사다리꼴을 원본 위에 표시) |
| `q`/`w`/`e`/`r`/`t` | 보조화면을 각각 raw/lidar/boxes/bev/bev_roi로 선택 |
| `v` | 분할보기 토글 — 켜면 주화면+보조화면을 가로로 나란히 표시 |

- YOLO는 cone/drum 탐지 모델(`best_cone.pt`)과 차량 후면 탐지 모델(`car_back.pt`) 두 개를 동시에
  돌려서 결과를 하나로 합쳐 발행한다 (`camera_perception_pkg/models/`, `yolov8_node`의 `model` 파라미터에
  콤마로 구분해서 넘기면 여러 모델을 함께 로딩함).

### launch 인자 (필요할 때만 덮어쓰기)

`full_bringup.launch.py`의 인자는 `fusion_bringup.launch.py`의 모든 인자에 `fov_deg`, `launch_rviz`가
추가된 것과 같다 (자세한 표는 [`sensor_fusion_bringup` README](src/sensor_fusion_ws/sensor_fusion_bringup/README.md) 참고).
자주 바꾸는 것 위주로 추리면:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `cam_num` | `config/params.yaml` 참고 | 카메라 장치 번호, 1번에서 확인한 값 |
| `serial_port` | `/dev/ttyUSB0` | 라이다가 다른 포트로 잡히면 변경 |
| `serial_baudrate` | `460800` | RPLIDAR C1 기준값 |
| `frame_id` | `laser` | 라이다 스캔 좌표계 이름 |
| `device` | `cuda:0` | YOLO/버드아이뷰 추론 디바이스 (`cuda:0` / `cpu`, GPU 없으면 `cpu`로 override) |
| `fx`, `cx` | `565.529459`, `337.983746` | 카메라 초점거리/광학중심(px), 캘리브레이션 결과값 |
| `lidar_front_offset_deg` | `-180.0` | 라이다 0도 방향과 카메라 정면 방향의 차이(도) |
| `display_mode` | `boxes` | Fusion Visualizer 시작 화면 모드 (`raw`/`lidar`/`boxes`/`bev`/`bev_roi`, 실행 중엔 키로 전환) |

예:
```bash
ros2 launch sensor_fusion_bringup full_bringup.launch.py serial_port:=/dev/ttyUSB1 device:=cuda:0
```

## 4. 정상 동작 확인 (다른 터미널에서)

```bash
source /opt/ros/humble/setup.bash
source ~/sensor_fusion_ws/install/setup.bash

ros2 topic list
# /image_raw, /scan, /detections 가 보여야 함

ros2 topic hz /image_raw     # ~30Hz 근처로 나오면 카메라 정상
ros2 topic hz /scan          # 라이다가 정상 발행 중이면 수 Hz~10Hz대로 나옴
ros2 topic hz /detections    # YOLO가 프레임마다 검출 결과를 발행 중인지 확인 (물체가 없어도 빈 배열은 발행됨)

ros2 topic echo /detections --once   # 검출 결과 1개 내용 확인 (class_name, score, bbox 등)
```

각 토픽이 하나라도 안 뜨면 3번 launch를 실행한 터미널의 로그에서 해당 노드가 에러를 내고 있는지 확인.

## 5. 문제가 생겼을 때 개별 노드로 나눠서 확인

한 번에 다 띄우지 않고 노드를 하나씩 켜보면 어느 단계에서 막히는지 좁힐 수 있다.

```bash
# 라이다만
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800 -p frame_id:=laser

# 카메라만
ros2 run camera_perception_pkg image_publisher_node

# YOLO만 (카메라가 먼저 떠 있어야 함, 모델 경로는 설치 경로 기준)
# model 파라미터에 콤마로 여러 개를 넘기면 각 모델을 모두 돌려서 결과를 합쳐 발행한다
MODELS_DIR=$(ros2 pkg prefix camera_perception_pkg)/share/camera_perception_pkg/models
ros2 run camera_perception_pkg yolov8_node --ros-args \
  -p model:="$MODELS_DIR/best_cone.pt,$MODELS_DIR/car_back.pt" \
  -p device:=cuda:0

# 퓨전만 (카메라·라이다·YOLO가 먼저 떠 있어야 함) — 실제 launch 파일이 쓰는 노드
ros2 run lidar_camera_fusion_pkg image_fusion_node --ros-args \
  -p fx:=565.529459 -p cx:=337.983746 -p front_angle_deg:=-180.0
```

> `lidar_camera_fusion_pkg`에는 `sensor_fusion_node`(동축 마운트 가정, 픽셀 각도 기반)라는 더 단순한
> 대안 노드도 있지만, 현재 launch 파일들은 `image_fusion_node`(3D 투영 기반)를 사용한다.
> 자세한 차이는 [`lidar_camera_fusion_pkg` README](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) 참고.

### 자주 나는 에러

- `can't open camera by index` — 다른 프로세스가 이미 해당 `/dev/video*`를 잡고 있거나(`fuser /dev/video0`로 확인 후
  `kill -9 <pid>`), 1-2번에서 확인한 것과 다른 `cam_num`을 쓰고 있는 경우.
- `serial_port` 관련 permission denied — `groups`에 `dialout` 없으면
  `sudo usermod -aG dialout $USER` 후 재로그인.
- YOLO가 뜨긴 하는데 거리 텍스트가 항상 `N/A` — `/scan` 토픽이 안 들어오고 있거나
  (라이다 미연결/포트 오류), `lidar_front_offset_deg`가 실제 마운트 방향과 안 맞는 경우.

## 6. 캘리브레이션이 바뀌면

- 카메라를 바꾸거나 재캘리브레이션하면 `fx`, `cx`(필요하면 `image_fusion_node`의 `fy`, `cy`도)를
  새 값으로 갱신할 것. `config/params.yaml`을 고치고 `colcon build --packages-select sensor_fusion_bringup`.
- 라이다/카메라 장착 방향을 바꾸면 `lidar_front_offset_deg` 재확인 (자세한 내용은
  [`lidar_camera_fusion_pkg` README](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) 참고)
- 새 YOLO 모델(`.pt`)을 받으면 `camera_perception_pkg/models/`에 넣고, `yolov8_node`의 `model`
  파라미터(콤마로 여러 개 지정 가능)를 그 파일명으로 맞춘 뒤 `camera_perception_pkg`를 재빌드할 것.
