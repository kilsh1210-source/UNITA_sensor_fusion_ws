# unita_minicar_master

UNITA 미니카 자율주행 전체 코드. **ROS2(Humble) 워크스페이스 + 아두이노 펌웨어**를 함께 담는다.

카메라(YOLOv8)로 차선과 장애물을 인식하고, 라이다(RPLIDAR C1)를 퓨전해 거리를 재고,
lattice 경로계획 + pure pursuit으로 조향을 만들어 아두이노로 내보낸다.

```
카메라 ─ yolov8_node ─┬─ lane_info_extractor_node ─┐
                      │   (차선 마스크 → 목표점)     │
                      └─ image_fusion_node ────────┤   (장애물 거리/픽셀x, 검출 전부 배열로도)
라이다 ───────────────────────────────────────────┤
                                                   ▼
                          path_planner_node (lattice 경로)
                                                   ▼
                    motion_planner_node (pure pursuit + PD)
                                                   ▼
                       serial_sender_node ─ "C,<조향>,<후륜PWM>" ─ 아두이노
                                                   ▼
                        firmware/autonomous_mega (조향 폐루프 + 구동)
```

| 디렉토리 | 내용 |
|---|---|
| `src/sensor_fusion_ws/` | ROS2 패키지 전체 (인식·퓨전·판단·제어) |
| `firmware/autonomous_mega/` | 아두이노 Mega 펌웨어 (PlatformIO). 조향 폐루프·구동·초음파 |
| `firmware/tools/` | 조향 캘리브레이션 측정 스크립트 |

**하드웨어 캘리브레이션 값과 그 근거는 [9번](#9-조향-캘리브레이션-실측-기준)에 정리돼 있다.**
조향이 이상하게 동작하면 그것부터 볼 것.

---

패키지별 상세 설명은 각 패키지의 README 참고:
[`sensor_fusion_bringup`](src/sensor_fusion_ws/sensor_fusion_bringup/README.md) ·
[`interfaces_pkg`](src/sensor_fusion_ws/interfaces_pkg/README.md) ·
[`camera_perception_pkg`](src/sensor_fusion_ws/camera_perception_pkg/README.md) ·
[`lidar_camera_fusion_pkg`](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) ·
[`lidar_cluster_pkg`](src/sensor_fusion_ws/lidar_cluster_pkg/README.md) ·
[`rplidar_ros`](src/sensor_fusion_ws/rplidar_ros/README.md)

자율주행(판단·제어) 패키지는 [minicar_sim](https://github.com/gwakminji/minicar_sim)에서 가져왔다:
`decision_making_pkg`(lattice 경로계획 + pure pursuit 조향), `serial_communication_pkg`(아두이노 시리얼 송신),
`camera_perception_pkg/lane_info_extractor_node`(차선 마스크 → 주행 목표점). 실행은 [3-1](#3-1-자율주행-실행-drivelaunchpy) 참고.

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
cd ~/UNITA_sensor_fusion_ws
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

- YOLO는 cone/drum 탐지 모델(`best_cone.pt`), 차량 후면 탐지 모델(`car_back.pt`),
  차선 세그멘테이션 모델(`lane_seg.pt`, 클래스 `lane_1`/`lane_2`) 세 개를 동시에 돌려서
  결과를 하나로 합쳐 발행한다 (`camera_perception_pkg/models/`, `yolov8_node`의 `model` 파라미터에
  콤마로 구분해서 넘기면 여러 모델을 함께 로딩함).
- 차선 마스크는 박스가 화면을 거의 다 덮기 때문에, Fusion Visualizer의 박스 표시와
  `/lidar_obstacle_info`(장애물 거리) 계산에서는 제외된다
  (`image_fusion_node`의 `box_class_exclude` / `obstacle_class_exclude` 파라미터).

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

## 3-1. 자율주행 실행 (drive.launch.py)

위의 인지(카메라·라이다·YOLO·퓨전)에 **판단·제어**를 붙여서 실제로 차를 굴리는 진입점.
판단/제어 코드는 [minicar_sim](https://github.com/gwakminji/minicar_sim)에서 가져와 이 워크스페이스에
맞춘 것이고, 마지막 시리얼 송신 노드는 실차용 `serial_communication_pkg`다.

```
카메라 → yolov8_node (cone / car_back / lane_seg) → /detections
   ├→ lane_info_extractor_node → /yolov8_lane_info      (BEV로 편 차선의 중심점들)
   └→ image_fusion_node        → /lidar_obstacle_info   (가장 가까운 장애물 거리[m] + 화면상 x[px])
                                → /lidar_obstacle_array  (검출된 장애물 전부: 거리/중심x/반폭)
        → path_planner_node (lattice)      → /path_planning_result
        → motion_planner_node (pure pursuit + PD) → /topic_control_signal  (MotionCommand)
        → serial_sender_node → 아두이노 시리얼 "C,<조향 -1.0~1.0>,<후륜 PWM>"
```

```bash
# 바퀴를 굴리지 않고 명령만 확인 (처음엔 반드시 이걸로 먼저)
ros2 launch sensor_fusion_bringup drive.launch.py enable_serial:=false

# 실제 주행 (차를 들어올리거나 넓은 곳에서, 전원 차단 준비하고)
ros2 launch sensor_fusion_bringup drive.launch.py cam_num:=2 serial_port:=/dev/ttyUSB1
```

**재launch하기 전에는 항상 `./stop_drive.sh`부터 실행할 것.** Ctrl+C 후 셸 프롬프트가 돌아오기
전에(=이전 launch 프로세스 트리가 아직 안 죽었는데) 재launch하면, `rplidar_node`가
`/dev/ttyUSB0`를 못 열어 그 자리에서 죽는다(SDK 에러 `0x80008004`, 라이다 자체 고장 아님 —
포트 경합). `stop_drive.sh`는 이전 launch를 정상 종료(최대 15초 대기, 안 죽으면 강제종료)까지
확인하고 라이다 포트가 비었는지까지 찍어준다.

```bash
./stop_drive.sh && ros2 launch sensor_fusion_bringup drive.launch.py ...
```

`full_bringup.launch.py`의 인자(`cam_num`, `serial_port`, `device` …)는 여기서도 그대로 먹는다.
추가 인자는 아래 두 개다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `enable_serial` | `true` | 아두이노로 실제 명령을 보낼지 여부. `false`면 `/topic_control_signal`까지만 돌아서 바퀴가 안 움직인다 |
| `decision_start_delay` | `5.0` | 센서·YOLO가 뜬 뒤 판단 노드를 올리기까지의 지연[s] |

주행 파라미터(차선 폭, 회피 거리, lattice 가중치, 조향/속도 이득, 시리얼 포트 등)는 전부
[`config/params.yaml`](src/sensor_fusion_ws/sensor_fusion_bringup/config/params.yaml)의
`lane_info_extractor_node` / `path_planner_node` / `motion_planner_node` / `serial_sender_node`
항목에 모여 있다. 고친 뒤에는 `colcon build --packages-select sensor_fusion_bringup`을 다시 해야
`install/`에 반영된다. 단 `--symlink-install`로 빌드한 워크스페이스라면 `params.yaml`과 파이썬
노드가 모두 `src/`로 심링크돼 있어 **재빌드 없이 launch만 다시 하면** 된다
(확인: `ls -l install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/params.yaml`).

조향 이득·스무딩 파라미터와 튜닝 순서는 11번에 따로 정리돼 있다.

### 확인 순서

```bash
ros2 topic hz /yolov8_lane_info      # 차선이 보이면 카메라 프레임레이트 근처로 나와야 함
ros2 topic hz /path_planning_result  # 유효 차선점이 3개 이상일 때만 발행됨
ros2 topic echo /topic_control_signal  # steering(-9~9), left_speed/right_speed
```

- `/yolov8_lane_info`가 안 나오면 → 차선이 YOLO에 안 잡히는 것. `lane_info_extractor_node`가 띄우는
  `Lane Info (ROI)` 창에서 흰 선이 보이는지 확인하고, 안 보이면 `params.yaml`의 `src_points`(BEV 사다리꼴)를
  이 카메라 장착 각도에 맞게 다시 잡아야 한다.
- `steering` 부호가 반대로 먹으면 `serial_sender_node`의 `steer_invert: true`.
- 처음 굴릴 때는 `motion_planner_node`의 `base_speed`를 낮춰서 시작할 것(현재 33).
  저속 튜닝 모드에서는 `base_speed`/`min_speed`/`max_speed`를 **같은 값으로 묶어야** 한다.
  `base_speed`만 올리면 `clamp(base − gain×|steer|, min, max)`의 상한에 걸려 그대로 `max_speed`가 나간다.
  `max_steer_cmd`는 `motion_planner_node`와 `serial_sender_node` 양쪽이 같은 값이어야 한다.

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

## 6. 라이다 점이 실제와 틀어져 보일 때 — 정렬 캘리브레이션 도구

라이다 점이 물체보다 살짝 위/아래/옆으로 밀려 보이는 가장 흔한 원인은 **카메라 다운틸트 각도**다.
이 각도는 자로 정확히 재는 게 사실상 불가능한데, **1도만 틀려도 화면에서 약 10 px가 밀린다**
(`fy≈567` 기준). 그래서 각도를 재는 대신 **화면을 보면서 키로 맞추고 한 키로 저장**하는
전용 도구를 쓴다.

```bash
ros2 launch lidar_camera_fusion_pkg calibration.launch.py
```

YOLO/버드아이뷰 없이 라이다 + 카메라 + 정렬 화면만 띄운다 (GPU를 안 쓰므로 가볍다).

### 맞추는 순서

1. **콘(또는 상자·벽)을 차 앞 1~3 m 정면에 둔다.** 화면 중앙선(`g`키로 켜고 끔)에 맞춰 놓으면
   좌우(yaw)를 판단하기 쉽다.
2. 창을 한 번 클릭해 포커스를 준다. 라이다 점이 색으로 표시되고, **가장 가까운 점은 흰 원과
   거리 값**으로 강조된다 — 보통 그게 맞추려는 콘이다.
3. `space`로 화면을 정지시키면 천천히 맞출 수 있다 (정지 중에도 보정은 실시간 반영됨).
4. 아래 키로 **라이다 점이 실제 콘 위에 오도록** 움직인다.
5. **`s` 키를 누르면 현재 값이 `config/params.yaml`에 바로 저장된다.** (원본은 `.bak`으로 백업)
6. 창을 닫고 아래를 실행해 주행 파이프라인에 반영한다:
   ```bash
   colcon build --packages-select sensor_fusion_bringup
   ```

### 키

| 키 | 동작 |
|---|---|
| `i` / `k` | pitch — 점을 **위 / 아래**로 |
| `j` / `l` | yaw — 점을 **왼쪽 / 오른쪽**으로 |
| `u` / `o` | roll — 점을 반시계 / 시계로 |
| `n` / `m` | height — 점을 위 / 아래로 (가까운 점일수록 크게 움직임) |
| `[` / `]` | 조정 스텝 절반 / 두 배 (기본 0.25도) |
| `0` | 보정값 전부 0으로 |
| `z` | 마지막 조정 취소 (undo) |
| `space` | 화면 정지 / 해제 |
| `g` | 조준용 중심선·격자 표시 토글 |
| **`s`** | **저장 — `params.yaml`에 기록** |
| `r` | 저장된 값 다시 불러오기 |
| `q` / `ESC` | 종료 |

> 팁: 회전(pitch/yaw)은 **거리와 상관없이 항상 같은 픽셀만큼** 점을 움직이고, height는
> 가까운 점일수록 크게 움직인다. 그러니 가까운 콘과 먼 콘이 **둘 다 같은 방향으로** 밀려 있으면
> pitch/yaw를, **가까운 것만** 많이 밀려 있으면 height를 건드리면 된다.

주행 중에 쓰는 `Fusion Visualizer`(3번 화면) 창에서도 같은 키(`i/k/j/l/u/o/n/m`)로 즉석 보정이
가능하고, `p`키를 누르면 `params.yaml`에 붙여넣을 값이 로그로 출력된다. 다만 저장(`s`)은
위 캘리브레이션 도구에만 있다.

### 그래도 안 맞으면

- **차가 움직일 때만** 어긋난다면 정렬이 아니라 **시간 지연** 문제다. 화면 하단 HUD의
  `scan age`를 보고, 150 ms를 넘으면 라이다/카메라 발행이 밀리고 있는 것이다.
- 화면 전체가 **좌우로 늘어난 느낌**이면 카메라가 요청 해상도(640x480)를 안 주고 있을 수 있다.
  `image_publisher_node` 로그에 해상도 경고가 뜨는지 확인할 것 (캘리브레이션을 뽑은 해상도와
  실제 해상도가 다르면 `fx/fy/cx/cy`가 통째로 안 맞는다).
- TF가 안 뜨면 `image_fusion_node`가 폴백 extrinsic으로 투영하며 경고를 남긴다. 그 경고가
  보이면 `description.launch.py`(robot_state_publisher + static TF)가 떠 있는지 확인할 것.

## 7. YOLO가 콘 대신 멀리 있는 엉뚱한 걸 잡을 때

**손바닥으로 렌즈를 가렸다 떼면 콘을 잘 잡는데 가만히 두면 이상한 걸 잡는다** — 이건 거의 항상
웹캠의 **오토포커스(AF) / 자동노출(AE)** 때문이다. 가만히 두면 AF가 초점을 찾아 헤매다 먼 배경에
락이 걸리고, 그러면 가까운 콘은 흐려져서 놓치고 선명해진 먼 배경에서 오탐이 난다. 손으로 가렸다
떼는 순간 AF/AE가 재수렴하면서 가까운 콘에 초점이 잡히는 것.

`config/params.yaml`의 `image_publisher_node`에서 고정한다 (기본값은 이미 AF/AE 끔):

```yaml
image_publisher_node:
  ros__parameters:
    autofocus: false
    focus: -1          # 0~255. 콘이 선명해지는 값으로 고정 (아래 방법으로 찾음)
    auto_exposure: false
    exposure: -1       # 화면이 너무 어두우면 올릴 것
```

**초점값 찾는 법** — 콘을 주행 시 보이는 거리에 두고, 카메라를 연결한 채 터미널에서:

```bash
# 이 카메라가 지원하는 항목과 범위 확인 (이름은 드라이버마다 조금씩 다름)
v4l2-ctl -d /dev/video0 --list-ctrls

# 오토포커스 끄고 초점을 바꿔가며 가장 선명한 값 찾기
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0
for f in 0 20 40 60 80 100 140 180 255; do
  v4l2-ctl -d /dev/video0 -c focus_absolute=$f
  echo "focus=$f"; sleep 1        # 화면 보면서 가장 선명한 값 기록
done
```

가장 선명했던 값을 `focus:`에 적고 `colcon build --packages-select sensor_fusion_bringup`.
노드 시작 시 로그에 실제 반영된 AF/AE 값이 찍히니, 안 먹었으면 위 `v4l2-ctl` 명령으로 직접 잡으면 된다.

### 그래도 먼 오탐이 남으면

`yolov8_node`의 크기·위치 필터로 거른다 (`config/params.yaml`):

```yaml
yolov8_node:
  ros__parameters:
    threshold: 0.5
    min_box_area_px: 576        # 24x24 px보다 작은 검출은 버림 (멀어서 의미 없음)
    min_box_height_px: 20
    min_box_bottom_ratio: 0.30  # 박스 아래변이 화면 위 30% 안에서 끝나면 버림(=지평선 위)
    class_thresholds: ""        # 예: "cone:0.60,drum:0.55" 클래스별로 다르게
```

너무 많이 걸러지면 로그에 "필터로 버린 검출 N개"가 주기적으로 찍히니 그걸 보고 값을 낮추면 된다.

## 8. 캘리브레이션이 바뀌면

- 카메라를 바꾸거나 재캘리브레이션하면 `fx`, `fy`, `cx`, `cy`를 새 값으로 갱신할 것
  (4개 모두 `config/params.yaml`에서 관리된다). 고친 뒤
  `colcon build --packages-select sensor_fusion_bringup`.
- 카메라 장착 각도를 바꾸면 위 6번의 캘리브레이션 도구로 다시 맞출 것. 다운틸트 자체를 아예
  새로 넣고 싶으면 `cam_pitch_deg`(또는 launch 인자 `camera_pitch_deg`)를 고친 뒤 잔차만 도구로 잡는다.
- 라이다/카메라 장착 방향을 바꾸면 `lidar_front_offset_deg` 재확인 (자세한 내용은
  [`lidar_camera_fusion_pkg` README](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) 참고)
- 새 YOLO 모델(`.pt`)을 받으면 `camera_perception_pkg/models/`에 넣고, `yolov8_node`의 `model`
  파라미터(콤마로 여러 개 지정 가능)를 그 파일명으로 맞춘 뒤 `camera_perception_pkg`를 재빌드할 것.

---

## 9. 조향 캘리브레이션 (실측 기준)

조향은 아두이노가 포텐셔미터로 위치를 되먹임하는 **폐루프**로 돈다. ROS는 각도를 직접 주지 않고
`-1.0 ~ +1.0` 정규화 값만 보내고, 펌웨어가 그걸 pot 목표값으로 바꾼다. 그래서 아래 값들이
틀어지면 인식이 아무리 정확해도 차는 엉뚱하게 간다.

### 9-1. 시리얼 프로토콜

`firmware/autonomous_mega/src/comm.ino`가 받는 형식은 **필드 2개**다.

```
C,<조향 -1.0~1.0>,<후륜 PWM -255~255>\n
```

`handleLine()`은 쉼표가 3개 이상이면 **줄 전체를 버린다**(`idx3 != -1` 검사). 뒤에 필드를
덧붙이면 명령이 통째로 무시되고 워치독(`DRIVE_WD_MS`, 300ms)이 모터를 0으로 내린다.
호스트 쪽 생성기는 `serial_communication_pkg/lib/protocol_convert_func_lib.py`.

조향 필드는 **PWM이 아니라 정규화 값**이다. 펌웨어가 `normalizedToSteeringPosition()`으로
pot 목표값으로 변환한다.

### 9-2. 펌웨어 상수 (`firmware/autonomous_mega/src/firmware_main.ino`)

| 상수 | 값 | 근거 |
|---|---|---|
| `steeringPotCenter` | 510 | 앞바퀴를 자로 재서 직진에 맞춘 뒤 A0 실측 (121샘플, 509~511) |
| `steeringPotMin` | 440 | 실측 기계적 한계 434 안쪽으로 여유 |
| `steeringPotMax` | 616 | 실측 기계적 한계 622 안쪽으로 여유 |
| `STEERING_DEADBAND` | 6 | 조향 분해능을 직접 결정한다. 아래 주의 참고 |
| `STEERING_MIN_PWM` | 55 | 목표 근처 접근용. 낮으면 지면에서 안 움직이고, 높으면 지나친다 |
| `STEERING_MAX_PWM` | 160 | 지면 정지 조향(dry steering)을 이길 만큼 |
| `STEERING_SLOWDOWN_RANGE` | 50 | 감속구간. 편측 가동폭보다 넓으면 PWM이 최대치에 도달하지 못한다 |

**중앙값은 반드시 Min~Max 사이여야 한다.** 벗어나면 `normalizedToSteeringPosition()`의
`constrain(target, Min, Max)`에 걸려 조향 목표가 한쪽 끝에 고정된다. 직진 명령(0.0)조차
풀락이 되고, 차선을 정확히 잡아도 계속 한쪽으로만 간다.

**데드밴드는 조향 분해능이다.** ROS는 `steering`을 `int32`로 `±max_steer_cmd`(=9) 정수로
보내므로 한 단계가 편측 가동폭의 1/9다. 데드밴드가 그보다 크면 명령 대부분이 삼켜져
조향이 걸리지 않는다.

가동범위는 좌 70 / 우 106으로 **비대칭**이다(링키지 자체가 치우쳐 있다. 중앙 510인데
기계적 중점은 528). 같은 크기의 명령이어도 우회전이 더 급하게 들어간다.

### 9-3. 조향 방향

`steer_invert`(`params.yaml`의 `serial_sender_node`)로 뒤집는다. **반드시 실측으로 확인할 것.**
부호가 반대면 제어기가 보정할수록 반대로 밀려서, 정지 상태는 멀쩡한데 굴리면 한쪽으로
계속 흘러간다.

확인 방법은 조향을 `+1`/`-1`로 번갈아 크게 물리고 앞바퀴를 직접 보는 것이다.

```bash
python3 firmware/tools/drive_straight.py 3 30   # 조향 중립 고정 + 직진 (기계적 트림 확인)
```

### 9-4. 펌웨어 빌드·업로드

```bash
cd firmware/autonomous_mega
~/.platformio/penv/bin/pio run --target upload
```

`/dev/ttyACM0`를 ROS 노드가 잡고 있으면 업로드가 실패한다. 런치를 먼저 내릴 것.

### 9-5. 측정 도구

```bash
python3 firmware/tools/steer_pwm_sweep.py
```

조향 최소 구동 PWM과 실제 기계적 가동범위를 단계별로 잰다. 매 측정 전에 중앙으로 복귀시키므로
이동량이 항상 중앙 기준이다. 각 단계마다 Enter를 기다리고, Ctrl+C로 즉시 정지한다
(명령 스트림이 끊기면 펌웨어 워치독도 300ms 안에 모터를 멈춘다).

> **앞바퀴를 띄우고 잴 것.** 지면에 닿은 정지 조향은 부하가 가장 커서, 기계적 한계가 아니라
> 마찰에 막힌 지점을 한계로 잘못 기록하게 된다. 반대로 최소 구동 PWM은 띄우면 실제보다
> 낮게 나오므로, 그 값은 하한으로만 쓰고 여유를 얹어야 한다.

## 10. 차선 중심 추정 (BEV)

`camera_perception_func_lib.py`의 `get_lane_center()`가 BEV ROI에서 차선 중심 x를 뽑는다.
실패하면 `-1`을 반환하고 호출측이 걸러낸다.

`draw_edges()`는 **추종 중인 클래스 하나만** 그린다. 그래서 두 차선이 다 검출돼도 이 함수가
보는 건 항상 선 1개이고, 중심은 `선위치 ± lane_width_for_center/2`로 추정된다.
**즉 `lane_width_for_center`는 일부 구간이 아니라 사실상 모든 프레임에 적용된다.**

| 파라미터 | 값 | 의미 |
|---|---|---|
| `fixed_lane_class` | `lane_2` | 추종 차선 고정. 빈 값이면 상태머신이 자동 판단 |
| `lane_width_for_center` | 216 | BEV에서 가정하는 차선 폭(px) |
| `lane_center_tilt_comp` | 1.0 | 차선 기울기(cos) 보정 강도. 0.0=꺼짐, 1.0=완전 보정 |
| `car_center_x` | 332 | BEV ROI에서의 **차량 중심선**. 영상 정중앙(320)이 아니다 |
| `target_y_end` | 95 | 목표점을 뽑을 행 범위 상한 (5/35/65 세 행만 사용) |

`car_center_x`가 320이 아닌 이유는 카메라가 차체 중심선에서 벗어나 달렸기 때문이다.
320을 쓰면 제어기가 *카메라*를 차선 중앙에 맞추고 *차체*는 그만큼 치우친 채 달린다.
값을 다시 잡으려면 앞바퀴에서 좌우 차선까지 거리를 **자로 재서** 맞춘 뒤 `target_x`를 측정한다
(눈대중으로 놓으면 ±50px씩 흔들린다).

`lane_width_for_center`는 주행 중 `target_x` 평균과 `car_center_x`의 차이로 역산한다.
`중심 = 선위치 − 폭/2`이므로 **`폭 보정량 = 2 × 편차`**다. 조향이 한쪽으로 쏠려 있으면
이 관계로 한 번에 맞출 수 있다.

### 곡선에서의 기울기 보정 (`lane_center_tilt_comp`)

`중심 = 선위치 ± 폭/2`의 이동을 **차선에 수직이 아니라 같은 행(가로)에서** 한다.
차선이 수직에서 `theta`만큼 기울면 같은 행에서의 가로 거리는 `(폭/2)/cos(theta)`이므로,
보정이 없으면 곡선에서 **항상 부족하게 밀린다**. 그만큼 추정 중심이 *지금 추종 중인 선* 쪽으로 끌려간다.

| `theta` | 실제 필요한 가로 이동 | 보정 없을 때(108px) | 부족분 |
|---|---|---|---|
| 30° | 124.7px | 108px | 16.7px |
| 40° | 141.0px | 108px | 33.0px |
| 50° | 168.0px | 108px | 60.0px |
| **60°** | **216.0px** | 108px | **108.0px** |

60°에서 부족분이 정확히 폭의 절반이다. 즉 **타겟이 선 위에 그대로 얹힌다.**
`lane_2`(좌측선, 08-18 실측으로 좌/우 정정됨 — 14번 참고)를 추종하는 좌회전 구간에서는
그 선이 곧 인코스라, "곡선에서 인코스 선을 밟고 주행"으로 나타났다.

`theta`는 `dominant_gradient()`가 이미 구해 `get_lane_center()`에 넘겨주던 값인데
그동안 부호 판정 폴백에만 쓰이고 보정에는 안 쓰였다. 현재 값은 아래로 확인한다.

```bash
ros2 topic echo /yolov8_lane_info --field slope   # 차선 기울기(도, 수직 기준)
```

보정량이 크므로(`theta`=49°면 타겟이 58px 이동) `0.5`에서 시작해 확인 후 `1.0`으로 올린다.
보정 배율은 어떤 경우에도 **2.0배로 제한**된다(`theta`가 90°에 가까우면 발산하므로).

> **[현재 상태]** `0.0`(진단용) → `0.5` 시운전을 거쳐 지금은 `lane_center_tilt_comp = 1.0`
> (완전 보정)으로 적용해 둔 상태. 실주행으로 곡선 인코스가 줄어드는지 확인할 것. 이미 최댓값이라
> 더 올릴 여지는 없고(보정 배율 2.0배 상한), 그래도 남으면 10번의 `path_planner_node` 진단이나
> 11-5의 `lookahead_distance` 쪽을 봐야 한다.

또한 `line_side`가 주어지면 `get_lane_center()`는 **항상 '선 1개' 경로**를 탄다.
`draw_edges()`가 추종 클래스 하나만 그리므로 실제로 선은 항상 1개인데, 곡선에서 마스크가
기울거나 조각나면 같은 선의 픽셀이 `lane_width/3`(=72px) 넘게 벌어져 '두 선이 보인다' 분기로
새고, 그러면 **그 선의 중앙**을 차선 중심으로 반환해 타겟이 108px 통째로 튀었다.

### 다중 장애물 회피 (`/lidar_obstacle_array`)

`image_fusion_node`가 가장 가까운 장애물 1개(`/lidar_obstacle_info`)뿐 아니라 검출된 장애물
전부를 `/lidar_obstacle_array`(Polygon, 점 하나 = `x`거리[m]/`y`중심x[px]/`z`반폭[px])로도 낸다.
가장 가까운 것 하나만 보면, 그걸 피해 지나가는 순간 박스가 사라져 판단에서 빠지고 **다음
장애물만 보고 꺾다가 직전 장애물을 들이받는다.** `lane_info_extractor_node`(차선 오프셋 판단)와
`path_planner_node`(래티스 페널티) 둘 다 이 배열을 우선 쓰고, 안 오면 기존 단일 토픽으로
대체한다(하위 호환).

### 래티스 경로 비용 함수 — 곡선 인코스 원인과 수정 (`path_planner_node`)

`select_best_candidate()`의 `lane_penalty`는 원래 아래처럼 계산됐다.

```python
lane_penalty = np.mean(np.abs(candidate_x - np.mean(candidate_x))) * lane_center_weight  # (수정 전)
```

이건 **후보 경로가 차선 중앙에서 얼마나 벗어났는지가 아니라, 후보 자기 자신의 평균에서 얼마나
흩어졌는지(직선일수록 낮음)**를 재는 식이었다. 후보는 `x_points + linspace(0, offset, N)`이라
오프셋(상수 성분)은 `candidate_x`의 평균을 그만큼 밀 뿐 산포도 자체는 거의 안 바꾸므로, 곡선
구간에서는 **offset=0보다 경로를 펴 버리는 큰 오프셋 쪽이 오히려 비용이 낮게 나올 수 있었다**
(실측: offset=0 비용 225 vs offset=+80 비용 42.5). `transition`의 최댓값이 차량 시작점 쪽에
붙으므로 이 경우 차량 기준점 자체가 옆으로 밀리고, `motion_planner_node`의 PD가 보는
`lookahead_y` 행에서도 dx가 커져 **좌/우 곡선 모두에서 안쪽으로 추가 조향**이 걸렸다.

원인을 `path_change_cost`를 10.0으로 올려(오프셋 변경 자체를 억제) 격리 확인한 뒤, **수식을
차선 중심(`x_points`)과의 편차로 고쳤다**:

```python
lane_penalty = np.mean(np.abs(candidate_x - x_points)) * lane_center_weight  # (수정 후)
```

이제 `offset=0`이면 `lane_penalty=0`이고 `|offset|`이 커질수록 정상적으로 커진다.
`path_change_cost`는 진단용 10.0에서 지그재그 억제 목적의 원래 값 `0.05`로 되돌렸다.

> **주의 (재검증 필요)**: `lane_center_weight`(10.0)는 이 버그가 있던 상태(오프셋과 거의
> 무관하게 채점되던 상태)에서 정해진 값이다. 수식이 고쳐진 지금은 `|offset|`에 실제로
> 비례해서 비용이 붙으므로, 장애물 회피(`obstacle_weight` 20 + `obstacle_penalty_gain` 3,
> 최대 감점 약 200)보다 `lane_penalty`가 더 커지는 오프셋(대략 40px 이상)에서는 래티스가
> 장애물을 피하는 쪽보다 차선 중앙 유지 쪽을 택할 수 있다. 다만 1차 회피는
> `lane_info_extractor_node`가 목표점 자체를 `lane_width_pixel`(280px)만큼 미리 옮겨주는
> 방식이라 이 영향을 크게 받지 않는다. 그래도 **장애물 회피 동작은 이 변경 이후 반드시
> 실측으로 재확인할 것.**

### 알려진 한계

- **기울기가 행별이 아님.** `theta`는 ROI 전체 Hough 각도의 중앙값이라 모든 행에 같은 값을
  쓴다. 곡선에서 가까운 행은 덜 기울어져 있으므로 근거리는 과보정, 원거리는 부족 보정이 된다.
- **행별 폭 미보정.** BEV는 원근 보정이라 실제 차선 폭이 행마다 다르다(근거리 ~125,
  원거리 ~222). 지금은 모든 행에 같은 `lane_width_for_center`를 써서, 차량에 가까운
  행(y=95, 125)은 먼 행과 70~80px 어긋난다. 그래서 `target_y_end`를 95로 두어 먼 행 3개만
  쓴다. 경로 점이 3개뿐이라 S자 곡선에서 중간 형상을 읽는 능력이 떨어진다.
  행별 폭을 쓰도록 고치면 5개로 늘릴 수 있다.
- **조향 해상도.** `MotionCommand.steering`이 `int32`라 `±max_steer_cmd`(=9) 정수 19단계다.
  더 세밀하게 하려면 `max_steer_cmd`를 키우거나(양쪽 노드 값을 같이) 메시지를 float로 바꿔야 한다.
  자세한 영향과 완화책은 11번.

## 11. 조향 출력 (순수 추종 + PD + 스무딩)

`motion_planner_node`가 `/path_planning_result`를 받아 조향 명령을 만든다.

```
steer_cmd = compute_pp_steer_cmd()   # 순수 추종(pure pursuit)
          + compute_pd_steer_cmd()   # 근거리 횡오차 PD 보정
          → clamp(±max_steer_cmd)
          → smooth_steer()           # EMA + 레이트리밋
          → round() → MotionCommand.steering (int32)
```

### 11-1. 룩어헤드 점 선택

`find_lookahead_point()`는 경로에서 **차량으로부터 `lookahead_distance` 이상 떨어진 첫 점**을 고른다.
경로는 y 오름차순, 즉 **먼 쪽 → 가까운 쪽** 순으로 들어오므로 반드시 `reversed()`로 가까운 쪽부터
훑어야 한다.

예전에는 들어온 순서 그대로(먼 쪽부터) 훑었다. 경로 최대 길이가 174px(차량 y=179, 최원점 y=5)이라
`lookahead_distance`가 174보다 작기만 하면 **최원점이 첫 검사에서 항상 통과**했다. 즉 120이든 170이든
똑같이 y=5 한 점만 봤고, 파라미터는 참조점 선택에 아무 영향이 없었다. 그 결과 곡선에서 중간 점
(y=35/65/95)의 형상을 못 읽고 최원점을 직선으로 겨냥해 **안쪽을 가로질렀다(인코스)**.

### 11-2. 조향각 공식의 분모

```python
lookahead_dist = math.hypot(lx - car_x, ly - car_y)     # 실제로 고른 점까지의 거리
steer_angle = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead_dist)
```

분모는 **파라미터가 아니라 실거리**여야 한다. 선택이 정상이면 둘이 거의 같지만, 경로가
`lookahead_distance`보다 짧아 더 먼 점으로 대체되면 그 비율만큼 조향이 과해진다.
예전에는 분모에 파라미터를 그대로 썼고, 11-1의 선택 버그와 겹쳐 실거리 174px에 분모 120을 쓰는
**1.45배 과조향**이 상시로 나갔다.

두 수정의 합산 효과(원호 경로 시뮬레이션, `lookahead_distance=120` 유지):

| 곡률반경 | 기존 조향 / 참조점 | 수정 후 조향 / 참조점 |
|---|---|---|
| R=400px | 2.83 / y=5 | 1.98 / y=59 |
| R=300px | 3.64 / y=5 | 2.57 / y=59 |
| R=200px | 4.99 / y=5 | 3.62 / y=63 |
| R=150px | 5.99 / y=5 | 4.48 / y=65 |

조향량이 약 30% 줄고 참조점이 경로 중간으로 내려온다.

> **[현재 상태]** 위 수정(11-1, 11-2) 직후 `lookahead_distance=120`을 그대로 두면 직진 횡오차
> 보정이 1.44배 과보정(위빙), 곡선 조향은 0.49배로 부족해져 직진조차 어려웠다. 그래서 지금은
> **170**으로 올려 수정 전과 비슷한 참조점(y≈12 부근)이 나오도록 맞춰 둔 상태다. 이건 10번의
> `lane_penalty`/`path_change_cost` 진단이 끝나기 전까지 다른 변수를 고정하기 위한 값이라,
> 그 진단이 끝나면 여기서부터 다시 하나씩(11-5 순서대로) 조여야 한다.

### 11-3. 출력 스무딩

조향 명령은 `int32`로 반올림되고 `serial_sender_node`가 `max_steer_cmd`로 나눠 -1.0~1.0으로
정규화한다. 즉 **명령 1칸 = 전체 조향의 1/9**이고, 조향 포텐셔미터로는 좌 `70/9≈7.8`,
우 `106/9≈11.8` counts다(펌웨어 440/510/616). 펌웨어 `STEERING_DEADBAND`가 6이라
**한 칸만 바뀌어도 항상 데드밴드를 넘어** 앞바퀴 모터가 켜졌다 확 밀고 꺼진다.

저속에서는 같은 곡률을 도는 데 제어 틱이 더 많이 들어가므로 이 계단이 하나씩 다 "틱틱"으로
느껴진다. `smooth_steer()`가 EMA와 레이트리밋으로 한 번의 큰 변화를 여러 틱에 나눠 보내
그 충격을 줄인다.

- 필터 상태(`steer_state`)는 **반올림 전 float**으로 유지한다. 반올림된 값을 되먹이면
  필터가 정수 격자에 갇혀 목표에 영영 못 닿는다.
- 정지(`publish_stop()`) 시 `steer_state`와 `prev_dx`를 0으로 리셋한다. 펌웨어는 `rear_pwm`과
  무관하게 `targetSteering`을 갱신하므로 정지 중 바퀴는 실제로 중립으로 간다.

10Hz 기준 동작:

| 입력 | 출력(반올림 후) |
|---|---|
| 0 → 9 스텝 | `3, 5, 7, 8, 8, 9` — 약 0.5s에 도달 |
| ±2 채터링 | `±0.5` → 반올림하면 대부분 0 (모터가 아예 안 움직임) |
| ±4 채터링 | `±1.0` (기존 pot 47counts → 12counts) |
| D항 스파이크 `2,2,5,2,2,5` | `1,1,3,2,2,3` |

> 입력이 `±max_steer_cmd`를 매 틱 오가는 극단적 진동이면 레이트리밋이 계속 걸리면서 출력 평균이
> 한쪽으로 치우친다(±9 입력 → 0~3 왕복). 그 정도로 떨고 있으면 스무딩이 아니라 상류(경로/PD)를
> 봐야 한다. 실제 진폭인 ±2~4에서는 레이트리밋이 걸리지 않아 대칭이다.

### 11-4. 파라미터

| 파라미터 | 값 | 의미 |
|---|---|---|
| `lookahead_distance` | 170.0 | 참조점까지의 거리(px). **작을수록 민감·인코스 감소**, 클수록 완만·인코스 증가 |
| `wheelbase` | 50.0 | 가상 휠베이스. 클수록 조향 계산이 완만 |
| `max_steer_angle_rad` | 0.55 | 조향각 정규화 기준. 작을수록 출력이 커짐 |
| `max_steer_cmd` | 9.0 | 최종 명령 최대 절대값. `serial_sender_node`와 **같은 값**이어야 함 |
| `Kp` / `Kd` | 0.01 / 0.045 | PD 보정 이득. `lookahead_y`(155) 행의 횡오차 기준 |
| `steer_smoothing_alpha` | 0.4 | EMA 계수. **1.0이면 스무딩 없음**. 10Hz에서 0.4면 시정수 약 0.2s |
| `steer_rate_limit` | 3.0 | 틱당 최대 변화량(전체 범위 ±9). 3.0이면 풀조향 왕복에 최소 0.6s. 0 이하면 제한 없음 |

`Kp`/`Kd`/`max_steer`/`steer_speed_gain`은 모두 `steer_cmd`와 같은 단위다. **`max_steer_cmd`를
바꾸면 이들을 같은 비율로 스케일해야 한다.** (예: 해상도를 높이려고 9 → 90으로 올리면
`Kp` 0.01→0.1, `Kd` 0.045→0.45, `max_steer` 4.0→40.0, `steer_speed_gain` 12.0→1.2)

### 11-5. 튜닝 순서

1. **곡선에서 인코스로 파고든다** → 10번의 `lane_center_tilt_comp`(인지 원인)와
   `path_planner_node`의 `lane_penalty`(경로 생성 원인, 수정 완료)는 이미 손봤다. 그래도
   남으면 `lookahead_distance`를 **낮춘다**(현재 기준값 170 → 140 → 120 → …).
   참조점이 차량 쪽으로 내려와 곡선을 가로지르지 않는다.
2. **곡선에서 조향이 과하다** → 같은 방향으로 `max_steer_angle_rad`를 키우거나 `wheelbase`를 키운다.
3. **여전히 틱틱거린다** → `steer_smoothing_alpha`를 0.4 → 0.25로.
4. **코너 진입이 굼뜨다** → `steer_smoothing_alpha`를 0.4 → 0.5~0.6으로. 속도를 올릴수록
   같이 올려야 한다(스무딩 지연 동안 차가 더 많이 나가므로).
5. **위로 안 잡히면** 분해능 한계다. `max_steer_cmd`를 키우고 위 표대로 이득을 재스케일한다.

## 12. 문제 해결 순서 (조향·주행)

증상별로 확인할 곳이 다르다. 위에서부터 순서대로 좁힌다.

1. **차가 아예 안 움직인다** — 시리얼 프로토콜(9-1). 아두이노가 `FRONT:0,REAR:0`을
   되돌려 보내는지 확인. 명령이 폐기되고 있으면 워치독이 계속 0으로 내린다.
2. **삐 소리만 나고 안 굴러간다** — 구동 PWM이 정지마찰보다 낮다. `base_speed`를 올린다.
   펌웨어 `setMotor()`에는 구동 모터 PWM 하한 보정이 없다.
3. **조향이 안 걸린다** — 데드밴드가 명령 한 단계보다 큰지 확인(9-2).
4. **조향은 되는데 방향이 반대로 간다** — `steer_invert`(9-3).
5. **직진 명령으로 굴렸을 때 한쪽으로 간다** — 기계적 트림. `steeringPotCenter`를 옮긴다.
   `drive_straight.py`로 확인.
6. **직진은 되는데 자율주행에서 한쪽으로 치우친다** — 인식 기준점. `car_center_x`와
   `lane_width_for_center`(10번).
7. **곡선에서 인코스 선을 밟는다 / 곡선에서만 조향이 과하다** — 먼저 **인지**를 의심한다.
   차선 기울기 보정(`lane_center_tilt_comp`, 10번)이 꺼져 있으면 곡선에서 타겟이 추종 중인
   선 쪽으로 최대 폭의 절반만큼 끌려간다. 제어 게인을 만지기 전에 이걸 먼저 볼 것.
8. **곡선을 못 따라간다(바깥으로 밀린다)** — 룩어헤드(11-1, 11-2).
   `lookahead_distance`는 **작을수록 민감하고 인코스가 준다**. 경로 점 개수도 함께 볼 것.
9. **조향이 부드럽지 않고 틱틱거린다** — 조향 해상도와 펌웨어 데드밴드(11-3).
   `steer_smoothing_alpha`를 낮춘다. 그래도 남으면 `max_steer_cmd`를 키운다(11-4).

## 13. 후방 서라운드 BEV (카메라 2/3/4)

후방 카메라 1대 + 좌/우 카메라 2대를 지면에 투영해서, 차 뒤를 위에서 내려다본 한 장으로
합친다. 후진/주차할 때 세 화면을 번갈아 보지 않아도 되게 하는 게 목적이다.

- `camera_2` (by-path 2.2.4.1) — 후방, 뒤를 정면으로 본다
- `camera_3` (by-path 2.2.4.2) — 좌측, 비스듬히 뒤를 본다
- `camera_4` (by-path 2.2.4.3) — 우측, 비스듬히 뒤를 본다

원리는 단순하다. **지면이 평면이라고 가정하면** 카메라 영상의 한 점과 지면의 한 점은
호모그래피 하나로 1:1 대응하고, 그 호모그래피는 대응점 네 쌍이면 결정된다. 그래서 각
카메라마다 "영상 속 지면 사다리꼴 네 점(`src_points`)"과 "그 점이 top-down 캔버스에서
놓일 자리(`dst_points`)"만 정해주면 된다. 이 가정이 깨질 때 무슨 일이 생기는지는 아래
"한계"에 적었다.

### 13-1. 캔버스 좌표계 — 왜 지금 안 쓰는 앞쪽까지 잡아뒀나

캔버스는 **차를 한가운데 둔 800x1000 지면 좌표계**다. 위가 차 앞, 아래가 차 뒤,
x가 클수록 차의 오른쪽이고 `[310,370,490,630]`이 차량 자리다.

지금 쓰는 건 뒤쪽뿐인데 앞쪽까지 잡아둔 이유는 `dst_points`가 **이 캔버스의 픽셀
좌표로 저장되기 때문**이다. 나중에 전방 카메라를 붙이면서 캔버스를 넓히면, 힘들게 찍어둔
네 점이 전부 어긋나서 세 카메라를 처음부터 다시 맞춰야 한다. 그래서 좌표계는 처음부터
최종 형태로 못박고, 당장 안 쓰는 앞쪽은 `view_rect: [0, 340, 800, 1000]`로 잘라낸다.

이 크롭은 다 그린 뒤에 잘라내는 게 아니라 **평행이동을 호모그래피에 미리 곱해서** 애초에
그 영역만 워핑한다. 안 쓰는 영역은 계산 자체를 안 하므로 그만큼 빠르다.

전방을 추가할 때는 `front.enabled: true` + `view_rect: [0, 0, 800, 1000]`으로 바꾸고
전방 네 점만 찍으면 된다. 후방/좌우는 다시 안 찍어도 된다.

### 13-2. 겹치는 영역 — 평균내지 않고 후방에 우선권을 준다

좌/우 카메라가 비스듬히 뒤를 보고 있어서 후방과 겹치는 영역이 넓다. 처음에는 겹치는 곳을
가중 평균했는데(`blend_mode: average`), 광각 왜곡이 심한 측면 영상이 선명한 후방 영상 위에
번져서 **둘 다 흐려졌다**. 정렬이 완벽하지 않으면 평균은 이중상이 된다.

그래서 `blend_mode: priority`를 쓴다. `blend_priority`가 높은 쪽(rear=30)이 겹치는 영역을
가져가고, 좌/우(10)는 후방이 못 보는 바깥만 채운다. 결과적으로 화면에 보이는 어긋남은
이음매 선 하나로 줄어든다.

구현상 주의점이 하나 있다. 우선순위 순으로 알파 합성을 하되 **누적 알파로 나눠서 되돌려야**
한다. 안 나누면 이음매를 부드럽게 하려고 넣은 깃털(feather) 구간의 알파가 1보다 작아서,
가장 아래 레이어의 바깥 테두리가 검게 죽는다.

### 13-3. ROI 피커 — 네 점을 어떻게 정할 것인가

이 작업의 실질적인 난이도는 전부 여기 있다. 바닥에 마커를 놓고 자로 재서 픽셀 좌표를 손으로
적어 넣는 건 비현실적이고, 값을 바꿀 때마다 노드를 재시작해서 확인하는 것도 못 할 짓이다.

그래서 도구를 만들었다 (`roi_picker_node`, `roi_picker.launch.py`).

```bash
ros2 launch sensor_fusion_bringup roi_picker.launch.py start_cameras:=true
```

왼쪽 패널은 카메라 원본, 오른쪽 패널은 top-down 캔버스다. **바닥의 같은 지점을 양쪽
패널에서 같은 순서로** 클릭하면 그 자리에서 워핑 결과가 캔버스에 겹쳐 나온다. 점 순서는
사각형을 한 바퀴 도는 순서면 무엇이든 되고(두 패널에서 같은 순서이기만 하면 대응이 맞는다),
자기교차하면 하단에 경고가 뜬다.

여러 카메라를 맞추는 방법이 핵심이다. **후방을 먼저 맞춰 기준을 만들고, 좌/우는 후방
워핑이 이미 그려진 캔버스 위에서 "후방 영상에 보이는 바로 그 바닥 지점"을 클릭한다.**
그러면 축척과 위치가 동시에 맞는다. 겹치는 영역 밖의 점을 쓰면 기준이 없어서 어긋난다.

`s`를 누르면 `params.yaml`의 해당 줄만 골라 치환한다. YAML을 파싱해서 다시 쓰면 그 파일에
잔뜩 달린 주석이 전부 날아가기 때문에 줄 단위 치환을 쓴다. 원본은 `params.yaml.bak`으로
백업된다.

| 키 | 동작 |
| --- | --- |
| 좌클릭 | 점 추가 (4개 다 찍힌 뒤에는 가장 가까운 점을 그 자리로 이동) |
| 우클릭 | 마지막 점 취소 |
| `h` `j` `k` `l` | 마지막으로 건드린 점을 1픽셀씩 이동 |
| `1` `2` `3` / `Tab` | 카메라 전환 (rear / left / right) |
| `r` / `z` | 현재 카메라 초기화 / 되돌리기 |
| `a` | dst 사각형 대충 깔아주기 (끌어다 맞추는 출발점) |
| `d` | 확인 모드 순환: `stack` → `diff` → `blink` |
| `x` | 카메라 패널 접고 캔버스만 크게 |
| `w` `g` `f` | 워핑 / 격자 / 캔버스 전체 보기 |
| `s` | `params.yaml`에 저장 |

`d`(확인 모드)는 나중에 추가한 것인데, 없으면 작업이 불가능했다. 기본 `stack` 모드는
우선순위가 높은 후방이 좌/우를 **덮어써서**, 정작 겹치는 영역에서 얼마나 어긋났는지 볼 수가
없다. `diff`는 겹치는 영역을 `|현재 카메라 − 나머지|`로 그려서 **맞으면 까맣게** 되고,
`blink`는 두 영상을 2.5Hz로 번갈아 보여준다(사람 눈은 정지 비교보다 깜빡임에 훨씬 민감하다).

### 13-4. 실행 — 명령어 전체 순서

**0) 빌드 (처음 한 번, 또는 노드를 고쳤을 때)**

```bash
cd ~/UNITA_sensor_fusion_ws
colcon build --packages-select camera_perception_pkg sensor_fusion_bringup --symlink-install
source install/setup.bash
```

`--symlink-install`이라 `.py` 파일만 고친 경우엔 다시 빌드하지 않아도 된다(다음 실행부터
바로 반영). `setup.py`의 entry_points를 건드렸을 때만 다시 빌드하면 된다.

**1) 카메라 장치 확인 (재연결/재부팅 후)**

```bash
ros2 run camera_perception_pkg list_cameras --probe
ls /dev/v4l/by-path/          # by-path 경로가 params.yaml의 camera_device와 맞는지
```

C920이 여러 대라 by-id가 서로 겹쳐서 못 쓴다. 반드시 **by-path**(꽂은 포트 기준)를 쓴다.
포트를 바꿔 꽂으면 경로가 깨지므로 위 명령으로 다시 확인할 것.

**2) 카메라 3대 원본을 격자로 확인**

```bash
ros2 launch sensor_fusion_bringup multi_camera_view.launch.py
```

`/camera_2,3,4/image_raw`를 한 창에 붙여서 보여준다. 프레임이 안 오는 카메라는 회색
`NO SIGNAL` 타일로 표시되므로, 창이 아예 안 뜨는 것과 카메라 하나만 죽은 것을 구분할 수 있다.
런치 인자: `show_preview`, `tile_width`(기본 480), `tile_height`(360), `resizable_window`.

**3) ROI 찍기 (13-3 참고)**

```bash
# 카메라가 이미 떠 있으면
ros2 launch sensor_fusion_bringup roi_picker.launch.py

# 아무것도 안 떠 있으면 카메라까지 같이
ros2 launch sensor_fusion_bringup roi_picker.launch.py start_cameras:=true
```

`3` → `x`(캔버스 크게) → `d` 두 번(diff 모드)으로 이음매를 보면서 맞추고 `s`로 저장.

**4) 서라운드 BEV 실행**

```bash
ros2 launch sensor_fusion_bringup rear_surround_view.launch.py start_cameras:=true

# 창 없이 토픽만 (원격/헤드리스)
ros2 launch sensor_fusion_bringup rear_surround_view.launch.py \
    start_cameras:=true show_preview:=false

# 카메라별 커버리지 윤곽선 표시 (정렬 확인용)
ros2 launch sensor_fusion_bringup rear_surround_view.launch.py \
    start_cameras:=true draw_camera_outlines:=true

# 겹치는 곳을 평균내는 예전 방식으로 비교해보고 싶을 때
ros2 launch sensor_fusion_bringup rear_surround_view.launch.py \
    start_cameras:=true blend_mode:=average
```

두 런치 모두 `start_cameras`가 기본 `false`다. 이미 카메라 노드가 떠 있는데 여기서 또 열면
`/dev/video*`를 이미 잡고 있어서 `cv2.VideoCapture`가 실패한다. 카메라가 안 떠 있을 때만
`start_cameras:=true`를 준다. 카메라를 2초씩 벌려 켜는 것도 같은 이유다 — 동시에 열면
VideoCapture가 멈추거나 열기 자체가 깨진다(USB 전원이 빠듯한 구성).

**5) 동작 확인 (다른 터미널에서)**

```bash
source ~/UNITA_sensor_fusion_ws/install/setup.bash

ros2 node list                     # camera_2/3/4_node + surround_view_node
ros2 topic hz /camera_2/image_raw  # 각 카메라 20~28Hz 정도
ros2 topic hz /surround_view/image # 13~19Hz (다른 노드와 CPU 경합에 따라)
ros2 topic info -v /surround_view/image
```

**6) 개별 노드로 나눠서 실행 (문제 추적용)**

```bash
# 카메라 한 대만
ros2 run camera_perception_pkg image_publisher_node --ros-args \
    --params-file install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/params.yaml \
    -r __node:=camera_2_node

# 서라운드 노드만 (카메라가 먼저 떠 있어야 함)
ros2 run camera_perception_pkg surround_view_node --ros-args \
    --params-file install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/params.yaml \
    -r __node:=surround_view_node -p show_preview:=false

# 피커만
ros2 run camera_perception_pkg roi_picker_node --ros-args \
    --params-file install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/params.yaml \
    -r __node:=roi_picker_node
```

**7) 종료**

```
Ctrl+C          # 런치를 띄운 터미널에서
```

`ros2 launch`를 죽여도 **자식 노드가 고아로 남는 경우가 있다**(실제로 겪었다). 창이 그대로
떠 있거나 `/dev/video*`가 계속 잡혀 있으면 직접 정리한다.

```bash
pgrep -af "surround_view_node|image_publisher_node|roi_picker_node"
pkill -f "[s]urround_view_node"      # 대괄호는 pkill이 자기 명령줄을 매칭하지 않게 하는 관용구
pkill -f "[i]mage_publisher_node"
pkill -f "[r]oi_picker_node"
```

**자주 나는 문제**

| 증상 | 원인 / 확인 |
| --- | --- |
| `VideoCapture` 실패, 창이 안 뜸 | 다른 노드가 `/dev/video*`를 잡고 있다. 위 `pgrep`으로 확인 |
| 특정 타일만 `NO SIGNAL` | 그 카메라의 USB가 빠졌거나 by-path 경로가 바뀜. `list_cameras --probe` |
| 피커에서 키가 안 먹음 | OpenCV 창은 **포커스가 있어야** 키를 받는다. 창을 클릭한 뒤 누를 것 |
| 창이 화면 밖으로 잘림 | 이 장비 화면이 1024x768이다. `roi_picker_node.pane_height`(기본 340)를 줄인다 |
| `No package metadata was found` | `install/setup.bash`를 source 안 했거나 워크스페이스 밖에서 실행함 |

### 13-5. 실제로 맞춘 과정 — 무엇이 통했고 무엇이 안 통했나

바닥이 체커보드라 정렬 확인에 유리한 조건이었는데도 쉽지 않았다. 기록해 둔다.

**1) 축척 불일치 (첫 시도)** — 카메라마다 체커보드 한 칸이 캔버스에서 30px / 57px / 64px로
2배 넘게 차이 났다. 카메라별로 제각각인 바닥 지점을 찍고 dst도 제각각인 크기로 놓은 탓이다.
**캔버스 격자(50px)를 자로 삼아 "체커보드 1칸 = 격자 1칸"으로 통일**하니 해결됐다.

**2) 후방이 기준이다** — 후방을 옮기면 좌/우가 **동시에** 어긋난다. 실제로 우측을 맞추려고
후방을 건드렸다가 좌측이 0.950 → 0.577로 무너진 적이 있다. 후방을 바꿔야 한다면 좌/우를
반드시 다시 맞춰야 한다. (반대로, 후방을 옮긴 게 더 나은 기준이었던 경우도 있었다 — 그때는
되돌리지 말고 좌/우를 새 기준에 다시 맞추는 게 맞다.)

**3) 자동 정렬 시도 — 대부분 실패했다**

- **ECC 영상 정합**(`cv2.findTransformECC`, euclidean/affine/homography): 전부 수렴 실패.
  체커보드가 반복 무늬라 목적함수에 국소최소가 촘촘히 깔려 있어서 옆 칸에 붙어버린다.
- **평행이동 브루트포스**(±45px 전수 탐색): 좌측은 MAD 55.4 → 31.4로 확실히 좋아졌지만,
  우측은 51.7 → 47.8로 거의 무효였다. 우측 오차가 단순 평행이동이 아니라는 뜻이다.
- **dst 네 점 자유(8자유도) 좌표하강**: 우측 MAD 47.6 → 39.2로 줄긴 했는데 점이 **144px**
  움직이면서 y가 1040까지, 즉 캔버스(높이 1000) 밖으로 나갔다. 좁은 이음매 띠에 과적합한
  것이라 폐기했다.

결국 **사람이 `diff` 모드를 보면서 손으로 맞추는 게 가장 잘 통했다.** 자동화는 좌측의
평행이동 보정 한 번만 실제로 기여했다.

### 13-6. 정렬을 어떻게 측정했나 — 그리고 그 지표의 한계

겹치는 영역에서 두 워핑 영상의 **평균절대차(MAD)와 상관계수**를 쟀다. 눈으로 보는 것보다
객관적이지만, 이 지표에는 함정이 세 개 있다.

- **반복 무늬에서 값이 접힌다.** 체커보드는 반 칸(약 35px) 밀리면 MAD가 최대가 되고 한 칸
  밀리면 다시 0으로 돌아온다. 그래서 "몇 px 어긋났다"를 읽을 수 없고, 거리별 오차 추세도
  단조롭게 나오지 않는다. 두 설정의 우열을 가리는 데는 쓸 수 있어도 절대 크기는 못 읽는다.
- **겹치는 면적이 다르면 평균 비교가 불공정하다.** ROI를 넓히면 안 맞는 바깥 영역이 평균에
  더 들어와서 나빠 보인다. 반드시 **두 설정의 공통 픽셀에서만** 비교해야 한다.
- **장면이 변하면 값이 요동친다.** 실제로 이음매를 걸어서 확인하는 동안 측정값이
  우측 +0.92 ↔ -0.03으로 튀었다. 사람이 화면에 들어왔다 나갔다 한 탓이다.

**가장 믿을 만한 검사는 이음매를 밟고 지나가며 발이 이어지는지 보는 것이다.** 발은 반복되지
않는 물체라 이어지면 이어진 것이고, 체커보드 상관계수가 못 보는 걸 본다.

### 13-7. 성능 — 6Hz에서 17Hz로

처음 돌렸을 때 목표 20Hz에 한참 못 미치는 5.7Hz가 나왔다. 원인은 **매 프레임 다시 계산하던
상수**였다.

- 깃털 처리용 `GaussianBlur`: 800x600에서 15ms, 카메라 3대분
- 블렌딩(float32 알파 합성 3장): 55ms

호모그래피도 블렌딩 마스크도 가중치도 **입력 해상도에만 의존**하므로 한 번 구하면 그만이다.
캐시로 바꾸고(7.2Hz), 블렌딩은 **정규화된 가중치를 미리 만들어 두고 프레임당 가중합 한
번**으로 바꿔서(55ms → 15.6ms) 19Hz까지 올렸다. 수식은 그대로라 결과 이미지는 동일하다
(참조 구현 대조 시 최대 차이 1, 부동소수 연산 순서 차이).

가중치는 **살아있는 카메라 조합별로** 캐시한다. 한 대가 끊기면 남은 조합으로 다시
정규화해야 하기 때문이다.

### 13-8. 한계와 다음 단계

체커보드 바닥에서 실측한 최종 정렬은 **후방↔우측 상관 0.92, 후방↔좌측 0.52**다.

가장 큰 제약은 **바닥이 완전한 평면이 아니라는 것**이다. 기준으로 쓴 판이 휘어 있어서
호모그래피의 전제가 깨진다. 이러면 네 점 중 어디를 맞추든 나머지가 틀어지고, 실제로 수동
조정을 세 번 반복해도 0.885 / 0.772 / 0.854로 수렴하지 않고 잡음 안에서 오갔다.

- **N점 최소자승** — 점을 5개 이상 찍어 `cv2.findHomography`로 풀면 휨이 여러 점에 분산돼
  "평균적으로 가장 덜 틀린" 평면이 잡힌다. 4점 한계를 실제로 넘는 방법이다.
- **렌즈 왜곡 보정** — 광각 왜곡이 측면 카메라에서 특히 크다. 내부파라미터(fx/fy/cx/cy,
  distortion)는 `params.yaml`의 각 `camera_N_node` 섹션에 이미 저장돼 있지만 아직 어떤
  노드도 읽지 않는다. 여기를 붙이면 측면이 눈에 띄게 좋아질 것이다.
- **전방 카메라 추가** — 13-1에 적은 대로 `view_rect`만 비우면 좌표계는 그대로 쓴다.

## 14. 변경 이력

### 2026-08-19

**후방 서라운드 BEV 추가 (13번)**
- 카메라 2/3/4를 지면 투영해 한 장으로 합치는 `surround_view_node` 정비, 네 점을 마우스로
  찍는 `roi_picker_node` 신규 작성, 런치 2개 추가.
- 겹치는 영역을 평균내면 왜곡 심한 측면이 후방 위에 번져서 `blend_mode: priority` 도입.
- 캔버스를 차 중심 800x1000으로 못박고 `view_rect`로 크롭 — 전방 카메라를 나중에 붙여도
  찍어둔 점이 유효하도록.
- 호모그래피/마스크/가중치 캐시로 5.7Hz → 17Hz.
- 실측 정렬 후방↔우측 0.92, 후방↔좌측 0.52. 바닥 판이 휘어 평면 가정이 깨지는 게 한계.

### 2026-08-14

**곡선 인코스(안쪽으로 붙어서 주행) 진단/수정**
- `lane_center_tilt_comp`: `0.0`(진단용) → `0.5` 시운전 → `1.0`(완전 보정)으로 상향. 10번 참고.
- `path_planner_node`의 `lane_penalty` 수식 버그 수정: "후보가 자기 평균에서 흩어진 정도"를
  재던 걸 "후보와 실제 차선 중심(`x_points`)의 편차"로 고쳤다. 곡선에서 오히려 경로를 펴버리는
  오프셋이 더 싸게 나오던 문제. 10번 참고.
- `path_change_cost`: 원인 확인용으로 올려뒀던 `10.0`을 원래 목적(지그재그 억제)에 맞는
  `0.05`로 복원.
- `roi_cutting_idx`: `300` → `260`으로 낮춰 타겟 포인트 추출 범위를 조금 더 원거리로 확장
  (곡선을 더 일찍 읽도록). `lane_width_for_center`(216)가 이 확장된 구간에서도 맞는지는
  미검증 — 시운전하며 `target_points`가 튀는지 볼 것.

**속도**
- `base_speed`/`min_speed`/`max_speed`: `30` → `40` (+33%). 저속 튜닝 모드(셋 다 동일값)는 유지.

**장애물 회피 — 인식 끊김에 의한 조기 복귀(콘 충돌) 수정**
- `lane_info_extractor_node`에 `active_avoidance_offset` 상태와
  `avoidance_release_threshold_count`(기본 15프레임) 디바운싱 추가.
- 회피 시작은 즉시, 복귀는 "장애물이 확실히 안 겹침"이 N프레임 연속 확인돼야만 하도록 변경.
  "미검출(정보 없음)"은 복귀 카운트에 반영되지 않고 현재 회피 상태를 그대로 유지 — 장애물에
  가까워질수록 YOLO 인식이 끊기는 근접 구간에서 조기 복귀하다 그대로 충돌하던 사고 원인.

**라이다 재연결(포트 충돌) 문제**
- 원인: 이전 `ros2 launch` 프로세스 트리가 완전히 종료되기 전에 재launch하면 `rplidar_node`가
  `/dev/ttyUSB0`를 못 열어 즉시 죽음(SDK 에러 `0x80008004`). 라이다 자체 고장이 아니라
  launch 타이밍 문제였음을 로그로 확인.
- 워크스페이스 루트에 `stop_drive.sh` 추가: 이전 launch를 정상 종료(최대 15초 대기 후
  강제종료)까지 확인하고 포트가 비었는지 알려주는 수동 정리 스크립트.
- `full_bringup.launch.py` 맨 앞에 `OpaqueFunction(_kill_stale_launch)` 추가: `ros2 launch`
  실행 시 이전 launch가 살아있으면 자동으로 정리 후 시작하도록 해서, 수동으로 `stop_drive.sh`를
  먼저 실행할 필요가 없게 함. `--show-args` 같은 정적 조회에서는 실행되지 않도록
  OpaqueFunction으로 감쌈(실제 실행 시에만 동작 확인됨).

**진단용 임시 코드 (아직 안 지움)**
- `camera_perception_func_lib.py`의 `get_lane_center()`에 `lane_center_debug` 로거로 15프레임마다
  분기(단일선/두선)·gap·theta·타겟값을 찍는 임시 로그가 남아있음. 곡선에서 "두 선 보임" 오분류
  분기(→ `tilt_comp` 미적용)가 실제로 뜨는지 확인용. 확인 끝나면 제거할 것.

**문서 오류 수정**
- 2번(빌드) 섹션의 `cd ~/sensor_fusion_ws`가 잘못된 경로였음(실제 폴더명은
  `UNITA_sensor_fusion_ws`). 이 경로로 `cd`가 조용히 실패하면 이후 `source install/setup.bash`도
  엉뚱한 디렉터리에서 실행돼 환경이 안 잡히고, `ros2 launch`가 패키지를 못 찾아 아무것도 안 뜬다
  ("라이다가 안 뜬다"로 보였던 원인 중 하나로 추정). `cd ~/UNITA_sensor_fusion_ws`로 정정.

### 2026-08-18

**차선 인식 오분류 원인 확정 및 수정 — 08-14에 남겨둔 진단 로그로 실제로 잡았다**
- 실주행 로그에서 `get_lane_center()`의 "두 선 보임" 분기(`gap < lane_width/3`=72px일 때만
  타야 함)가 **gap 300px 안팎에서도 계속 잘못 발동**하는 게 확인됐다. 이 분기를 타면
  "두 선의 중점"이라며 사실상 무관한 두 점의 평균을 타겟으로 써서 값이 계속 밀리고
  (`raw_target_x` 382→491px로 표류), 결국 `Not enough valid lane points`로 인식이
  통째로 무너졌다. **차선 밖으로 나가 벽에 부딪힌 원인이 이것이었다.**
- 이 분기에서는 `tilt_comp`가 아예 적용이 안 돼서, 08-14에 `tilt_comp`를 0.5→1.0→0.0으로
  계속 바꿔봐도 증상이 그대로였던 이유도 이걸로 설명된다 — 애초에 잘못된 분기를 보고 있었다.
- `lane_center_force_single_line: false → true`로 이 분기를 막았다. 항상 단일선+half_width
  (±`tilt_comp`) 계산을 쓰게 된다. 주의: `lane_width_for_center`(216)는 이 분기가 섞여 있던
  상태에서 역산된 값이라 재보정이 필요할 수 있다(README 10번). `tilt_comp`는 0.0으로 꺼둔
  채 우선 안정성부터 확인하고, 이제는 실제로 적용되니 그 다음에 다시 조정할 것.

**장애물 회피 동작을 "일시 회피 후 자동 복귀"에서 "옮긴 차선 유지 후 재회피"로 변경**
- 기존: 장애물을 피해 옆 차선으로 옮긴 뒤, 장애물이 안 보이면(N프레임 연속 확인) 자동으로
  원래 차선에 복귀.
- 변경: 옮긴 차선을 계속 유지하며 주행하다가, **그 차선에서 새 장애물을 만나야만** 반대쪽으로
  다시 옮긴다. 진입/복귀 둘 다 "지금 있는 차선에 장애물 확인"이라는 같은 이벤트로 통일해서
  즉시 반응(시간 기반 디바운싱 제거, `avoidance_release_threshold_count` 파라미터도 함께 제거).
- **회피 중 차선 전환 시 위치가 튀던 버그 수정**: 회피로 `current_offset`이 이동 중일 때
  추종 선이 바뀌면(예: 회피하려고 옆으로 옮겨서 원래 선이 화면에서 사라짐) 그 순간
  `final_offset_modifier`가 한 프레임 만에 `±lane_width_pixel`만큼 튀는데, `current_offset`이
  이 전환을 모른 채 이전 값을 그대로 이어받아 최종 타겟이 차선 폭만큼 더 밀렸다("옆 차선이
  아니라 차선 밖으로 나감"의 원인 중 하나). `final_offset_modifier`가 바뀐 만큼
  `current_offset`도 같이 보정해서 화면상 타겟 위치가 끊기지 않게 고쳤다.

**카메라 2/3/4번 추가 (4캠 서라운드뷰 준비)**
- 기존 1번(cam_num=0, YOLO 적용 주행용)은 안 건드림. 2번(후면, cam_num=2)/3번(좌측,
  cam_num=4)/4번(우측, cam_num=6)을 물리 연결하고 `params.yaml`에 `camera_2_node`/
  `camera_3_node`/`camera_4_node` 섹션으로 각각 `camera_device`(by-path)와 캘리브레이션
  (fx/fy/cx/cy/distortion, 아직 코드에서는 안 읽음 — 왜곡보정 미구현)을 저장해뒀다.
  YOLO/라이다 퓨전/버드아이뷰는 아직 안 붙임(추후 4대 동시 서라운드뷰 작업 때 추가 예정).
  전용 런치파일은 안 만들고, 나중에 4대를 한 launch에 몰아넣기 쉽도록 노드 파라미터
  섹션만 준비해둔 상태. 테스트는 `ros2 run camera_perception_pkg image_publisher_node
  --ros-args -r __node:=camera_2_node --params-file <params.yaml 경로> -p logger:=true`처럼
  노드 이름만 remap해서 확인.
- C920이 여러 대라 by-id가 서로 겹쳐서(연결 순서에 따라 하나만 by-id를 가짐) 반드시
  by-path(꽂은 포트 기준)를 써야 한다. 포트를 바꿔 꽂으면 깨지니 재연결 시
  `list_cameras --probe`로 다시 확인할 것.

**기존 1번 카메라 `camera_device` 포트 경로가 실제와 달라서 계속 실패하던 문제**
- `image_publisher_node`의 `camera_device`가 `...-2.2.4.3:1.0-video-index0`으로 박혀 있었는데
  실제 연결은 `...-2.2.3:1.0-video-index0`이었다. by-path는 꽂은 포트가 바뀌면 깨진다는 걸
  코드 주석에 이미 적어뒀었는데 그대로 재현된 사례. `2.2.3`으로 정정.
- `image_fusion_node`의 `[No Image] image_topic='/image_raw' is not arriving` 경고로
  확인했다 — 카메라가 "멈춘 것"처럼 보였지만 실제로는 즉시 실패하고 있었을 뿐이었다.

**카메라도 라이다처럼 기동 순서 분리** (`fusion_bringup.launch.py`)
- `image_publisher_node`도 YOLO 등 무거운 노드와 동시에 뜨면 `cv2.VideoCapture()` 오픈이
  멈추는 증상이 있었다(단독 실행은 매번 성공). 라이다(0초) → 카메라(3초) → YOLO/버드아이뷰/
  퓨전(5초) 순으로 지연 기동하도록 변경. `decision_start_delay`도 8.0 → 10.0초로 같이 늦춤.

**기타**
- `l_shape_node`의 `launch_rviz`가 `serial_sender_node` 섹션 밑에 잘못 들어가 있어서 실제로는
  안 읽히고 항상 `false`(fallback)였던 죽은 설정을 `l_shape_node` 섹션으로 옮기고 `false`로 명시.
- 워크스페이스 여러 곳에 오래된 고아 프로세스(부모 `ros2 launch`가 죽은 뒤에도 계속 남아있던
  자식 노드들)가 쌓여 있던 걸 발견하고 정리. `full_bringup.launch.py`의 자동 정리 로직은
  "다른 `ros2 launch` 프로세스"만 찾아서 죽이므로, 부모 없이 남은 고아 노드는 못 잡는다는
  한계가 있음(알려진 한계로 기록).

**차선 좌/우 매핑이 반대였음 — `lane_2`=좌측선, `lane_1`=우측선으로 정정 (실측 확인)**
- 지금까지 코드 전체(주석·로직)가 "lane_1=좌측선, lane_2=우측선"으로 가정하고 있었는데,
  실제로는 반대였다. 오늘 회피 중 "왼쪽에 벽이 있는데도 왼쪽으로 이동해 차선을 완전히
  벗어나는" 증상의 근본 원인이었다.
- 영향받은 부분 전부 수정: `lane_info_extractor_node.py`의 `get_lane_center()` 호출 시
  `line_side` 매핑, 회피 오프셋 부호(트리거·복귀 방향), 한쪽 선만 보일 때 반대 선으로
  대체 추종하는 `final_offset_modifier` 부호, 박스 미검출 시 픽셀 기준 좌/우 판정.
  `camera_perception_func_lib.py`/README의 관련 주석도 같이 정정.
- `fixed_lane_class: 'lane_2'`(좌측선 고정 추종)라 이 오류가 특히 크게 나타났다 — 회피
  트리거 방향이 항상 반대로 나가고 있었다.
