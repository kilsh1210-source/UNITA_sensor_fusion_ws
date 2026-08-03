# sensor_fusion_ws

카메라(YOLOv8) + 라이다(RPLIDAR C1)를 퓨전해서, 검출된 물체까지의 거리를 측정하는 ROS2(Humble) 워크스페이스.

패키지별 상세 설명은 각 패키지의 README 참고:
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

기본값(`cam_num:=0`, `serial_port:=/dev/ttyUSB0`)과 다르면, **소스를 고칠 필요 없이** launch 인자로 넘기면 된다:

```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py \
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

## 3. 전체 파이프라인 실행

라이다 + 카메라 + YOLO + 퓨전(거리 측정) 노드를 한 번에 띄운다.

```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py
```

- 정상 동작하면 `"Sensor Fusion"`이라는 이름의 OpenCV 창이 떠서, 카메라 화면에 YOLO bbox와
  `"클래스명 거리m"` 텍스트가 겹쳐서 표시된다.
- 화면을 마우스로 클릭하면 그 지점 방향의 라이다 거리도 표시된다 (물체 검출과 무관하게 확인용).

### launch 인자 (필요할 때만 덮어쓰기)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `cam_num` | `0` | 카메라 장치 번호, 1번에서 확인한 값 |
| `serial_port` | `/dev/ttyUSB0` | 라이다가 다른 포트로 잡히면 변경 |
| `serial_baudrate` | `460800` | RPLIDAR C1 기준값 |
| `frame_id` | `laser` | 라이다 스캔 좌표계 이름 |
| `device` | `cpu` | YOLO 추론 디바이스 (`cpu` / `cuda:0`) |
| `camera_fov` | `59.5399` | 카메라 수평 화각(도), 캘리브레이션 fx=559.431712 기준 산출값 |
| `lidar_front_offset_deg` | `180.0` | 라이다 0도 방향과 카메라 정면 방향의 차이(도) |

예:
```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py serial_port:=/dev/ttyUSB1 device:=cuda:0
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
ros2 run camera_perception_pkg yolov8_node --ros-args \
  -p model:=$(ros2 pkg prefix camera_perception_pkg)/share/camera_perception_pkg/models/best.pt \
  -p device:=cpu

# 퓨전만 (카메라·라이다·YOLO가 먼저 떠 있어야 함)
ros2 run lidar_camera_fusion_pkg sensor_fusion_node --ros-args \
  -p camera_fov:=59.5399 -p lidar_front_offset_deg:=180.0
```

### 자주 나는 에러

- `can't open camera by index` — 다른 프로세스가 이미 해당 `/dev/video*`를 잡고 있거나(`fuser /dev/video0`로 확인 후
  `kill -9 <pid>`), 1-2번에서 확인한 것과 다른 `cam_num`을 쓰고 있는 경우.
- `serial_port` 관련 permission denied — `groups`에 `dialout` 없으면
  `sudo usermod -aG dialout $USER` 후 재로그인.
- YOLO가 뜨긴 하는데 거리 텍스트가 항상 `N/A` — `/scan` 토픽이 안 들어오고 있거나
  (라이다 미연결/포트 오류), `lidar_front_offset_deg`가 실제 마운트 방향과 안 맞는 경우.

## 6. 캘리브레이션이 바뀌면

- 카메라를 바꾸거나 재캘리브레이션하면 `camera_fov`를 다시 계산해서 launch 인자로 넘길 것:
  `camera_fov = 2 * atan(width / (2 * fx))` (도 단위 변환 필요)
- 라이다/카메라 장착 방향을 바꾸면 `lidar_front_offset_deg` 재확인 (자세한 내용은
  [`lidar_camera_fusion_pkg` README](src/sensor_fusion_ws/lidar_camera_fusion_pkg/README.md) 참고)
