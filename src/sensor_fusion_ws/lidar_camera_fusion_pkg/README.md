# lidar_camera_fusion_pkg

카메라(YOLO 검출)와 라이다(`/scan`)를 결합해서, 검출된 객체까지의 거리를 계산하는 패키지.
방식이 다른 두 개의 퓨전 노드가 들어있고, 워크스페이스 전체를 한 번에 띄우는 launch 파일도 여기 있다.

## 노드

### `sensor_fusion_node` (권장 — 현재 마운트 계획에 맞는 방식)

라이다와 카메라가 **z(높이)만 다르고 x, y는 동일한 동축(coaxial) 위치**에 마운트된다는 가정 하에,
별도의 3D 외부파라미터(extrinsic) 계산 없이 카메라 픽셀 각도만으로 라이다 거리를 찾는 방식.

동작 순서: bbox 중심 픽셀의 x좌표 → 카메라 수평 화각(`camera_fov`) 기준 각도로 변환 →
그 각도(+ `lidar_front_offset_deg` 보정)에 해당하는 라이다 스캔 인덱스의 거리값을 읽음.

- 구독: `image_raw` (`sensor_msgs/Image`), `/scan` (`sensor_msgs/LaserScan`), `/detections` (`interfaces_pkg/DetectionArray`)
- `cv2.imshow`로 "Sensor Fusion" 창을 띄워 bbox + 거리 텍스트를 오버레이. 화면을 클릭하면 그 지점의 거리도 표시.
- **ROS 파라미터 (재빌드 없이 launch 인자로 조정 가능)**
  - `camera_fov`: 카메라 수평 화각(도). 현재 값 `59.5399`는 실측 캘리브레이션 fx=559.431712, width=640 기준
    (`2*atan(width/(2*fx))`)으로 산출. 카메라를 바꾸면 재캘리브레이션 후 갱신할 것.
  - `lidar_front_offset_deg`: 라이다가 인식하는 "정면"과 실제 카메라가 보는 방향이 다를 때 보정값(도).
    현재 `180.0` (라이다 0도 방향이 카메라 반대쪽을 보는 마운트 기준).

### `image_fusion_node` (대안 — 라이다·카메라가 떨어져 있을 때)

카메라 내부파라미터(fx, fy, cx, cy)와 라이다→카메라 외부파라미터(회전+변환)를 이용해 라이다 포인트를
이미지 평면에 3D 투영하는 방식. 동축 마운트가 아니거나 더 정밀한 매핑이 필요할 때 쓴다. 단, 체커보드
카메라 캘리브레이션과 정확한 extrinsic(위치 오프셋) 측정이 필요해서 `sensor_fusion_node`보다 설정이 까다롭다.

- 구독/발행 토픽 및 파라미터는 파일 상단 `declare_parameter` 참고 (`fx`, `fy`, `cx`, `cy`, `cam_x_offset`, `cam_height` 등)
- `fx`, `fy`, `cx`, `cy` 기본값은 동일한 실측 캘리브레이션 결과(559.431712, 568.785249, 302.888725, 329.408991)로
  갱신해뒀다. 단, 이 노드는 렌즈 왜곡(distortion) 보정은 적용하지 않으므로 이미지 가장자리 쪽 투영은 오차가 있을 수 있다.

## launch/fusion_bringup.launch.py

라이다 + 카메라 + YOLO + `sensor_fusion_node`를 한 번에 띄우는 통합 launch 파일.

```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py
```

인자: `serial_port`(기본 `/dev/ttyUSB0`), `serial_baudrate`(기본 `460800`, RPLIDAR C1 기준),
`frame_id`(기본 `laser`), `device`(YOLO 추론 디바이스, 기본 `cpu`), `camera_fov`(기본 `59.5399`),
`lidar_front_offset_deg`(기본 `180.0`).

예: 나중에 라이다 오프셋을 다시 재보정한 경우
```bash
ros2 launch lidar_camera_fusion_pkg fusion_bringup.launch.py lidar_front_offset_deg:=180.0 camera_fov:=59.5399
```
