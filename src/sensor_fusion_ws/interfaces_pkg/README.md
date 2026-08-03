# interfaces_pkg

카메라 인식(YOLO), 라이다-카메라 퓨전 노드들이 주고받는 커스텀 ROS2 메시지 정의 패키지.
`ament_cmake` + `rosidl_generate_interfaces`로 빌드되는 순수 메시지 패키지이며, 실행 파일은 없다.

## 이 워크스페이스에서 실제로 쓰는 메시지

| 메시지 | 용도 |
|---|---|
| `Point2D` | 픽셀 좌표 (x, y) |
| `Vector2` | 2D 크기 (x, y) — bbox의 width/height 등 |
| `Pose2D` | 픽셀 좌표 위치(`position`) + 회전(`theta`) |
| `BoundingBox2D` | 검출 박스: 중심(`center: Pose2D`) + 크기(`size: Vector2`) |
| `Mask` | 세그멘테이션 마스크 외곽선 (`Point2D[]`) + 원본 이미지 크기 |
| `KeyPoint2D` | 포즈 추정 키포인트 1개 (id, 픽셀 좌표, score) |
| `KeyPoint2DArray` | 키포인트 배열 |
| `Detection` | YOLO 검출 결과 1개 (class_id, class_name, score, bbox, mask, keypoints) |
| `DetectionArray` | 한 프레임의 전체 검출 결과 (`header` + `Detection[]`) |

`camera_perception_pkg`의 `yolov8_node`가 `DetectionArray`를 발행(`/detections`)하고,
`lidar_camera_fusion_pkg`의 퓨전 노드들이 이를 구독해서 라이다 거리와 결합한다.

## 같이 들어있지만 현재 미사용인 메시지

H-Mobility 워크스페이스의 `interfaces_pkg`를 통째로 가져오면서 딸려온 것들로, 차선 추종/모션 플래닝용이라
지금 이 워크스페이스의 어떤 노드도 발행/구독하지 않는다. 나중에 관련 기능을 추가할 때 참고용으로 남겨둔 것.

- `BoundingBox3D`, `KeyPoint3D`, `KeyPoint3DArray` — 3D(월드 좌표) 버전의 검출 결과
- `LaneInfo`, `TargetPoint` — 차선 인식 결과
- `MotionCommand` — 조향/모터 속도 명령
- `PathPlanningResult` — 경로 계획 결과
