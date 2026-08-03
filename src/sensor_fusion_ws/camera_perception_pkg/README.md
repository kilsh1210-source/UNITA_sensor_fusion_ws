# camera_perception_pkg

카메라 입력을 발행하고, YOLOv8로 객체를 검출하는 패키지.

## 노드

### `image_publisher_node`

OpenCV(`cv2.VideoCapture`)로 카메라(혹은 이미지/동영상 파일)를 읽어 `sensor_msgs/Image`로 발행한다.
usb_cam 드라이버 패키지 없이 웹캠을 바로 쏠 수 있어서, 이 워크스페이스의 실제 카메라 입력원으로 쓴다.

- 발행 토픽: `image_raw` (기본값, 파라미터로 변경 가능)
- 파라미터
  - `data_source`: `camera` / `image` / `video` 중 택1 (기본 `camera`)
  - `cam_num`: 카메라 장치 번호, `ls /dev/video*`로 확인 (기본 `0`)
  - `img_dir`, `video_path`: `data_source`가 `image`/`video`일 때 쓰는 경로
  - `pub_topic`: 발행 토픽 이름 (기본 `image_raw`)
  - `logger`: `True`면 `cv2.imshow`로 화면에 미리보기 (기본 `True`)
  - `timer`: 발행 주기(초) (기본 `0.03` ≈ 33Hz)

### `yolov8_node`

`ultralytics` YOLOv8 모델로 `image_raw`를 받아 추론하고, 결과를 `interfaces_pkg/DetectionArray`로 발행하는
lifecycle 노드. bbox, segmentation mask, pose keypoint를 모두 지원(모델 종류에 따라).

- 구독 토픽: `image_raw`
- 발행 토픽: `detections` (`interfaces_pkg/DetectionArray`)
- 서비스: `enable` (`std_srvs/SetBool`) — 런타임에 추론 on/off
- 파라미터
  - `model`: 가중치(.pt) 경로. 기본값은 파일명만(`best.pt`)이라, 실제 실행 시에는 launch 파일에서
    `models/best.pt`의 절대경로를 넣어줘야 한다 (`fusion_bringup.launch.py` 참고).
  - `device`: `cpu` 또는 `cuda:0`
  - `threshold`: confidence threshold (기본 `0.5`)
  - `enable`: 시작 시 추론 활성화 여부 (기본 `True`)

## models/

`best.pt` — 학습된 YOLOv8 가중치 파일. `setup.py`에서 `share/camera_perception_pkg/models/`로 설치된다.
