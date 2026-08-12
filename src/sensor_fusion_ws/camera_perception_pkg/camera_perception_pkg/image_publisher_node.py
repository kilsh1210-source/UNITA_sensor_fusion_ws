import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

import sys
import cv2
import os

from camera_perception_pkg.camera_select import resolve_camera

#---------------Variable Setting---------------
# Publish할 토픽 이름
PUB_TOPIC_NAME = 'image_raw'

# 데이터 입력 소스: 'camera', 'image', 또는 'video' 중 택1하여 입력
DATA_SOURCE = 'camera'

# 카메라(웹캠) 장치 번호 (ls /dev/video* 명령을 터미널 창에 입력하여 확인)
# 주의: 카메라를 2대 이상 꽂으면 이 번호는 재부팅/재연결마다 바뀔 수 있다.
#       그럴 땐 아래 CAMERA_NAME 또는 CAMERA_DEVICE로 고정할 것.
CAM_NUM = 2

# 카메라를 번호 대신 "바뀌지 않는 이름/경로"로 고르기 (둘 다 비우면 CAM_NUM 사용).
# 연결된 카메라 목록과 이름을 보려면:
#     ros2 run camera_perception_pkg list_cameras --probe
#
# CAMERA_NAME: by-id 이름이나 카메라 이름의 일부 (대소문자 무시). 예: 'C920', 'HD_Pro'
#              -> 모델이 서로 다른 카메라 2대를 구분할 때 가장 편하다.
# CAMERA_DEVICE: 명시적 경로. 같은 모델 2대라면 포트 기준인 by-path를 쓸 것.
#              예: '/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_XXXX-video-index0'
CAMERA_NAME = ''
CAMERA_DEVICE = ''

# 이미지 데이터가 들어있는 디렉토리의 경로를 입력
IMAGE_DIRECTORY_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/sample_dataset'

# 비디오 데이터 파일의 경로를 입력
VIDEO_FILE_PATH = 'src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/driving_simulation.mp4'

# 화면에 publish하는 이미지를 띄울것인지 여부: True, 또는 False 중 택1하여 입력
# Fusion Visualizer 창이 이미 카메라 영상을 보여주므로 기본은 False (중복 imshow 오버헤드 제거)
SHOW_IMAGE = False

# 이미지 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.03

# 해상도. 캘리브레이션(fx/fy/cx/cy)을 뽑을 때 쓴 해상도와 반드시 같아야 한다.
# 다르면 라이다 투영이 그 비율만큼 어긋난다.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- 오토포커스/자동노출 고정 ---
# USB 웹캠은 기본적으로 오토포커스(AF)와 자동노출(AE)이 켜져 있다. 정지 장면을 계속
# 비추고 있으면 AF가 초점을 찾아 헤매다가 먼 배경에 락이 걸리고, 그러면 가까운 콘은
# 흐릿해져서 YOLO가 놓치고 대신 선명해진 먼 배경 쪽에서 오탐이 난다.
# 손바닥으로 렌즈를 가렸다 떼면 AF/AE가 재수렴하면서 가까운 콘에 초점이 잡히기 때문에
# 그 순간에만 콘이 잘 잡히는 것. 그래서 아래 값들로 AF/AE를 꺼서 고정한다.
AUTOFOCUS = False       # False면 오토포커스 끔 (권장)
FOCUS = -1              # 0~255 정도의 절대 초점값. -1이면 건드리지 않음
AUTO_EXPOSURE = False   # False면 수동 노출 (V4L2에서 1=수동, 3=자동)
EXPOSURE = -1           # 노출값(exposure_time_absolute). -1이면 건드리지 않음
AUTO_WHITE_BALANCE = False
WHITE_BALANCE = -1      # 색온도(K). -1이면 건드리지 않음
USE_MJPG = True         # MJPG로 받으면 같은 대역폭에서 프레임률/지연이 유리
#----------------------------------------------

class ImagePublisherNode(Node):
    def __init__(self, data_source=DATA_SOURCE, cam_num=CAM_NUM, img_dir=IMAGE_DIRECTORY_PATH, video_path=VIDEO_FILE_PATH, pub_topic=PUB_TOPIC_NAME, logger=SHOW_IMAGE, timer=TIMER):
        super().__init__('image_publisher_node')
        self.declare_parameter('data_source', data_source)
        self.declare_parameter('cam_num', cam_num)
        self.declare_parameter('img_dir', img_dir)
        self.declare_parameter('video_path', video_path)
        self.declare_parameter('pub_topic', pub_topic)
        self.declare_parameter('logger', logger)
        self.declare_parameter('timer', timer)

        self.declare_parameter('camera_name', CAMERA_NAME)
        self.declare_parameter('camera_device', CAMERA_DEVICE)
        self.declare_parameter('frame_width', FRAME_WIDTH)
        self.declare_parameter('frame_height', FRAME_HEIGHT)
        self.declare_parameter('autofocus', AUTOFOCUS)
        self.declare_parameter('focus', FOCUS)
        self.declare_parameter('auto_exposure', AUTO_EXPOSURE)
        self.declare_parameter('exposure', EXPOSURE)
        self.declare_parameter('auto_white_balance', AUTO_WHITE_BALANCE)
        self.declare_parameter('white_balance', WHITE_BALANCE)
        self.declare_parameter('use_mjpg', USE_MJPG)

        self.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        self.cam_num = self.get_parameter('cam_num').get_parameter_value().integer_value
        self.img_dir = self.get_parameter('img_dir').get_parameter_value().string_value
        self.video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.logger = self.get_parameter('logger').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer').get_parameter_value().double_value

        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.autofocus = self.get_parameter('autofocus').get_parameter_value().bool_value
        self.focus = self.get_parameter('focus').get_parameter_value().integer_value
        self.auto_exposure = self.get_parameter('auto_exposure').get_parameter_value().bool_value
        self.exposure = self.get_parameter('exposure').get_parameter_value().integer_value
        self.auto_white_balance = self.get_parameter(
            'auto_white_balance').get_parameter_value().bool_value
        self.white_balance = self.get_parameter('white_balance').get_parameter_value().integer_value
        self.use_mjpg = self.get_parameter('use_mjpg').get_parameter_value().bool_value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.br = CvBridge()
        
        if self.data_source == 'camera':
            # 카메라가 2대 이상이면 /dev/video 번호가 부팅마다 바뀌므로,
            # camera_device / camera_name 으로 "쓸 카메라 한 대"를 고정할 수 있다.
            target, desc, warns = resolve_camera(
                device=self.camera_device, name=self.camera_name, index=self.cam_num)
            for w in warns:
                self.get_logger().warn(w)

            self.camera_target = target
            self.camera_desc = desc
            self.get_logger().info(f'카메라 선택: {desc}')

            if isinstance(target, str):
                self.cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
            else:
                self.cap = cv2.VideoCapture(target)

            if not self.cap.isOpened():
                self.get_logger().error(
                    f"카메라를 열지 못했습니다: {desc} - 다른 프로세스가 잡고 있거나"
                    f"(fuser /dev/video* 로 확인) 지정이 틀렸을 수 있습니다. "
                    f"연결된 카메라 확인: ros2 run camera_perception_pkg list_cameras --probe")
            self._configure_camera()
        elif self.data_source == 'video':
            self.cap = cv2.VideoCapture(self.video_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            if not self.cap.isOpened():
                self.get_logger().error('Cannot open video file: %s' % self.video_path)
                rclpy.shutdown()
                sys.exit(1)
        elif self.data_source == 'image':
            if os.path.isdir(self.img_dir):
                self.img_list = sorted(os.listdir(self.img_dir))
                self.img_num = 0
            else:
                self.get_logger().error('Not a directory file: %s' % self.img_dir)
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Wrong data source: %s \nCheck that the DATA_SOURCE variable is either 'camera', 'image', or 'video'." % self.data_source)
            rclpy.shutdown()
            sys.exit(1)
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _configure_camera(self):
        """해상도/AF/AE/WB를 명시적으로 고정한다.

        오토포커스와 자동노출이 켜져 있으면 장면을 가만히 두는 동안 계속 재수렴하면서
        초점이 먼 배경으로 옮겨가고, 그 결과 가까운 콘은 흐려져 놓치고 먼 배경에서
        엉뚱한 오탐이 생긴다. 손으로 렌즈를 가렸다 떼면 다시 수렴해서 잠깐 잘 잡히는
        것이 바로 그 증상이다. 여기서 전부 수동으로 고정해 그 흔들림을 없앤다.

        설정한 뒤 다시 읽어서 실제 반영 여부를 로그로 남긴다 - 웹캠에 따라 조용히
        무시하는 항목이 있어서, 안 먹으면 v4l2-ctl로 직접 잡아야 하기 때문.
        """
        cap = self.cap

        # MJPG를 먼저 잡아야 원하는 해상도/프레임률이 나오는 웹캠이 많다
        if self.use_mjpg:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        # 드라이버 버퍼를 1로 줄여 지연을 낮춘다. 지연이 크면 라이다 점과 영상의
        # 시간이 어긋나서, 차가 움직일 때 퓨전 오버레이가 밀려 보인다.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- 오토포커스 ---
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if self.autofocus else 0)
        if not self.autofocus and self.focus >= 0:
            cap.set(cv2.CAP_PROP_FOCUS, float(self.focus))

        # --- 자동노출 ---
        # V4L2(UVC)에서 CAP_PROP_AUTO_EXPOSURE는 bool이 아니다: 1=수동, 3=자동.
        # 노출값은 반드시 "수동으로 바꾼 뒤에" 설정해야 먹는다.
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0 if self.auto_exposure else 1.0)
        if not self.auto_exposure and self.exposure >= 0:
            cap.set(cv2.CAP_PROP_EXPOSURE, float(self.exposure))

        # --- 화이트밸런스 ---
        cap.set(cv2.CAP_PROP_AUTO_WB, 1.0 if self.auto_white_balance else 0.0)
        if not self.auto_white_balance and self.white_balance >= 0:
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(self.white_balance))

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            "camera %d: %dx%d (요청 %dx%d) fps=%.1f  autofocus=%.0f focus=%.0f "
            "auto_exposure=%.0f(1=수동,3=자동) exposure=%.0f auto_wb=%.0f" % (
                self.cam_num, actual_w, actual_h, self.frame_width, self.frame_height,
                cap.get(cv2.CAP_PROP_FPS),
                cap.get(cv2.CAP_PROP_AUTOFOCUS), cap.get(cv2.CAP_PROP_FOCUS),
                cap.get(cv2.CAP_PROP_AUTO_EXPOSURE), cap.get(cv2.CAP_PROP_EXPOSURE),
                cap.get(cv2.CAP_PROP_AUTO_WB)))

        if (actual_w, actual_h) != (self.frame_width, self.frame_height):
            self.get_logger().warn(
                "카메라가 요청 해상도를 그대로 주지 않음(%dx%d -> %dx%d). resize로 맞추긴 하지만, "
                "가로세로 비율이 다르면 캘리브레이션(fx/fy/cx/cy)이 안 맞아 라이다 투영이 어긋난다."
                % (self.frame_width, self.frame_height, actual_w, actual_h))

        if not self.autofocus and cap.get(cv2.CAP_PROP_AUTOFOCUS) != 0:
            self.get_logger().warn(
                "오토포커스가 꺼지지 않음. 터미널에서 직접 고정할 것: "
                "v4l2-ctl -d /dev/video%d -c focus_automatic_continuous=0 -c focus_absolute=<값>"
                % self.cam_num)

    def timer_callback(self):
        if self.data_source == 'camera':
            ret, frame = self.cap.read()
            if ret:
                if frame.shape[1] != self.frame_width or frame.shape[0] != self.frame_height:
                    frame = cv2.resize(frame, (self.frame_width, self.frame_height))
                image_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                if self.logger:
                    cv2.imshow('Camera Image', frame)
                    cv2.waitKey(1)
        elif self.data_source == 'image':
            while self.img_num < len(self.img_list):
                img_file = self.img_list[self.img_num]
                img_path = os.path.join(self.img_dir, img_file)
                img = cv2.imread(img_path)
                if img is None:
                    self.get_logger().warn('Skipping non-image file: %s' % img_file)
                else:
                    img = cv2.resize(img, (self.frame_width, self.frame_height))
                    image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
                    image_msg.header = Header()
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    image_msg.header.frame_id = 'image_frame'
                    self.publisher.publish(image_msg)
                    if self.logger:
                        self.get_logger().info('Published image: %s' % img_file)
                        cv2.imshow('Saved Image', img)
                        cv2.waitKey(1)
                
                self.img_num += 1
                break
            else:
                self.img_num = 0
        elif self.data_source == 'video':
            ret, img = self.cap.read()
            if ret:
                img = cv2.resize(img, (self.frame_width, self.frame_height))
                image_msg = self.br.cv2_to_imgmsg(img, encoding='bgr8')
                image_msg.header = Header()
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = 'image_frame'
                self.publisher.publish(image_msg)
                if self.logger:
                    cv2.imshow('Video Frame', img)
                    cv2.waitKey(1)
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset video to the first frame
    
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
        pass
    node.destroy_node()
    # data_source='image'일 때는 cap 자체가 없어서 예전 코드는 종료 시 AttributeError가 났다
    cap = getattr(node, 'cap', None)
    if cap is not None and cap.isOpened():
        cap.release()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()
