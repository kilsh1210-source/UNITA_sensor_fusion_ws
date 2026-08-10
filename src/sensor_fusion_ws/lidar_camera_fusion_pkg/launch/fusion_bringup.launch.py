import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    device = LaunchConfiguration('device', default='cpu')
    fx = LaunchConfiguration('fx', default='565.529459')
    cx = LaunchConfiguration('cx', default='337.983746')
    lidar_front_offset_deg = LaunchConfiguration('lidar_front_offset_deg', default='-180.0')
    cam_num = LaunchConfiguration('cam_num', default='0')
    display_mode = LaunchConfiguration('display_mode', default='boxes')
    distance_tolerance = LaunchConfiguration('distance_tolerance', default='0.6')
    draw_all_points = LaunchConfiguration('draw_all_points', default='true')
    use_urdf_extrinsic = LaunchConfiguration('use_urdf_extrinsic', default='false')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera_optical_frame_tilted')

    # cone/drum 탐지 모델 + 차량 후면 탐지 모델을 함께 로딩 (콤마로 구분, yolov8_node가 각각 추론 후 결과를 합쳐서 발행)
    camera_models_dir = os.path.join(
        get_package_share_directory('camera_perception_pkg'), 'models'
    )
    model_path = ','.join([
        os.path.join(camera_models_dir, 'best_cone.pt'),
        os.path.join(camera_models_dir, 'car_back.pt'),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value=serial_port,
            description='RPLIDAR USB serial port'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value=serial_baudrate,
            description='RPLIDAR serial baudrate (C1: 460800)'),
        DeclareLaunchArgument(
            'frame_id', default_value=frame_id,
            description='RPLIDAR scan frame_id'),
        DeclareLaunchArgument(
            'device', default_value=device,
            description='YOLO inference device (cpu / cuda:0)'),
        DeclareLaunchArgument(
            'fx', default_value=fx,
            description='카메라 초점거리 fx(px), 캘리브레이션 결과값'),
        DeclareLaunchArgument(
            'cx', default_value=cx,
            description='카메라 광학 중심 cx(px), 캘리브레이션 결과값'),
        DeclareLaunchArgument(
            'lidar_front_offset_deg', default_value=lidar_front_offset_deg,
            description='LiDAR 0-angle vs camera forward direction offset in degrees '
                        '(정반대 마운트면 180)'),
        DeclareLaunchArgument(
            'cam_num', default_value=cam_num,
            description='카메라 장치 번호 (ls /dev/video* 로 확인)'),
        DeclareLaunchArgument(
            'display_mode', default_value=display_mode,
            description='Fusion Visualizer 시작 화면 모드: raw / lidar / boxes / bev '
                        '(창에서 숫자키 1~4로 실행 중 전환 가능)'),
        DeclareLaunchArgument(
            'draw_all_points', default_value=draw_all_points,
            description='카메라 위에 라이다 포인트를 전부 그릴지 여부'),
        DeclareLaunchArgument(
            'distance_tolerance', default_value=distance_tolerance,
            description='bbox 거리 계산 시 허용할 거리 오차 범위 [m]'),
        DeclareLaunchArgument(
            'use_urdf_extrinsic', default_value=use_urdf_extrinsic,
            description='URDF/TF 기반 외인척 변환을 사용할지 여부'),
        DeclareLaunchArgument(
            'lidar_frame_id', default_value=lidar_frame_id,
            description='LiDAR frame id'),
        DeclareLaunchArgument(
            'camera_frame_id', default_value=camera_frame_id,
            description='Camera frame id'),

        # LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),

        # Camera
        Node(
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
            parameters=[{
                'cam_num': cam_num,
                'logger': False,
            }],
        ),

        # YOLO detection
        Node(
            package='camera_perception_pkg',
            executable='yolov8_node',
            name='yolov8_node',
            output='screen',
            parameters=[{
                'model': model_path,
                'device': device,
                'threshold': 0.5,
            }],
        ),

        # LiDAR-Camera fusion (distance overlay)
        Node(
            package='lidar_camera_fusion_pkg',
            executable='image_fusion_node',
            name='image_fusion_node',
            output='screen',
            parameters=[{
                'fx': fx,
                'cx': cx,
                'front_angle_deg': lidar_front_offset_deg,
                'display_mode': display_mode,
                'draw_all_points': draw_all_points,
                'distance_tolerance': distance_tolerance,
                'use_urdf_extrinsic': use_urdf_extrinsic,
                'lidar_frame_id': lidar_frame_id,
                'camera_frame_id': camera_frame_id,
                'display': True,
                'publish_annotated': False,
            }],
        ),
    ])
