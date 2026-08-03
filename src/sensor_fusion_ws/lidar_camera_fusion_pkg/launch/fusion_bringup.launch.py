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
    fx = LaunchConfiguration('fx', default='559.431712')
    cx = LaunchConfiguration('cx', default='302.888725')
    lidar_front_offset_deg = LaunchConfiguration('lidar_front_offset_deg', default='-180.0')
    cam_num = LaunchConfiguration('cam_num', default='0')

    model_path = os.path.join(
        get_package_share_directory('camera_perception_pkg'),
        'models', 'best.pt'
    )

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
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen',
            parameters=[{
                'fx': fx,
                'cx': cx,
                'lidar_front_offset_deg': lidar_front_offset_deg,
            }],
        ),
    ])
