from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    front_angle_deg = LaunchConfiguration('front_angle_deg', default='180.0')
    fov_deg = LaunchConfiguration('fov_deg', default='150.0')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')

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
            'front_angle_deg', default_value=front_angle_deg,
            description='라이다 자체 좌표계에서 실제 전방에 해당하는 각도(라이다가 뒤집혀 장착된 경우 보정)'),
        DeclareLaunchArgument(
            'fov_deg', default_value=fov_deg,
            description='front_angle_deg를 중심으로 남길 전체 시야각(도)'),
        DeclareLaunchArgument(
            'launch_rviz', default_value=launch_rviz,
            description='l_shape_node가 시작될 때 rviz2를 자동으로 띄울지 여부'),

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

        Node(
            package='lidar_cluster_pkg',
            executable='l_shape_node',
            name='l_shape_node',
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'front_angle_deg': front_angle_deg,
                'fov_deg': fov_deg,
                'launch_rviz': launch_rviz,
            }],
        ),
    ])
