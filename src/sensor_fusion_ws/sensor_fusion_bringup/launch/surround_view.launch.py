"""Launch two camera publishers and the four-camera-ready surround view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'), 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_side_cameras', default_value='false',
            description='Start left/right publishers (also enable them in params.yaml)'),
        Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name='surround_front_camera', output='screen',
            parameters=[config, {
                'camera_device': '', 'cam_num': 0,
                'pub_topic': '/front/image_raw', 'logger': False,
            }],
        ),
        TimerAction(period=1.0, actions=[
            Node(
                package='camera_perception_pkg', executable='image_publisher_node',
                name='surround_rear_camera', output='screen',
                parameters=[config, {
                    'camera_device': '', 'cam_num': 2,
                    'pub_topic': '/rear/image_raw', 'logger': False,
                }],
            ),
        ]),
        TimerAction(period=2.0, actions=[
            Node(
                package='camera_perception_pkg', executable='image_publisher_node',
                name='surround_left_camera', output='screen',
                condition=IfCondition(LaunchConfiguration('enable_side_cameras')),
                parameters=[config, {
                    'camera_device': '', 'cam_num': 4,
                    'pub_topic': '/left/image_raw', 'logger': False,
                }],
            ),
        ]),
        TimerAction(period=3.0, actions=[
            Node(
                package='camera_perception_pkg', executable='image_publisher_node',
                name='surround_right_camera', output='screen',
                condition=IfCondition(LaunchConfiguration('enable_side_cameras')),
                parameters=[config, {
                    'camera_device': '', 'cam_num': 6,
                    'pub_topic': '/right/image_raw', 'logger': False,
                }],
            ),
        ]),
        TimerAction(period=4.0, actions=[
            Node(
                package='camera_perception_pkg', executable='surround_view_node',
                name='surround_view_node', output='screen', parameters=[config],
            ),
        ]),
    ])
