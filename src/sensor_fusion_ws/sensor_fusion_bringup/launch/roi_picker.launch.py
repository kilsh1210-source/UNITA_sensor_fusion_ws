"""Interactive picker for the surround-view ground ROI (src_points/dst_points).

왼쪽 패널의 카메라 영상과 오른쪽 패널의 top-down 캔버스에서 바닥의 같은 지점을 같은
순서로 클릭하면, 그 자리에서 워핑 결과가 캔버스에 겹쳐 보인다. 세 카메라 모두 같은
바닥 마커를 찍으면 서로 포개진다. s키가 params.yaml의 해당 줄을 직접 갱신한다.

rear_surround_view.launch.py와 같은 이유로 카메라는 기본적으로 안 켠다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERAS = (
    ('camera_2_node', 0.0),    # 후방
    ('camera_3_node', 2.0),    # 좌측
    ('camera_4_node', 4.0),    # 우측
)


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'), 'config', 'params.yaml')
    start_cameras = LaunchConfiguration('start_cameras')

    actions = [
        DeclareLaunchArgument(
            'start_cameras', default_value='false',
            description='카메라 퍼블리셔도 같이 띄울지 여부. 이미 떠 있으면 false로 둘 것'),
    ]

    for node_name, delay in CAMERAS:
        camera = Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name=node_name, output='screen', condition=IfCondition(start_cameras),
            parameters=[config, {'logger': False}],
        )
        actions.append(camera if delay == 0.0
                       else TimerAction(period=delay, actions=[camera],
                                        condition=IfCondition(start_cameras)))

    actions.append(Node(
        package='camera_perception_pkg', executable='roi_picker_node',
        name='roi_picker_node', output='screen', emulate_tty=True,
        parameters=[config],
    ))

    return LaunchDescription(actions)
