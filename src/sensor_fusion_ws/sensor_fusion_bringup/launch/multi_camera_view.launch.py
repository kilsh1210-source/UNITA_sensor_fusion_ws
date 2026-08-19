"""Start cameras 2/3/4 and show all three in one tiled window.

1번 카메라(주행용, YOLO)는 건드리지 않는다 - 이 런치는 나머지 3대 확인용이다.

기동을 2초씩 벌려둔 이유: 카메라를 동시에 열면 cv2.VideoCapture()가 멈추는 증상이
있었다(fusion_bringup.launch.py에도 같은 이유로 지연이 들어가 있다). USB 전원이
빠듯한 구성이라 동시에 켜면 열거 자체가 깨지기도 해서 순차 기동이 더 안전하다.

각 노드 이름을 params.yaml의 섹션 이름(camera_2_node 등)과 맞춰야 by-path
camera_device와 캘리브레이션 값이 로드된다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAMERAS = (
    # (노드 이름, 기동 지연(초))
    ('camera_2_node', 0.0),
    ('camera_3_node', 2.0),
    ('camera_4_node', 4.0),
)


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'), 'config', 'params.yaml')

    actions = [
        DeclareLaunchArgument(
            'show_preview', default_value='true',
            description='타일 창을 띄울지 여부 (false면 /multi_view/image 발행만)'),
        DeclareLaunchArgument(
            'tile_width', default_value='480',
            description='타일 하나의 가로 크기 (3개 가로 배치라 화면 폭에 맞춰 줄임)'),
        DeclareLaunchArgument(
            'tile_height', default_value='360', description='타일 하나의 세로 크기'),
        DeclareLaunchArgument(
            'resizable_window', default_value='true',
            description='창을 마우스로 늘였다 줄일 수 있게 할지 여부'),
    ]

    for node_name, delay in CAMERAS:
        # logger는 항상 false - 개별 창이 뜨면 타일 창과 중복이고 imshow 부담만 늘어난다.
        camera = Node(
            package='camera_perception_pkg', executable='image_publisher_node',
            name=node_name, output='screen',
            parameters=[config, {'logger': False}],
        )
        actions.append(camera if delay == 0.0
                       else TimerAction(period=delay, actions=[camera]))

    actions.append(TimerAction(period=6.0, actions=[
        Node(
            package='camera_perception_pkg', executable='multi_view_node',
            name='multi_view_node', output='screen',
            # LaunchConfiguration은 그냥 넘기면 문자열이라 노드의 int/bool 파라미터와
            # 타입이 안 맞는다. ParameterValue로 타입을 명시해야 한다.
            parameters=[config, {
                'show_preview': ParameterValue(
                    LaunchConfiguration('show_preview'), value_type=bool),
                'tile_width': ParameterValue(
                    LaunchConfiguration('tile_width'), value_type=int),
                'tile_height': ParameterValue(
                    LaunchConfiguration('tile_height'), value_type=int),
                'resizable_window': ParameterValue(
                    LaunchConfiguration('resizable_window'), value_type=bool),
            }],
        ),
    ]))

    return LaunchDescription(actions)
