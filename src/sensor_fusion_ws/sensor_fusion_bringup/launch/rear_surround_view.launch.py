"""Rear-facing surround bird's-eye view from cameras 2 (rear), 3 (left), 4 (right).

카메라는 기본적으로 안 켠다(start_cameras:=false). multi_camera_view.launch.py로
이미 세 대를 띄워 놓은 상태에서 그 토픽에 붙는 게 보통이고, 여기서 또 열면
/dev/video*가 이미 잡혀 있어 cv2.VideoCapture가 실패하기 때문이다. 카메라가 안 떠
있는 상태에서 한 번에 다 띄우려면 start_cameras:=true.

카메라를 2초씩 벌려 켜는 이유는 multi_camera_view.launch.py와 같다 - 동시에 열면
VideoCapture가 멈추거나 열기 자체가 깨진다(USB 전원이 빠듯한 구성).

surround_view_node는 프레임이 없으면 그냥 발행을 안 할 뿐이라, 카메라보다 먼저 떠도
문제가 없다. 그래서 지연 없이 바로 띄운다.

투영 파라미터(src_points/dst_points)는 params.yaml의 surround_view_node 섹션에 있고,
roi_picker.launch.py로 찍어서 채운다. 안 맞춘 상태로 띄우면 화면은 나오지만 지면이
안 맞는 그림이 나온다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAMERAS = (
    # (params.yaml 섹션 이름, 기동 지연(초))
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
        DeclareLaunchArgument(
            'show_preview', default_value='true',
            description='OpenCV 창을 띄울지 여부 (false면 /surround_view/image 발행만)'),
        DeclareLaunchArgument(
            'draw_camera_outlines', default_value='false',
            description='카메라별 커버리지 윤곽선 표시 (정렬 확인용)'),
        DeclareLaunchArgument(
            'blend_mode', default_value='priority',
            description='priority(겹치면 후방 우선) / average(가중 평균)'),
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
        package='camera_perception_pkg', executable='surround_view_node',
        name='surround_view_node', output='screen',
        # LaunchConfiguration을 그냥 넘기면 문자열이라 노드의 bool 파라미터와 타입이
        # 안 맞는다. ParameterValue로 타입을 명시해야 한다.
        parameters=[config, {
            'show_preview': ParameterValue(
                LaunchConfiguration('show_preview'), value_type=bool),
            'draw_camera_outlines': ParameterValue(
                LaunchConfiguration('draw_camera_outlines'), value_type=bool),
            'blend_mode': ParameterValue(
                LaunchConfiguration('blend_mode'), value_type=str),
        }],
    ))

    return LaunchDescription(actions)
