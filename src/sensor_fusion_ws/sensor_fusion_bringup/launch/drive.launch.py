"""자율주행 전체 실행: 센서/인지(full_bringup) + 판단(decision_making) + 제어(serial).

체인:
  카메라 -> yolov8_node(cone/car_back/lane_seg) -> /detections
    -> lane_info_extractor_node  -> /yolov8_lane_info   (차선 중심점)
    -> image_fusion_node         -> /lidar_obstacle_info (가장 가까운 장애물 거리/픽셀x)
    -> path_planner_node(lattice) -> /path_planning_result
    -> motion_planner_node(pure pursuit + PD) -> /topic_control_signal (MotionCommand)
    -> serial_sender_node -> 아두이노("C,<조향 -1.0~1.0>,<후륜 PWM>")

모든 파라미터는 config/params.yaml에서 관리한다.
full_bringup.launch.py의 인자(cam_num, serial_port, device, ...)는 여기에도 그대로 먹는다.

예)
  ros2 launch sensor_fusion_bringup drive.launch.py cam_num:=1
  ros2 launch sensor_fusion_bringup drive.launch.py enable_serial:=false   # 바퀴 안 굴리고 확인만
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'),
        'config', 'params.yaml'
    )
    full_bringup_path = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'),
        'launch', 'full_bringup.launch.py'
    )

    # 아두이노로 실제 명령을 내보낼지 여부. false면 /topic_control_signal까지만 돌아서
    # (ros2 topic echo /topic_control_signal) 바퀴를 안 굴리고 조향/속도를 확인할 수 있다.
    enable_serial = LaunchConfiguration('enable_serial', default='true')
    # 판단 노드들을 센서/YOLO가 뜬 뒤에 올리기 위한 지연[s]
    # fusion_bringup.launch.py가 라이다(0s) -> 카메라(3s) -> YOLO 등(5s) 순으로 늦게
    # 띄우도록 바뀌어서, 그만큼 5.0 -> 10.0으로 같이 늦췄다.
    decision_start_delay = LaunchConfiguration('decision_start_delay', default='10.0')

    # 버드아이뷰(bird_eye_node) - 퓨전과 같은 카메라(/image_raw)를 보고 차선을 그린다.
    # 결과는 Fusion Visualizer의 4(bev)/5(bev_roi) 화면으로 들어간다.
    # GPU가 모자라면 enable_bird_eye:=false로 끄면 된다 (lane_seg 추론이 한 번 줄어듦).
    enable_bird_eye = LaunchConfiguration('enable_bird_eye', default='true')
    # 자체 미리보기 창('Lane bird-eye | original')은 Fusion Visualizer와 중복이라 기본 꺼둠
    bird_eye_preview = LaunchConfiguration('bird_eye_preview', default='false')

    decision_nodes = [
        # 차선 마스크 -> 주행 목표점
        Node(
            package='camera_perception_pkg',
            executable='lane_info_extractor_node',
            name='lane_info_extractor_node',
            output='screen',
            parameters=[config_file],
        ),

        # 목표점 + 장애물 -> lattice 경로
        Node(
            package='decision_making_pkg',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[config_file],
        ),

        # 경로 -> 조향/속도 명령
        Node(
            package='decision_making_pkg',
            executable='motion_planner_node',
            name='motion_planner_node',
            output='screen',
            parameters=[config_file],
        ),

        # 명령 -> 아두이노 시리얼
        Node(
            package='serial_communication_pkg',
            executable='serial_sender_node',
            name='serial_sender_node',
            output='screen',
            condition=IfCondition(enable_serial),
            parameters=[config_file],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_serial', default_value=enable_serial,
            description='아두이노로 실제 제어 명령을 보낼지 여부. false면 명령 토픽까지만 확인'),
        DeclareLaunchArgument(
            'decision_start_delay', default_value=decision_start_delay,
            description='센서/YOLO가 뜬 뒤 판단 노드를 올리기까지의 지연[s]'),
        DeclareLaunchArgument(
            'enable_bird_eye', default_value=enable_bird_eye,
            description='버드아이뷰(bird_eye_node) 실행 여부. GPU가 모자라면 false'),
        DeclareLaunchArgument(
            'bird_eye_preview', default_value=bird_eye_preview,
            description='bird_eye_node 자체 미리보기 창(원본+버드아이뷰) 표시 여부'),

        # 센서 + 인지 + 퓨전 + 버드아이뷰
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_bringup_path),
            launch_arguments={
                'enable_bird_eye': enable_bird_eye,
                'bird_eye_preview': bird_eye_preview,
            }.items(),
        ),

        TimerAction(period=decision_start_delay, actions=decision_nodes),
    ])
