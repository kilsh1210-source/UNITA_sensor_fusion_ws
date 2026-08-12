"""라이다-카메라 정렬 캘리브레이션 도구 실행.

주행용 전체 파이프라인(full_bringup)과 달리 YOLO/버드아이뷰를 띄우지 않는다.
정렬만 볼 거라서 GPU를 놀려둘 이유가 없고, 화면도 창 하나만 뜬다.

    ros2 launch lidar_camera_fusion_pkg calibration.launch.py

창에서 i/k(위아래), j/l(좌우)로 라이다 점을 실제 물체에 맞춘 뒤 s키로 저장.
저장 후에는 sensor_fusion_bringup을 다시 빌드해야 주행 파이프라인에 반영된다:

    colcon build --packages-select sensor_fusion_bringup
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


_FALLBACK = {
    'rplidar_node': {'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 460800,
                     'frame_id': 'laser'},
    'image_publisher_node': {'cam_num': 0},
    'image_fusion_node': {'cam_pitch_deg': 14.0},
}


def _load_params():
    try:
        config_file = os.path.join(
            get_package_share_directory('sensor_fusion_bringup'), 'config', 'params.yaml')
        with open(config_file) as f:
            raw = yaml.safe_load(f)
        return config_file, {k: v['ros__parameters']
                             for k, v in raw.items() if 'ros__parameters' in v}
    except Exception:
        return None, _FALLBACK


def generate_launch_description():
    config_file, params = _load_params()

    def p(node, key, default=''):
        return str(params.get(node, {}).get(key, _FALLBACK.get(node, {}).get(key, default)))

    # params.yaml을 그대로 넘겨서, image_fusion_node용 내부/외부 파라미터를
    # 캘리브레이션 노드도 똑같이 쓰게 한다 (두 화면이 어긋나면 의미가 없으므로).
    fusion_params = dict(params.get('image_fusion_node', {}))
    # 노드 이름이 달라서 yaml 자동 매칭이 안 되므로 dict로 직접 넘긴다
    calib_params = {k: v for k, v in fusion_params.items()
                    if k not in ('display_mode', 'draw_all_points', 'distance_tolerance',
                                 'det_clear_after_empty', 'show_calib_hud')}

    serial_port = LaunchConfiguration('serial_port', default=p('rplidar_node', 'serial_port'))
    serial_baudrate = LaunchConfiguration(
        'serial_baudrate', default=p('rplidar_node', 'serial_baudrate'))
    frame_id = LaunchConfiguration('frame_id', default=p('rplidar_node', 'frame_id'))
    cam_num = LaunchConfiguration('cam_num', default=p('image_publisher_node', 'cam_num'))
    camera_pitch_deg = LaunchConfiguration(
        'camera_pitch_deg', default=p('image_fusion_node', 'cam_pitch_deg', '14.0'))
    params_file = LaunchConfiguration('params_file', default='')

    description_launch = os.path.join(
        get_package_share_directory('unita_minicar_description'), 'launch',
        'description.launch.py')

    base = [config_file] if config_file else []

    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        DeclareLaunchArgument('cam_num', default_value=cam_num),
        DeclareLaunchArgument(
            'camera_pitch_deg', default_value=camera_pitch_deg,
            description='카메라 다운틸트(도). 이 값에서 출발해 화면에서 잔차를 맞춘다'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='저장할 params.yaml 경로. 비우면 소스 트리에서 자동 탐색'),

        # URDF TF (base_link -> laser / camera_optical_frame_tilted)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'camera_pitch_deg': camera_pitch_deg}.items(),
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=base + [{
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
            package='camera_perception_pkg',
            executable='image_publisher_node',
            name='image_publisher_node',
            output='screen',
            parameters=base + [{
                'cam_num': cam_num,
                'logger': False,
            }],
        ),

        Node(
            package='lidar_camera_fusion_pkg',
            executable='calibration_node',
            name='lidar_camera_calibration_node',
            output='screen',
            parameters=[calib_params, {
                'cam_pitch_deg': camera_pitch_deg,
                'params_file': params_file,
            }],
        ),
    ])
