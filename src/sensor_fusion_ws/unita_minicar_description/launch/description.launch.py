import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 카메라가 지면 쪽으로 내려다보는 각도(도, 실측값). URDF의 camera_joint는 rpy가 전부 0이라
# 실제 다운틸트가 반영돼 있지 않음. URDF 파일 자체는 건드리지 않고, camera_link 아래에
# 이 각도만큼 기운 자식 프레임(camera_link_tilted)을 별도 static TF로 추가해서 보정한다.
#
# 이 값은 손으로 잰 값이라 1~2도 오차가 흔한데, 화면상 1도 ≈ 10 px라서 그게 곧
# 라이다 투영이 "살짝 위/아래로 틀어져 보이는" 원인이 된다. 각도 자체를 다시 재는 대신,
# image_fusion_node의 calib_pitch_deg로 잔차만 걷어내는 쪽이 실용적이다
# (Fusion Visualizer 창에서 i/k 키로 맞춘 뒤 p키로 값 출력 -> params.yaml에 기록).
DEFAULT_CAMERA_PITCH_DEG = 14.0


def _nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory('unita_minicar_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'unita_minicar.urdf')

    camera_pitch_deg = float(LaunchConfiguration('camera_pitch_deg').perform(context))

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # base_link -> laser / camera_link 등 고정 조인트 TF를 발행.
    # 조향/구동 조인트는 joint_state_publisher가 없어도 fixed 조인트 TF에는 영향 없음.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # camera_link 좌표계(X:정면, Y:왼쪽, Z:위) 기준 Y축 양의 회전(pitch)이 정면(X)을
    # 아래로 기울인다. camera_link -> camera_link_tilted 로 CAMERA_PITCH_DEG만큼 회전만
    # 추가하고 위치는 그대로(0,0,0) 둔다.
    camera_tilt_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tilt_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', str(math.radians(camera_pitch_deg)), '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_link_tilted',
        ],
    )

    # camera_link_tilted(로봇 표준 좌표계: X정면/Y왼쪽/Z위)를 카메라 광학 좌표계
    # (X오른쪽/Y아래/Z정면=깊이)로 변환. URDF의 camera_optical_joint와 동일한 회전이지만
    # camera_link_tilted의 자식으로 붙여서, 핀홀 투영(image_fusion_node)이 틸트 보정과
    # 광학 축 변환을 모두 반영한 프레임을 쓸 수 있게 한다.
    camera_optical_tilted_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tilted_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
            '--frame-id', 'camera_link_tilted',
            '--child-frame-id', 'camera_optical_frame_tilted',
        ],
    )

    return [
        robot_state_publisher_node,
        camera_tilt_tf_node,
        camera_optical_tilted_tf_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_pitch_deg', default_value=str(DEFAULT_CAMERA_PITCH_DEG),
            description='카메라 다운틸트(도). 실측값이 바뀌면 소스를 고치지 말고 이 인자로 넘길 것'),
        OpaqueFunction(function=_nodes),
    ])
