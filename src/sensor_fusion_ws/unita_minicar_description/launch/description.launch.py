import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 카메라가 지면 쪽으로 내려다보는 각도(도, 실측값). URDF의 camera_joint는 rpy가 전부 0이라
# 실제 다운틸트가 반영돼 있지 않음. URDF 파일 자체는 건드리지 않고, camera_link 아래에
# 이 각도만큼 기운 자식 프레임(camera_link_tilted)을 별도 static TF로 추가해서 보정한다.
CAMERA_PITCH_DEG = 10.0


def generate_launch_description():
    pkg_share = get_package_share_directory('unita_minicar_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'unita_minicar.urdf')

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
            '--roll', '0', '--pitch', str(math.radians(CAMERA_PITCH_DEG)), '--yaw', '0',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_link_tilted',
        ],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        camera_tilt_tf_node,
    ])
