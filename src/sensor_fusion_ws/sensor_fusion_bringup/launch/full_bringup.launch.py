import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 모든 기본값은 config/params.yaml 하나로 관리. 값을 바꾸려면 그 파일을 수정할 것
    # (launch 인자로 임시 override도 여전히 가능).
    config_file = os.path.join(
        get_package_share_directory('sensor_fusion_bringup'),
        'config', 'params.yaml'
    )
    with open(config_file) as f:
        params = yaml.safe_load(f)

    rplidar_p = params['rplidar_node']['ros__parameters']
    cam_p = params['image_publisher_node']['ros__parameters']
    yolo_p = params['yolov8_node']['ros__parameters']
    fusion_p = params['image_fusion_node']['ros__parameters']
    l_shape_p = params['l_shape_node']['ros__parameters']

    # --- fusion_bringup.launch.py 인자 (그대로 전달) ---
    serial_port = LaunchConfiguration('serial_port', default=str(rplidar_p['serial_port']))
    serial_baudrate = LaunchConfiguration('serial_baudrate', default=str(rplidar_p['serial_baudrate']))
    frame_id = LaunchConfiguration('frame_id', default=str(rplidar_p['frame_id']))
    device = LaunchConfiguration('device', default=str(yolo_p['device']))
    fx = LaunchConfiguration('fx', default=str(fusion_p['fx']))
    cx = LaunchConfiguration('cx', default=str(fusion_p['cx']))
    lidar_front_offset_deg = LaunchConfiguration(
        'lidar_front_offset_deg', default=str(fusion_p['front_angle_deg']))
    cam_num = LaunchConfiguration('cam_num', default=str(cam_p['cam_num']))
    show_split_view = LaunchConfiguration(
        'show_split_view', default=str(fusion_p['show_split_view']).lower())
    distance_tolerance = LaunchConfiguration(
        'distance_tolerance', default=str(fusion_p['distance_tolerance']))
    draw_all_points = LaunchConfiguration(
        'draw_all_points', default=str(fusion_p['draw_all_points']).lower())
    use_urdf_extrinsic = LaunchConfiguration(
        'use_urdf_extrinsic', default=str(fusion_p['use_urdf_extrinsic']).lower())
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default=str(fusion_p['lidar_frame_id']))
    camera_frame_id = LaunchConfiguration('camera_frame_id', default=str(fusion_p['camera_frame_id']))

    # --- l_shape_node 전용 인자 ---
    fov_deg = LaunchConfiguration('fov_deg', default=str(l_shape_p['fov_deg']))
    launch_rviz = LaunchConfiguration('launch_rviz', default=str(l_shape_p['launch_rviz']).lower())

    fusion_launch_path = os.path.join(
        get_package_share_directory('lidar_camera_fusion_pkg'), 'launch', 'fusion_bringup.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                               description='RPLIDAR USB serial port'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
                               description='RPLIDAR serial baudrate (C1: 460800)'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                               description='RPLIDAR scan frame_id'),
        DeclareLaunchArgument('device', default_value=device,
                               description='YOLO inference device (cpu / cuda:0)'),
        DeclareLaunchArgument('fx', default_value=fx,
                               description='카메라 초점거리 fx(px), 캘리브레이션 결과값'),
        DeclareLaunchArgument('cx', default_value=cx,
                               description='카메라 광학 중심 cx(px), 캘리브레이션 결과값'),
        DeclareLaunchArgument('lidar_front_offset_deg', default_value=lidar_front_offset_deg,
                               description='LiDAR 0-angle vs camera forward direction offset in degrees '
                                           '(정반대 마운트면 180). l_shape_node의 front_angle_deg에도 '
                                           '동일하게 적용됨'),
        DeclareLaunchArgument('cam_num', default_value=cam_num,
                               description='카메라 장치 번호 (ls /dev/video* 로 확인)'),
        DeclareLaunchArgument('show_split_view', default_value=show_split_view,
                               description='전체 클라우드와 bbox-겹침 뷰를 함께 보여줄지 여부'),
        DeclareLaunchArgument('draw_all_points', default_value=draw_all_points,
                               description='카메라 위에 라이다 포인트를 전부 그릴지 여부'),
        DeclareLaunchArgument('distance_tolerance', default_value=distance_tolerance,
                               description='bbox 거리 계산 시 허용할 거리 오차 범위 [m]'),
        DeclareLaunchArgument('use_urdf_extrinsic', default_value=use_urdf_extrinsic,
                               description='URDF/TF 기반 외인척 변환을 사용할지 여부'),
        DeclareLaunchArgument('lidar_frame_id', default_value=lidar_frame_id,
                               description='LiDAR frame id'),
        DeclareLaunchArgument('camera_frame_id', default_value=camera_frame_id,
                               description='Camera frame id'),
        DeclareLaunchArgument('fov_deg', default_value=fov_deg,
                               description='l_shape_node: front_angle_deg를 중심으로 남길 전체 시야각(도)'),
        DeclareLaunchArgument('launch_rviz', default_value=launch_rviz,
                               description='l_shape_node가 시작될 때 rviz2를 자동으로 띄울지 여부'),

        # 라이다 드라이버 + 카메라 + YOLO + 퓨전 (rplidar_node는 여기서 한 번만 실행됨)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch_path),
            launch_arguments={
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'device': device,
                'fx': fx,
                'cx': cx,
                'lidar_front_offset_deg': lidar_front_offset_deg,
                'cam_num': cam_num,
                'show_split_view': show_split_view,
                'distance_tolerance': distance_tolerance,
                'draw_all_points': draw_all_points,
                'use_urdf_extrinsic': use_urdf_extrinsic,
                'lidar_frame_id': lidar_frame_id,
                'camera_frame_id': camera_frame_id,
            }.items(),
        ),

        # L-shape fitting (같은 /scan을 구독만 하므로 별도 rplidar_node 없음)
        # config/params.yaml 전체를 로드하고, launch 인자로만 일부 값을 override
        Node(
            package='lidar_cluster_pkg',
            executable='l_shape_node',
            name='l_shape_node',
            output='screen',
            parameters=[config_file, {
                'frame_id': frame_id,
                'front_angle_deg': lidar_front_offset_deg,
                'fov_deg': fov_deg,
                'launch_rviz': launch_rviz,
            }],
        ),
    ])
