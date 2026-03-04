import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # 패키지 경로
    pkg_robot_description = get_package_share_directory('robot_description')
    world_file = os.path.join(pkg_robot_description, 'worlds', 'simple_world.sdf')

    # YOLO 모델 경로 (절대 경로로 지정하여 작업 디렉터리 문제 방지)
    # get_package_prefix → install/robot_launch  →  ../.. == workspace root
    ws_root = os.path.dirname(os.path.dirname(get_package_prefix('robot_launch')))
    yolo_model_path = os.path.join(ws_root, 'yolov8n_ncnn_model')
    if not os.path.isdir(yolo_model_path):
        yolo_model_path = os.path.join(ws_root, 'yolo11n_ncnn_model')
    if not os.path.isdir(yolo_model_path):
        yolo_model_path = os.path.join(ws_root, 'yolov8n.pt')
    if not os.path.isfile(yolo_model_path):
        yolo_model_path = os.path.join(ws_root, 'yolo11n.pt')

    # Gazebo Sim 실행
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # Gazebo -> ROS2 브릿지 (카메라 이미지)
    # Use YAML config to force one-way bridges only
    pkg_robot_launch = get_package_share_directory('robot_launch')
    bridge_cfg = os.path.join(pkg_robot_launch, 'config', 'bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_cfg}'],
        remappings=[
            ('/camera', '/camera/image_raw'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo Sim 실행
        gazebo,

        # ROS2 브릿지
        bridge,

        # 사람 감지 노드 (YOLO)
        Node(
            package='robot_perception',
            executable='person_detector',
            parameters=[{
                'model_path': yolo_model_path,
                'force_ncnn': False,
            }],
            output='screen',
        ),

        # 추적 컨트롤러 (cmd_vel 발행)
        Node(
            package='robot_control',
            executable='tracking_controller',
            output='screen',
        ),

        # 사람 프록시에 느린 원운동 명령 발행
        Node(
            package='robot_control',
            executable='person_mover',
            output='screen',
        ),

        # 공 랜덤 움직임 (로봇이 따라다닐 대상)
        Node(
            package='robot_control',
            executable='ball_randomizer',
            output='screen',
        ),

        # 이미지 뷰어 (감지 결과)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/camera/annotated'],
            output='screen',
        ),
    ])
