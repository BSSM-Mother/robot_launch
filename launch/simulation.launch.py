import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    robot_description_pkg = get_package_share_directory('robot_description')
    
    # URDF 파일 (xacro로 변환)
    robot_urdf_xacro = os.path.join(robot_description_pkg, 'urdf', 'cylinder_robot.urdf.xacro')
    
    # xacro 명령어로 URDF 생성
    robot_description = Command(['xacro ', robot_urdf_xacro])
    
    return LaunchDescription([
        # 카메라 시뮬레이터
        Node(
            package='robot_perception',
            executable='camera_simulator',
            output='screen',
        ),
        
        # 로봇 상태 발행자
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
            ]
        ),
        
        # 추적 제어 노드
        Node(
            package='robot_control',
            executable='tracking_controller',
            output='screen',
        ),
        
        # 바퀴 제어 노드
        Node(
            package='robot_base',
            executable='wheel_controller',
            output='screen',
        ),
        
        # 사람 감지 노드 (YOLO)
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
        ),
    ])
