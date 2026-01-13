import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    robot_description_pkg = get_package_share_directory('robot_description')
    
    # URDF 파일 (xacro로 변환)
    robot_urdf_xacro = os.path.join(robot_description_pkg, 'urdf', 'cylinder_robot.urdf.xacro')
    
    # 절대 경로로 월드 파일 지정 (소스 디렉토리)
    world_file = '/home/user/Mother/robot_ws/src/robot_description/worlds/tracking_world.sdf'
    
    # xacro 명령어로 URDF 생성
    robot_description = Command(['xacro ', robot_urdf_xacro])
    
    return LaunchDescription([
        # Gazebo 시뮬레이션 (직접 실행)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
        ),
        
        # 로봇 상태 발행자
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
            ]
        ),
        
        # 추적 제어 노드
        Node(
            package='robot_control',
            executable='tracking_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
        
        # 바퀴 제어 노드
        Node(
            package='robot_base',
            executable='wheel_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
        
        # 사람 감지 노드 (YOLO)
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
    ])
