import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
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
        DeclareLaunchArgument(
            'camera_device_id',
            default_value='0',
            description='USB camera device index (0, 1, 2...)'
        ),
        # 로봇 상태 발행자 (robot_state_publisher)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
            ]
        ),
        
        # 추적 제어 노드 (C++)
        Node(
            package='robot_control',
            executable='tracking_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
        
        # 바퀴 제어 노드 (C++)
        Node(
            package='robot_base',
            executable='wheel_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
        
        # USB 카메라 (image_tools cam2image)
        Node(
            package='image_tools',
            executable='cam2image',
            output='screen',
            parameters=[
                {'device_id': LaunchConfiguration('camera_device_id')},
                {'width': 640},
                {'height': 480},
                {'frequency': 30.0},
            ],
            remappings=[
                ('/image', '/camera/image_raw'),
            ]
        ),
        
        # 사람 감지 노드 (Python)
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        ),
    ])
