import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라 시뮬레이터
        Node(
            package='robot_perception',
            executable='camera_simulator',
            output='screen',
        ),
        
        # 사람 감지 노드 (YOLO)
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
        ),
        
        # 이미지 뷰어 (감지 결과 표시)
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
        ),
    ])
