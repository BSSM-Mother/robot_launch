import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ─────────────────────────────────────────────────────────────
# Raspberry Pi 전용 launch 파일
#
# YOLO11n NCNN 모델 변환 (데스크탑에서 1회 실행 후 Pi로 복사):
#   python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='ncnn')"
#   → yolo11n_ncnn_model/ 폴더를 Pi의 작업 디렉터리에 복사
# ─────────────────────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device_id',
            default_value='0',
            description='USB 카메라 디바이스 인덱스 (0, 1, 2...)'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='yolo11n_ncnn_model',
            description='YOLO11 NCNN 모델 디렉터리 경로'
        ),

        # ── 추적 제어 노드 (C++) ──────────────────────────────
        Node(
            package='robot_control',
            executable='tracking_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ]
        ),

        # ── 바퀴 제어 노드 (C++) ──────────────────────────────
        Node(
            package='robot_base',
            executable='wheel_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False},
            ]
        ),

        # ── USB 카메라 (Pi 최적화: 320×240 @ 15fps) ───────────
        Node(
            package='image_tools',
            executable='cam2image',
            output='screen',
            parameters=[
                {'device_id': LaunchConfiguration('camera_device_id')},
                {'width': 320},
                {'height': 240},
                {'frequency': 15.0},
            ],
            remappings=[
                ('/image', '/camera/image_raw'),
            ]
        ),

        # ── 사람/공 감지 노드 - YOLO11 NCNN (Python) ─────────
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'model_path': LaunchConfiguration('model_path')},
                {'conf_threshold': 0.4},
            ]
        ),
    ])
