import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
        # 시스템 IPA 모듈 사용 (ROS Jazzy 번들 libcamera 버전 불일치 우회)
        SetEnvironmentVariable('LIBCAMERA_IPA_MODULE_PATH', '/usr/lib/aarch64-linux-gnu/libcamera'),
        SetEnvironmentVariable('LIBCAMERA_IPA_FORCE_ISOLATION', '0'),

        DeclareLaunchArgument(
            'model_path',
            default_value='yolov8n_ncnn_model',
            description='YOLO NCNN 모델 디렉터리 경로'
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

        # ── Pi Camera (CSI, libcamera 기반) ──────────────────
        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            namespace='camera',
            parameters=[
                {'width': 320},
                {'height': 240},
                {'format': 'RGB888'},
            ],
        ),

        # ── 사람/공 감지 노드 - YOLO11 NCNN (Python) ─────────
        Node(
            package='robot_perception',
            executable='person_detector',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'model_path': LaunchConfiguration('model_path')},
                {'conf_threshold': 0.2},
            ]
        ),
    ])
