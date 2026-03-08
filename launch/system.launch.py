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
            'model_path',
            default_value='yolov8n_ncnn_model',
            description='YOLO NCNN 모델 디렉터리 경로'
        ),
        DeclareLaunchArgument(
            'mqtt_host',
            default_value='localhost',
            description='MQTT 브로커 호스트'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='1883',
            description='MQTT 브로커 포트'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='LiDAR 시리얼 포트'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800',
            description='LiDAR 시리얼 보드레이트 (C1=460800)'
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
        # ── MQTT 브리지 노드 (Python) ────────────────────────────────
        Node(
            package='robot_mqtt',
            executable='mqtt_bridge',
            output='screen',
            parameters=[
                {'mqtt_host': LaunchConfiguration('mqtt_host')},
                {'mqtt_port': LaunchConfiguration('mqtt_port')},
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

        # ── Pi Camera (CSI, libcamera 기반 — 소스 빌드) ───────
        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            parameters=[
                {'width': 320},
                {'height': 240},
                {'format': 'RGB888'},
            ],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
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

        # ── LiDAR 드라이버 노드 (sllidar_ros2) ───────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'channel_type': 'serial'},
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baudrate': LaunchConfiguration('serial_baudrate')},
                {'frame_id': 'laser'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Standard'},
            ]
        ),
    ])
