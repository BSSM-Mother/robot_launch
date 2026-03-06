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

        # ── 장애물 회피 노드 (C++) ─────────────────────────────
        # /scan (LiDAR) + /cmd_vel_raw → /cmd_vel
        # robot_radius: 로봇 반지름(m), safety_margin: 추가 여유(m)
        Node(
            package='robot_control',
            executable='obstacle_avoider',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                # 로봇 지름 140mm → 반지름 0.07m
                {'robot_radius': 0.07},
                # stop_dist = 0.07 + 0.03 = 0.10m
                # 200mm 복도 중앙 주행 시 라이다→벽 거리가 0.10m이므로
                # 이 값보다 작아지면 긴급 정지
                {'safety_margin': 0.03},
                # slow_dist = 0.10 + 0.25 = 0.35m (이 거리부터 감속 시작)
                {'slowdown_zone': 0.25},
                {'forward_half_angle_deg': 35.0},
                {'side_start_deg': 40.0},
                {'side_end_deg': 140.0},
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
                {'conf_threshold': 0.2},
            ]
        ),
    ])
