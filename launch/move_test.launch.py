from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── 바퀴 제어 노드만 실행 (cmd_vel 수신 → 모터 출력) ──
        Node(
            package='robot_base',
            executable='wheel_controller',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'wheel_distance': 0.07},   # 휠 간격 70mm
                {'min_pwm': 100},           # 최소 PWM — 모터가 생각보다 강함
            ]
        ),
    ])
