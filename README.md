# robot_launch - 로봇 시스템 실행 관리

**로봇의 모든 노드를 통합하여 실행하고 파라미터를 관리하는 런치 패키지**

## 📌 개요

ROS2 런치 파일을 통해 카메라, 인식, 제어, 모터 등 모든 노드를 한 번에 시작하고, 각 노드의 파라미터를 중앙에서 관리합니다.

## 📁 폴더 구조

```
robot_launch/
├── launch/
│   ├── robot.launch.py           # 메인 런치 파일
│   ├── bringup.launch.py         # 하드웨어 초기화
│   ├── perception.launch.py      # 비전 노드
│   ├── control.launch.py         # 제어 노드
│   └── sim.launch.py             # 시뮬레이션 모드
├── config/
│   ├── robot_params.yaml         # 로봇 전역 파라미터
│   ├── tracking_params.yaml      # 추적 제어 파라미터
│   ├── perception_params.yaml    # 비전 파라미터
│   └── hardware_params.yaml      # 하드웨어 설정
└── CMakeLists.txt
```

## 🚀 실행 방법

### 기본 실행 (전체 시스템)

```bash
ros2 launch robot_launch robot.launch.py
```

### 개별 런치 파일 실행

```bash
# 시뮬레이션 모드
ros2 launch robot_launch sim.launch.py

# 비전 처리만
ros2 launch robot_launch perception.launch.py

# 제어 로직만
ros2 launch robot_launch control.launch.py
```

## ⚙️ 런치 파일 구성

### robot.launch.py - 메인 런치 파일

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라 드라이버
        Node(
            package='camera_ros',
            executable='camera_ros_node',
            parameters=[{
                'video_device': '/dev/video0',
                'camera_info_url': 'file:///path/to/camera_info.yaml',
            }]
        ),

        # 인식 노드
        Node(
            package='robot_perception',
            executable='person_detector',
            parameters=['config/perception_params.yaml']
        ),

        # 제어 노드들
        Node(
            package='robot_control',
            executable='tracking_controller',
            parameters=['config/tracking_params.yaml']
        ),

        # 모터 제어
        Node(
            package='robot_base',
            executable='wheel_controller',
            parameters=['config/hardware_params.yaml']
        ),
    ])
```

## 📋 파라미터 파일 예시

### config/hardware_params.yaml
```yaml
wheel_controller:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 115200
    motor_left_id: 1
    motor_right_id: 2
    timeout: 5.0
```

### config/tracking_params.yaml
```yaml
tracking_controller:
  ros__parameters:
    kp_linear: 0.6
    ki_linear: 0.02
    kd_linear: 0.08
    kp_angular: 0.8
    ki_angular: 0.03
    kd_angular: 0.15
    search_timeout: 5.0
    max_linear_vel: 0.8
    max_angular_vel: 1.5
```

### config/perception_params.yaml
```yaml
person_detector:
  ros__parameters:
    model_path: "yolov8n.pt"
    model_format: "pytorch"
    confidence_threshold: 0.5
    camera_topic: "/camera/image_raw"
```

## 🎯 노드 시작 순서

```
1. camera_ros             ← 센서 입력
     │
     ▼
2. robot_perception       ← 비전 처리
     │
     ▼
3. robot_control (all)    ← 제어 로직
     │
     ▼
4. robot_base             ← 모터 명령 실행
```

## 📊 전체 아키텍처

```
┌─────────────────────────────────────────────────┐
│          robot.launch.py (Main)                 │
├─────────────────────────────────────────────────┤
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │ Bringup (하드웨어 초기화)                 │  │
│  │ - camera_ros                             │  │
│  │ - sllidar_ros2 (LIDAR)                   │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                             │
│  ┌──────────────────────────────────────────┐  │
│  │ Perception (비전 처리)                    │  │
│  │ - person_detector                        │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                             │
│  ┌──────────────────────────────────────────┐  │
│  │ Control (제어 로직)                       │  │
│  │ - tracking_controller                    │  │
│  │ - obstacle_avoider                       │  │
│  │ - person_mover                           │  │
│  └──────────────────────────────────────────┘  │
│                    ↓                             │
│  ┌──────────────────────────────────────────┐  │
│  │ Base (모터 제어)                          │  │
│  │ - wheel_controller (시리얼 통신)          │  │
│  └──────────────────────────────────────────┘  │
│                                                  │
└─────────────────────────────────────────────────┘
```

## 🔧 런치 파라미터 옵션

```bash
# 파라미터 오버라이드
ros2 launch robot_launch robot.launch.py \
    port:=/dev/ttyACM0 \
    model_path:=yolo11n.pt \
    confidence_threshold:=0.6

# 시뮬레이션 모드
ros2 launch robot_launch robot.launch.py use_sim_time:=True

# 디버그 출력
ros2 launch robot_launch robot.launch.py \
    launch_prefix:='xterm -e gdb -ex run --args'
```

## 🛠️ 트러블슈팅

### 노드가 시작되지 않음

1. 의존성 확인:
   ```bash
   apt-get update && rosdep install --from-paths src --ignore-src -y
   ```

2. 런치 파일 문법 확인:
   ```bash
   ros2 launch robot_launch robot.launch.py --show-args
   ```

### 파라미터 로드 오류

```bash
# 파라미터 파일 경로 확인
ros2 launch robot_launch robot.launch.py \
    --params-file $(ros2 pkg prefix robot_launch)/share/robot_launch/config/tracking_params.yaml
```

### 포트 권한 오류

```bash
# 시리얼 포트 권한 추가
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

## 📊 실행 상태 모니터링

```bash
# 실행 중인 노드 확인
ros2 node list

# 토픽 확인
ros2 topic list -t

# 노드 그래프 시각화
rqt_graph

# 토픽 데이터 모니터링
ros2 topic echo /cmd_vel_raw
```

## 🌍 시뮬레이션 모드

```bash
# Gazebo와 함께 실행
ros2 launch robot_launch sim.launch.py world:=empty
ros2 launch robot_launch sim.launch.py world:=warehouse
```

## 📝 런치 파일 커스터마이징

### 새로운 노드 추가

```python
# robot.launch.py에 추가
Node(
    package='my_package',
    executable='my_node',
    name='my_node',
    output='screen',  # 터미널에 출력
    parameters=[LaunchConfiguration('params_file')]
),
```

### 조건부 실행

```python
from launch.conditions import IfCondition

condition = IfCondition(LaunchConfiguration('use_sim'))

Node(
    package='gazebo',
    executable='gazebosim',
    condition=condition
),
```

## ⚠️ 주의사항

- **실행 순서**: 센서 드라이버는 제어 로직 전에 시작되어야 함
- **포트 충돌**: 여러 런치 파일 동시 실행 시 포트 설정 확인
- **파라미터 타입**: YAML 파일의 타입 일치 필요 (문자열, 숫자, 불린)
- **로그 파일**: `~/.ros/log/` 디렉토리에서 로그 확인 가능

## 🔗 연관 패키지

- **포함 노드**: camera_ros, robot_perception, robot_control, robot_base, sllidar_ros2
- **설정 소스**: robot_description (URDF, 메시)
- **의존성**: rclpy, rclcpp, ros2launch

## 📚 관련 명령어

```bash
# 런치 파일 문법 검사
ros2 launch robot_launch robot.launch.py --show-args

# 런치 파일 내용 확인
ros2 launch robot_launch robot.launch.py --show-launch

# 로그 확인
tail -f ~/.ros/log/latest/*/stdout.log
```
