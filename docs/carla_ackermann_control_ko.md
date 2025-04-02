# CARLA Ackermann 제어

CARLA Ackermann 제어 패키지는 Ackermann 스티어링 시스템을 사용하는 차량을 위한 제어 시스템을 제공합니다.

## 기능

- Ackermann 스티어링 시스템 기반 차량 제어
- 차량의 조향각, 속도, 가속도 제어
- ROS 메시지를 통한 제어 명령 전송

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ackermann_control
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ackermann_control
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
roslaunch carla_ackermann_control carla_ackermann_control.launch
```

### ROS 2

```sh
ros2 launch carla_ackermann_control carla_ackermann_control.launch.py
```

## ROS API

### 토픽

#### 발행
- `/carla/<ROLE NAME>/ackermann_control/current_control`: 현재 제어 상태
- `/carla/<ROLE NAME>/ackermann_control/current_velocity`: 현재 속도
- `/carla/<ROLE NAME>/ackermann_control/current_steering`: 현재 조향각

#### 구독
- `/carla/<ROLE NAME>/ackermann_control/control_cmd`: 제어 명령

### 서비스
- `/carla/<ROLE NAME>/ackermann_control/set_control_mode`: 제어 모드 설정
- `/carla/<ROLE NAME>/ackermann_control/set_control_parameters`: 제어 파라미터 설정

## 설정

`carla_ackermann_control/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `wheelbase`: 차축 간 거리
- `max_steering_angle`: 최대 조향각
- `max_steering_speed`: 최대 조향 속도
- `max_speed`: 최대 속도
- `max_acceleration`: 최대 가속도
- `max_deceleration`: 최대 감속도 