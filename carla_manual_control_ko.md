# CARLA 수동 제어

CARLA 수동 제어 패키지는 키보드와 게임패드를 사용하여 CARLA 시뮬레이터의 차량을 수동으로 제어할 수 있게 해주는 패키지입니다.

## 기능

- 키보드 입력을 통한 차량 제어
- 게임패드 입력을 통한 차량 제어
- 실시간 차량 상태 모니터링
- ROS 메시지를 통한 제어 명령 전송

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_manual_control
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_manual_control
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
roslaunch carla_manual_control carla_manual_control.launch
```

### ROS 2

```sh
ros2 launch carla_manual_control carla_manual_control.launch.py
```

## 키보드 컨트롤

- `W`: 전진
- `S`: 후진
- `A`: 좌회전
- `D`: 우회전
- `Q`: 좌회전 (조향각 유지)
- `E`: 우회전 (조향각 유지)
- `Space`: 브레이크
- `Tab`: 자동/수동 모드 전환
- `H`: 도움말 표시

## 게임패드 컨트롤

- 왼쪽 스틱: 조향
- 오른쪽 스틱: 가속/감속
- 트리거: 브레이크
- 버튼: 기타 기능 (모드 전환 등)

## ROS API

### 토픽

#### 발행
- `/carla/<ROLE NAME>/manual_control/control_cmd`: 제어 명령
- `/carla/<ROLE NAME>/manual_control/status`: 제어 상태

#### 구독
- `/carla/<ROLE NAME>/manual_control/vehicle_status`: 차량 상태

### 서비스
- `/carla/<ROLE NAME>/manual_control/set_control_mode`: 제어 모드 설정
- `/carla/<ROLE NAME>/manual_control/set_control_parameters`: 제어 파라미터 설정

## 설정

`carla_manual_control/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `keyboard_control`: 키보드 제어 활성화
- `gamepad_control`: 게임패드 제어 활성화
- `max_steering_angle`: 최대 조향각
- `max_speed`: 최대 속도
- `max_acceleration`: 최대 가속도
- `max_deceleration`: 최대 감속도 