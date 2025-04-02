# RQT CARLA 플러그인

RQT CARLA 플러그인은 CARLA 시뮬레이터를 RQT 인터페이스를 통해 제어할 수 있게 해주는 플러그인입니다.

## 기능

- CARLA 시뮬레이터 제어
- 차량 상태 모니터링
- 센서 데이터 표시
- 시나리오 제어

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select rqt_carla_control
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select rqt_carla_control
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
rqt --standalone rqt_carla_control
```

### ROS 2

```sh
rqt --standalone rqt_carla_control
```

## 플러그인 기능

### 제어
- 차량 제어 (가속, 감속, 조향)
- 센서 설정
- 시나리오 실행/중지
- 날씨 설정

### 모니터링
- 차량 상태
- 센서 데이터
- 시나리오 상태
- 시스템 상태

## ROS API

### 토픽

#### 발행
- `/rqt_carla_control/control_cmd`: 제어 명령
- `/rqt_carla_control/status`: 플러그인 상태
- `/rqt_carla_control/debug`: 디버그 정보

#### 구독
- `/carla/<ROLE NAME>/vehicle_status`: 차량 상태
- `/carla/<ROLE NAME>/camera/<sensor_name>/image_color`: 카메라 이미지
- `/carla/<ROLE NAME>/lidar/point_cloud`: LiDAR 포인트 클라우드

### 서비스
- `/rqt_carla_control/set_control_mode`: 제어 모드 설정
- `/rqt_carla_control/set_visualization_mode`: 시각화 모드 설정
- `/rqt_carla_control/set_scenario`: 시나리오 설정

## 설정

`rqt_carla_control/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `control_mode`: 제어 모드
- `visualization_mode`: 시각화 모드
- `update_rate`: 업데이트 주기
- `debug_mode`: 디버그 모드 활성화
- `vehicle_model`: 차량 모델
- `sensor_config`: 센서 설정 