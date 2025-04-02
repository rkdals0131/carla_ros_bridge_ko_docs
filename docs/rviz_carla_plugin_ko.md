# RViz CARLA 플러그인

RViz CARLA 플러그인은 CARLA 시뮬레이터의 데이터를 RViz에서 시각화할 수 있게 해주는 플러그인입니다.

## 기능

- CARLA 시뮬레이터 데이터 시각화
- 센서 데이터 표시
- 차량 상태 모니터링
- 시나리오 제어

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select rviz_carla_plugin
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select rviz_carla_plugin
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
roslaunch rviz_carla_plugin rviz_carla_plugin.launch
```

### ROS 2

```sh
ros2 launch rviz_carla_plugin rviz_carla_plugin.launch.py
```

## 플러그인 기능

### 시각화
- 카메라 이미지
- LiDAR 포인트 클라우드
- GNSS 데이터
- IMU 데이터
- 차량 상태
- 경로 계획

### 제어
- 시나리오 실행/중지
- 차량 제어
- 센서 설정
- 카메라 시점 변경

## ROS API

### 토픽

#### 발행
- `/rviz_carla_plugin/status`: 플러그인 상태
- `/rviz_carla_plugin/control_cmd`: 제어 명령
- `/rviz_carla_plugin/debug`: 디버그 정보

#### 구독
- `/carla/<ROLE NAME>/camera/<sensor_name>/image_color`: 카메라 이미지
- `/carla/<ROLE NAME>/lidar/point_cloud`: LiDAR 포인트 클라우드
- `/carla/<ROLE NAME>/gnss/gnss1/fix`: GNSS 데이터
- `/carla/<ROLE NAME>/imu/imu1/data`: IMU 데이터

### 서비스
- `/rviz_carla_plugin/set_visualization_mode`: 시각화 모드 설정
- `/rviz_carla_plugin/set_control_mode`: 제어 모드 설정
- `/rviz_carla_plugin/set_camera_view`: 카메라 시점 설정

## 설정

`rviz_carla_plugin/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `visualization_mode`: 시각화 모드
- `control_mode`: 제어 모드
- `camera_view`: 카메라 시점
- `update_rate`: 업데이트 주기
- `debug_mode`: 디버그 모드 활성화 