# CARLA ROS 브릿지

CARLA ROS 브릿지는 CARLA 시뮬레이터와 ROS 간의 양방향 통신을 가능하게 하는 핵심 패키지입니다.

## 기능

- CARLA 시뮬레이터와 ROS 간의 실시간 데이터 교환
- 차량 제어 및 센서 데이터 처리
- 시뮬레이션 상태 모니터링
- 다양한 ROS 메시지 타입 지원

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_bridge
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_bridge
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
roslaunch carla_ros_bridge carla_ros_bridge.launch
```

### ROS 2

```sh
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

## ROS API

### 토픽

#### 발행
- `/carla/<ROLE NAME>/camera/<sensor_name>/image_color`: 카메라 이미지
- `/carla/<ROLE NAME>/lidar/point_cloud`: LiDAR 포인트 클라우드
- `/carla/<ROLE NAME>/gnss/gnss1/fix`: GNSS 데이터
- `/carla/<ROLE NAME>/imu/imu1/data`: IMU 데이터
- `/carla/<ROLE NAME>/odometry`: 오도메트리 데이터
- `/carla/<ROLE NAME>/vehicle_status`: 차량 상태
- `/carla/<ROLE NAME>/vehicle_info`: 차량 정보

#### 구독
- `/carla/<ROLE NAME>/vehicle_control_cmd`: 차량 제어 명령
- `/carla/<ROLE NAME>/camera/<sensor_name>/image_color/compressed`: 압축된 카메라 이미지
- `/carla/<ROLE NAME>/lidar/point_cloud/compressed`: 압축된 LiDAR 포인트 클라우드

### 서비스
- `/carla/<ROLE NAME>/spawn_actor`: 새로운 액터 생성
- `/carla/<ROLE NAME>/destroy_actor`: 액터 제거
- `/carla/<ROLE NAME>/get_blueprints`: 사용 가능한 블루프린트 목록 조회
- `/carla/<ROLE NAME>/get_spawn_points`: 스폰 포인트 목록 조회
- `/carla/<ROLE NAME>/get_actor_list`: 현재 액터 목록 조회
- `/carla/<ROLE NAME>/get_actor_info`: 액터 정보 조회

## 설정

`carla_ros_bridge/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `host`: CARLA 서버 IP 주소
- `port`: CARLA 서버 포트
- `timeout`: 연결 타임아웃
- `passive`: 수동 모드 활성화
- `synchronous_mode`: 동기 모드 활성화
- `fixed_delta_seconds`: 고정 시간 간격
- `register_all_sensors`: 모든 센서 등록
- `ego_vehicle`: 에고 차량 설정
- `weather`: 날씨 설정
- `traffic_manager`: 교통 관리자 설정 