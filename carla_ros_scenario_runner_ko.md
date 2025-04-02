# CARLA ROS 시나리오 러너

CARLA ROS 시나리오 러너는 CARLA 시뮬레이터에서 자율주행 시나리오를 실행하고 테스트할 수 있게 해주는 패키지입니다.

## 기능

- XML 기반 시나리오 정의
- 다양한 자율주행 시나리오 실행
- 시나리오 실행 결과 분석
- ROS 메시지를 통한 시나리오 제어

## 설치 방법

### ROS 1

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_scenario_runner
source install/setup.bash
```

### ROS 2

```sh
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_scenario_runner
source install/setup.bash
```

## 실행 방법

### ROS 1

```sh
roslaunch carla_ros_scenario_runner carla_ros_scenario_runner.launch
```

### ROS 2

```sh
ros2 launch carla_ros_scenario_runner carla_ros_scenario_runner.launch.py
```

## 시나리오 정의

시나리오는 XML 파일로 정의됩니다. 예시:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<scenario>
    <ego_vehicle>
        <model>vehicle.toyota.prius</model>
        <spawn_point x="0" y="0" z="2" yaw="0"/>
    </ego_vehicle>
    <weather>
        <cloudiness>0</cloudiness>
        <precipitation>0</precipitation>
        <sun_azimuth_angle>0</sun_azimuth_angle>
        <sun_altitude_angle>90</sun_altitude_angle>
    </weather>
    <route>
        <waypoint x="10" y="0" z="2" yaw="0"/>
        <waypoint x="20" y="0" z="2" yaw="0"/>
    </route>
</scenario>
```

## ROS API

### 토픽

#### 발행
- `/carla/<ROLE NAME>/scenario_runner/status`: 시나리오 실행 상태
- `/carla/<ROLE NAME>/scenario_runner/result`: 시나리오 실행 결과

#### 구독
- `/carla/<ROLE NAME>/scenario_runner/control`: 시나리오 제어 명령

### 서비스
- `/carla/<ROLE NAME>/scenario_runner/load_scenario`: 시나리오 로드
- `/carla/<ROLE NAME>/scenario_runner/start_scenario`: 시나리오 시작
- `/carla/<ROLE NAME>/scenario_runner/stop_scenario`: 시나리오 중지
- `/carla/<ROLE NAME>/scenario_runner/get_scenario_list`: 사용 가능한 시나리오 목록 조회

## 설정

`carla_ros_scenario_runner/config/settings.yaml` 파일에서 다음 설정을 변경할 수 있습니다:

- `scenario_path`: 시나리오 XML 파일 경로
- `timeout`: 시나리오 타임아웃 시간
- `debug`: 디버그 모드 활성화
- `sync_mode`: 동기 모드 활성화
- `repetitions`: 시나리오 반복 횟수 