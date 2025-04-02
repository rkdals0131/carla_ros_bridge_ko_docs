# CARLA AD 에이전트

[CARLA AD 에이전트](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_agent)는 주어진 경로를 따라가고, 다른 차량과의 충돌을 피하며, 그라운드 트루스 데이터에 접근하여 신호등 상태를 준수할 수 있는 AD(자율 주행) 에이전트입니다. [CARLA AD 데모](carla_ad_demo_ko.md)에서 ROS 브리지를 사용하는 방법의 예시를 제공하기 위해 사용됩니다.

- [__요구 사항__](#요구-사항)
- [__ROS API__](#ros-api)
    - [__AD 에이전트 노드__](#ad-에이전트-노드)
        - [파라미터](#파라미터)
        - [구독 (Subscriptions)](#구독-subscriptions)
        - [발행 (Publications)](#발행-publications)
    - [__로컬 플래너 노드__](#로컬-플래너-노드)
        - [파라미터](#파라미터-1)
        - [구독 (Subscriptions)](#구독-subscriptions-1)
        - [발행 (Publications)](#발행-publications-1)

내부적으로 CARLA AD 에이전트는 [로컬 플래닝](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_agent/src/carla_ad_agent/local_planner.py)을 위한 별도의 노드를 사용합니다. 이 노드는 기어 변속 지연이 없는 `vehicle.tesla.model3`에 최적화되어 있습니다.

PID 파라미터는 [Ziegler-Nichols 방법](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)을 통해 수집되었습니다.

---

## 요구 사항

`carla_ad_agent`를 사용하려면 최소한의 센서 세트를 스폰해야 합니다 ([Carla Spawn Objects](carla_spawn_objects_ko.md)에서 센서 스폰 방법에 대한 정보 참조):

- 차량에 부착된 `odometry` 역할 이름(role-name)을 가진 오도메트리 의사 센서 (`sensor.pseudo.odom`).
- 차량에 부착된 `objects` 역할 이름을 가진 객체 의사 센서 (`sensor.pseudo.objects`).
- `traffic_lights` 역할 이름을 가진 신호등 의사 센서 (`sensor.pseudo.traffic_lights`).

---

## ROS API

### AD 에이전트 노드

#### 파라미터

| 파라미터     | 타입                | 설명                             |
|--------------|---------------------|----------------------------------|
| `role_name`  | string (기본값: `ego_vehicle`) | 에고 차량의 CARLA 역할 이름      |
| `avoid_risk` | bool (기본값: `true`)   | True이면 다른 차량과의 충돌을 피하고 신호등을 준수합니다 |

<br>

#### 구독 (Subscriptions)

| 토픽                           | 타입                                                                                      | 설명                               |
|--------------------------------|-------------------------------------------------------------------------------------------|------------------------------------|
| `/carla/<ROLE NAME>/target_speed` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)             | 에고 차량의 목표 속도              |
| `/carla/<ROLE NAME>/odometry`     | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)           | 에고 차량의 오도메트리             |
| `/carla/<ROLE NAME>/vehicle_info` | [carla_msgs/CarlaEgoVehicleInfo](ros_msgs_ko.md#carlaegovehicleinfomsg)                   | 에고 차량의 CARLA 액터 ID 식별     |
| `/carla/<ROLE NAME>/objects`      | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | 다른 액터에 대한 정보            |
| `/carla/traffic_lights/status`  | [carla_msgs/CarlaTrafficLightStatusList](ros_msgs_ko.md#carlatrafficlightstatuslistmsg) | 현재 신호등 상태 가져오기        |
| `/carla/traffic_lights/info`    | [carla_msgs/CarlaTrafficLightInfoList](ros_msgs_ko.md#carlatrafficlightinfolistmsg)   | 신호등 정보 가져오기             |

<br>

#### 발행 (Publications)

| 토픽                           | 타입                                                                                      | 설명       |
|--------------------------------|-------------------------------------------------------------------------------------------|------------|
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)             | 목표 속도  |

<br>

### 로컬 플래너 노드

#### 파라미터

| 파라미터           | 타입                | 설명                                 |
|--------------------|---------------------|--------------------------------------|
| `role_name`        | string (기본값: `ego_vehicle`) | 에고 차량의 CARLA 역할 이름          |
| `control_time_step`| float (기본값: `0.05`)   | 제어 루프 속도                       |
| `Kp_lateral`       | float (기본값: `0.9`)    | 횡방향 PID 컨트롤러의 비례(P) 항     |
| `Ki_lateral`       | float (기본값: `0.0`)    | 횡방향 PID 컨트롤러의 적분(I) 항     |
| `Kd_lateral`       | float (기본값: `0.0`)    | 횡방향 PID 컨트롤러의 미분(D) 항     |
| `Kp_longitudinal`  | float (기본값: `0.206`)  | 종방향 PID 컨트롤러의 비례(P) 항     |
| `Ki_longitudinal`  | float (기본값: `0.0206`) | 종방향 PID 컨트롤러의 적분(I) 항     |
| `Kd_longitudinal`  | float (기본값: `0.515`)  | 종방향 PID 컨트롤러의 미분(D) 항     |

<br>

#### 구독 (Subscriptions)

| 토픽                           | 타입                                                                                      | 설명                 |
|--------------------------------|-------------------------------------------------------------------------------------------|----------------------|
| `/carla/<ROLE NAME>/waypoints`    | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)                 | 따라갈 경로          |
| `/carla/<ROLE NAME>/odometry`     | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)           | 에고 차량의 오도메트리 |
| `/carla/<ROLE NAME>/speed_command` | [std_msgs/Float64](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html)             | 목표 속도            |

<br>

#### 발행 (Publications)

| 토픽                               | 타입                                                                                              | 설명                   |
|------------------------------------|---------------------------------------------------------------------------------------------------|------------------------|
| `/carla/<ROLE NAME>/next_target`     | [visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | 다음 목표 포즈 마커    |
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [carla_msgs/CarlaEgoVehicleControl](ros_msgs_ko.md#carlaegovehiclecontrolmsg)                 | 차량 제어 명령         |
