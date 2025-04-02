# CARLA Waypoint Publisher

[CARLA Waypoint Publisher](https://github.com/carla-simulator/ros-bridge/tree/master/carla_waypoint_publisher)는 웨이포인트 계산을 ROS 컨텍스트에서 사용할 수 있게 하고 CARLA 웨이포인트를 쿼리하는 서비스를 제공합니다. 웨이포인트에 대한 자세한 내용은 CARLA [문서](https://carla.readthedocs.io/en/latest/core_map/#navigation-in-carla)를 참조하십시오.

- [__Waypoint Publisher 실행__](#waypoint-publisher-실행)
    - [목표 설정](#목표-설정)
- [__Waypoint Publisher 사용__](#waypoint-publisher-사용)
- [__ROS API__](#ros-api)
    - [발행 (Publications)](#발행-publications)
    - [서비스 (Services)](#서비스-services)

---

## Waypoint Publisher 실행

CARLA 서버가 실행 중인 상태에서 다음 명령을 실행합니다:

```sh
# ROS 1
roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch

# ROS 2
ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py
```

### 목표 설정

사용 가능한 경우 `/carla/<ROLE NAME>/goal` 토픽에서 목표를 읽어오고, 그렇지 않으면 고정된 스폰 지점을 사용합니다.

목표를 설정하는 선호되는 방법은 RVIZ에서 '2D Nav Goal'을 클릭하는 것입니다.

![rviz_set_goal](images/rviz_set_start_goal.png)

---

### Waypoint Publisher 사용

[CARLA AD 데모](carla_ad_demo_ko.md)는 Waypoint Publisher를 사용하여 [CARLA AD 에이전트](carla_ad_agent_ko.md)를 위한 경로를 계획합니다. 이것이 어떻게 사용되는지에 대한 예시는 CARLA AD 데모 [launchfile](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch)을 참조하십시오.

---

## ROS API

#### 발행 (Publications)

| 토픽                           | 타입                                                                                      | 설명             |
|--------------------------------|-------------------------------------------------------------------------------------------|------------------|
| `/carla/<ego vehicle name>/waypoints` | [nav_msgs/Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html)                 | 계산된 경로 발행 |

<br>

#### 서비스 (Services)

| 서비스                                                | 타입                                                                                                                       | 설명                   |
|-------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|------------------------|
| `/carla_waypoint_publisher/<ego vehicle name>/get_waypoint` | [carla_waypoint_types/GetWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetWaypoint.srv) | 특정 위치의 웨이포인트 가져오기 |
| `/carla_waypoint_publisher/<ego vehicle name>/get_actor_waypoint` | [carla_waypoint_types/GetActorWaypoint](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_waypoint_types/srv/GetActorWaypoint.srv) | 액터 ID에 대한 웨이포인트 가져오기 | 