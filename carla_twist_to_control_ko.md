# Carla Twist to Control

[`carla_twist_to_control` 패키지](https://github.com/carla-simulator/ros-bridge/tree/master/carla_twist_to_control)는 [geometry_msgs.Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)를 [carla_msgs.CarlaEgoVehicleControl](ros_msgs_ko.md#carlaegovehiclecontrolmsg)로 변환합니다.

---
## ROS API

### 구독 (Subscriptions)

| 토픽                           | 타입                                                                    | 설명                                  |
|--------------------------------|-------------------------------------------------------------------------|---------------------------------------|
| `/carla/<ROLE NAME>/vehicle_info` | [`carla_msgs.CarlaEgoVehicleInfo`](ros_msgs_ko.md#carlaegovehicleinfomsg) | 에고 차량 정보, 최대 조향각 수신용. |
| `/carla/<ROLE NAME>/twist`        | [`geometry_msgs.Twist`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | 변환할 Twist 메시지.                 |

<br>

### 발행 (Publications)

| 토픽                               | 타입                                                                        | 설명                       |
|------------------------------------|-----------------------------------------------------------------------------|----------------------------|
| `/carla/<ROLE NAME>/vehicle_control_cmd` | [`carla_msgs.CarlaEgoVehicleControl`](ros_msgs_ko.md#carlaegovehiclecontrolmsg) | 변환된 차량 제어 명령.   | 