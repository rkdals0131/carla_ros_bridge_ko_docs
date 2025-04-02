# ROS 브리지 패키지

`carla_ros_bridge` 패키지는 기본적인 ROS 브리지 기능을 실행하는 데 필요한 주요 패키지입니다. 이 섹션에서는 ROS 환경 준비, ROS 브리지 실행, 설정 구성 방법, 동기 모드 사용, 에고 차량 제어 및 사용 가능한 구독, 발행, 서비스 요약을 배웁니다.

- [__ROS 환경 설정__](#ros-환경-설정)
    - [ROS 1 환경 준비](#ros-1-환경-준비)
    - [ROS 2 환경 준비](#ros-2-환경-준비)
- [__ROS 브리지 실행__](#ros-브리지-실행)
- [__CARLA 설정 구성__](#carla-설정-구성)
- [__동기 모드에서 ROS 브리지 사용__](#동기-모드에서-ros-브리지-사용)
- [__에고 차량 제어__](#에고-차량-제어)
- [__ROS API__](#ros-api)
    - [구독 (Subscriptions)](#구독-subscriptions)
    - [발행 (Publications)](#발행-publications)
    - [서비스 (Services)](#서비스-services)
---

## ROS 환경 설정

ROS 브리지는 공통 인터페이스를 사용하는 별도의 구현을 통해 ROS 1과 ROS 2를 모두 지원합니다. ROS 브리지를 실행하려면 사용하는 모든 터미널에서 ROS 버전에 따라 ROS 환경을 설정해야 합니다:

#### ROS 1 환경 준비:

실행할 명령은 ROS 브리지를 Debian 패키지를 통해 설치했는지 또는 소스 빌드를 통해 설치했는지에 따라 다릅니다. Debian 옵션의 경우 경로에서 ROS 버전을 변경해야 합니다:

```sh
    # ROS 브리지의 Debian 설치용. 설치된 ROS 버전에 따라 명령을 변경하십시오.
    source /opt/carla-ros-bridge/<melodic/noetic>/setup.bash

    # ROS 브리지의 GitHub 저장소 설치용
    source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
```

#### ROS 2 환경 준비:

```sh
    source ./install/setup.bash
```

## ROS 브리지 실행

ROS 환경을 설정하고 CARLA 서버를 실행한 후에는 다른 패키지를 사용하기 전에 `carla_ros_bridge` 패키지를 시작해야 합니다. 그렇게 하려면 다음 명령을 실행합니다:

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

ROS 브리지를 시작하는 동시에 다른 패키지나 플러그인을 시작하는 위의 기능을 결합한 다른 launchfile도 있습니다:

- `carla_ros_bridge_with_example_ego_vehicle.launch` (ROS 1) 및 `carla_ros_bridge_with_example_ego_vehicle.launch.py` (ROS 2)는 [`carla_spawn_objects`](carla_spawn_objects_ko.md) 및 [`carla_manual_control`](carla_manual_control_ko.md) 패키지와 함께 ROS 브리지를 시작합니다.

---

## CARLA 설정 구성

설정은 launchfile 내에서 설정하거나 명령줄에서 파일을 실행할 때 인수로 전달해야 합니다. 예를 들면 다음과 같습니다:


```sh
roslaunch carla_ros_bridge carla_ros_bridge.launch passive:=True
```

다음 설정을 사용할 수 있습니다:

* __use_sim_time__: ROS가 시스템 시간 대신 시뮬레이션 시간을 사용하도록 __True__로 설정해야 합니다. 이 파라미터는 ROS [`/clock`][ros_clock] 토픽을 CARLA 시뮬레이션 시간과 동기화합니다.
*  __host 및 port__: Python 클라이언트를 사용하여 CARLA에 연결하기 위한 네트워크 설정입니다.
* __timeout__: 서버와의 성공적인 연결을 기다리는 시간입니다.
* __passive__: 수동 모드(Passive mode)는 동기 모드에서 사용하기 위한 것입니다. 활성화되면 ROS 브리지는 한 발 물러서고 다른 클라이언트가 월드를 틱(tick)해야 __합니다__. ROS 브리지는 모든 센서에서 예상되는 모든 데이터가 수신될 때까지 기다립니다.
*  __synchronous_mode__:
	*  __false인 경우__: 데이터는 모든 `world.on_tick()` 및 모든 `sensor.listen()` 콜백에서 발행됩니다.
	*  __true인 경우 (기본값)__: ROS 브리지는 다음 틱 전에 예상되는 모든 센서 메시지를 기다립니다. 이는 전체 시뮬레이션 속도를 늦출 수 있지만 재현 가능한 결과를 보장합니다.
*  __synchronous_mode_wait_for_vehicle_control_command__: 동기 모드에서 차량 제어가 완료될 때까지 틱을 일시 중지합니다.
*  __fixed_delta_seconds__: 시뮬레이션 스텝 간의 시뮬레이션 시간(델타 초)입니다. __0.1보다 작아야 합니다__. 이에 대한 자세한 내용은 [문서](https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)를 참조하십시오.
*  __ego_vehicle__: 에고 차량을 식별하기 위한 역할 이름입니다. 관련 토픽이 생성되어 이러한 차량을 ROS에서 제어할 수 있습니다.
* __town__: 사용 가능한 CARLA 타운(예: 'town01') 또는 OpenDRIVE 파일(`.xodr`로 끝남)을 사용합니다.
*  __register_all_sensors__:
	*  __false인 경우__: 브리지에 의해 스폰된 센서만 등록됩니다.
	*  __true인 경우 (기본값)__: 시뮬레이션에 존재하는 모든 센서가 등록됩니다.


[ros_clock]: https://wiki.ros.org/Clock

---

## 동기 모드에서 ROS 브리지 사용

ROS 브리지는 기본적으로 동기 모드에서 작동합니다. 재현 가능한 결과를 보장하기 위해 현재 프레임 내에서 예상되는 모든 센서 데이터를 기다립니다.

동기 모드에서 여러 클라이언트를 실행할 때 하나의 클라이언트만 월드를 틱할 수 있습니다. ROS 브리지는 기본적으로 수동 모드가 활성화되지 않는 한 월드를 틱할 수 있는 유일한 클라이언트가 됩니다. [`ros-bridge/carla_ros_bridge/config/settings.yaml`](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/config/settings.yaml)에서 수동 모드를 활성화하면 ROS 브리지가 한 발 물러서서 다른 클라이언트가 월드를 틱하도록 허용합니다. __다른 클라이언트가 월드를 틱해야 하며, 그렇지 않으면 CARLA가 멈춥니다.__

ROS 브리지가 수동 모드가 아닌 경우(ROS 브리지가 월드를 틱하는 경우), 서버에 스텝 제어를 보내는 두 가지 방법이 있습니다:

- `/carla/control` 토픽에 [`carla_msgs.CarlaControl`](ros_msgs_ko.md#carlacontrolmsg) 메시지를 보냅니다.
- [Control rqt 플러그인](rqt_plugin_ko.md)을 사용합니다. 이 플러그인은 간단한 인터페이스를 가진 새 창을 시작합니다. 그런 다음 스텝을 관리하고 `/carla/control` 토픽에 발행하는 데 사용됩니다. 이를 사용하려면 동기 모드에서 CARLA를 실행한 상태에서 다음 명령을 실행합니다:
```sh
    rqt --standalone rqt_carla_control
```

---

## 에고 차량 제어

에고 차량을 제어하는 두 가지 모드가 있습니다:

1. 일반 모드 - `/carla/<ROLE NAME>/vehicle_control_cmd`에서 명령 읽기
2. 수동 모드 - `/carla/<ROLE NAME>/vehicle_control_cmd_manual`에서 명령 읽기. 이를 통해 소프트웨어 스택에서 발행한 차량 제어 명령을 수동으로 재정의(override)할 수 있습니다.

`/carla/<ROLE NAME>/vehicle_control_manual_override`에 발행하여 두 모드 간에 전환할 수 있습니다. 이것이 사용되는 예시는 [Carla Manual Control](carla_manual_control_ko.md)을 참조하십시오.

명령줄에서 조향을 테스트하려면:

__1.__ 에고 차량과 함께 ROS 브리지를 시작합니다:

```sh
    # ROS 1
    roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch

    # ROS 2
    ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

__2.__ 다른 터미널에서 `/carla/<ROLE NAME>/vehicle_control_cmd` 토픽에 발행합니다:

```sh
    # 최대 전진 스로틀과 최대 우회전 조향

    # ROS 1용
    rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

    # ROS 2용
    ros2 topic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl "{throttle: 1.0, steer: 1.0}" -r 10

```

차량의 현재 상태는 `/carla/<ROLE NAME>/vehicle_status` 토픽을 통해 수신할 수 있습니다. 차량에 대한 정적 정보는 `/carla/<ROLE NAME>/vehicle_info`를 통해 수신할 수 있습니다.

[AckermannDrive](https://docs.ros.org/en/api/ackermann_msgs/html/msg/AckermannDrive.html) 메시지를 사용하여 에고 차량을 제어할 수 있습니다. 이는 [CARLA Ackermann Control](carla_ackermann_control_ko.md) 패키지를 사용하여 달성할 수 있습니다.

---

## ROS API

#### 구독 (Subscriptions)

| 토픽                 | 타입                                                                                              | 설명                                           |
|----------------------|---------------------------------------------------------------------------------------------------|------------------------------------------------|
| `/carla/debug_marker`  | [visualization_msgs/MarkerArray](https://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html) | CARLA 월드에 마커를 그립니다.                  |
| `/carla/weather_control` | [carla_msgs/CarlaWeatherParameters](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaWeatherParameters.msg) | CARLA 날씨 파라미터를 설정합니다.            |
| `/carla/control`     | [carla_msgs/CarlaControl](ros_msgs_ko.md#carlacontrolmsg)                                         | CARLA 시뮬레이션 스텝 제어 (재생/일시정지/스텝) |

<br>

!!! 주의
    `debug_marker`를 사용할 때 마커가 센서에서 발행하는 데이터에 영향을 미칠 수 있음을 유의하십시오. 지원되는 마커에는 화살표(두 점으로 지정), 점, 큐브 및 라인 스트립이 포함됩니다.
<br>

#### 발행 (Publications)

| 토픽              | 타입                                                                                        | 설명                                 |
|-------------------|---------------------------------------------------------------------------------------------|--------------------------------------|
| `/carla/status`     | [carla_msgs/CarlaStatus](ros_msgs_ko.md#carlastatusmsg)                                     | CARLA의 현재 상태를 읽습니다.        |
| `/carla/world_info` | [carla_msgs/CarlaWorldInfo](ros_msgs_ko.md#carlaworldinfomsg)                               | 현재 CARLA 맵에 대한 정보입니다.     |
| `/clock`          | [rosgraph_msgs/Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html) | ROS에서 시뮬레이션된 시간을 발행합니다. |
| `/rosout`         | [rosgraph_msgs/Log](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Log.html)     | ROS 로깅입니다.                      |

<br>

#### 서비스 (Services)

| 서비스                  | 타입                                                                                                                                   | 설명                 |
|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------|----------------------|
| `/carla/destroy_object` | [carla_msgs/DestroyObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/DestroyObject.srv) | 객체를 파괴합니다.   |
| `/carla/get_blueprints` | [carla_msgs/GetBlueprints.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/GetBlueprints.srv) | 블루프린트를 얻습니다. |
| `/carla/spawn_object`   | [carla_msgs/SpawnObject.srv](https://github.com/carla-simulator/ros-carla-msgs/blob/f75637ce83a0b4e8fbd9818980c9b11570ff477c/srv/SpawnObject.srv)   | 객체를 스폰합니다.   |

---

