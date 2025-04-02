# CARLA AD 데모

[AD 데모](https://github.com/carla-simulator/ros-bridge/tree/master/carla_ad_demo)는 AD 차량이 포함된 CARLA ROS 환경을 시작하는 데 필요한 모든 것을 제공하는 예제 패키지입니다.

- [__시작하기 전에__](#시작하기-전에)
- [__데모 실행__](#데모-실행)
    - [랜덤 경로](#랜덤-경로)
    - [시나리오 실행](#시나리오-실행)
---

## 시작하기 전에

[Scenario Runner](https://carla-scenariorunner.readthedocs.io/en/latest/getting_scenariorunner/)를 설치하고 Scenario Runner ["시작하기" 튜토리얼](https://github.com/carla-simulator/scenario_runner/blob/master/Docs/getting_started.md)을 따라 제대로 작동하는지 확인합니다.

Scenario Runner 설치 경로를 찾기 위해 환경 변수를 설정합니다:

```sh
export SCENARIO_RUNNER_PATH=<scenario_runner_경로>
```

---

## 데모 실행

#### 랜덤 경로

CARLA 서버를 시작한 후, 에고 차량이 무작위로 생성된 경로를 따라가는 데모를 시작하려면 다음 명령을 실행합니다:

```sh
# ROS 1
roslaunch carla_ad_demo carla_ad_demo.launch

# ROS 2
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

다른 터미널에서 다음 명령을 실행하여 추가 차량이나 보행자를 스폰할 수도 있습니다:

```sh
cd <CARLA_경로>/PythonAPI/examples/

python3 spawn_npc.py
```

#### 시나리오 실행

미리 정의된 시나리오로 데모를 실행하려면 CARLA 서버를 시작한 후 다음 명령을 실행합니다:

```sh
# ROS 1
roslaunch carla_ad_demo carla_ad_demo_with_scenario.launch

# ROS 2
ros2 launch carla_ad_demo carla_ad_demo_with_scenario.launch.py
```

RVIZ Carla 플러그인 내에서 "FollowLeadingVehicle" 예제 시나리오를 선택하고 "Execute"를 누릅니다. 에고 차량의 위치가 재조정되고 시나리오가 처리됩니다.

`/carla/available_scenarios` 토픽에 발행하여 사용자 정의 시나리오를 지정할 수 있습니다. [launchfile](https://github.com/carla-simulator/ros-bridge/blob/ros2/carla_ad_demo/launch/carla_ad_demo_with_scenario.launch)은 이를 수행하는 방법의 예시를 보여줍니다:

```launch
  <node pkg="rostopic" type="rostopic" name="publish_scenarios"
    args="pub /carla/available_scenarios carla_ros_scenario_runner_types/CarlaScenarioList '{ 'scenarios': 