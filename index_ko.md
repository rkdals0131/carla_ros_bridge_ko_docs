# ROS 브리지 문서

이 문서는 ROS와 CARLA 간의 양방향 통신을 가능하게 하는 ROS 브리지에 대한 문서입니다. CARLA 서버의 정보는 ROS 토픽으로 변환됩니다. 마찬가지로 ROS 노드 간에 전송되는 메시지는 CARLA에 적용될 명령으로 변환됩니다.

ROS 브리지는 ROS 1과 ROS 2 모두와 호환됩니다.

ROS 브리지는 다음과 같은 기능을 자랑합니다:

- LIDAR, Semantic LIDAR, 카메라(깊이, 세그멘테이션, RGB, DVS), GNSS, Radar 및 IMU에 대한 센서 데이터 제공.
- 변환(transforms), 신호등 상태, 시각화 마커, 충돌 및 차선 침범과 같은 객체 데이터 제공.
- 조향, 스로틀 및 브레이크를 통한 AD 에이전트 제어.
- 동기 모드, 시뮬레이션 재생 및 일시 중지, 시뮬레이션 파라미터 설정과 같은 CARLA 시뮬레이션의 측면 제어.

---

## 시작하기

- [__ROS 1용 ROS 브리지 설치__](ros_installation_ros1_ko.md)
- [__ROS 2용 ROS 브리지 설치__](ros_installation_ros2_ko.md)

---

## 주요 ROS 브리지 패키지에 대해 알아보기

- [__CARLA ROS 브리지__](run_ros_ko.md) - ROS 브리지를 실행하는 데 필요한 주요 패키지
- [__ROS 호환성 노드__](ros_compatibility_ko.md) - 동일한 API가 ROS 1 또는 ROS 2 함수를 호출할 수 있도록 하는 인터페이스

---

## 추가 ROS 브리지 패키지에 대해 알아보기

- [__CARLA Spawn Objects__](carla_spawn_objects_ko.md) - 액터를 스폰하는 일반적인 방법 제공
- [__CARLA Manual Control__](carla_manual_control_ko.md) - 에고 차량을 위한 ROS 기반 시각화 및 제어 도구 (CARLA에서 제공하는 `carla_manual_control.py`와 유사)
- [__CARLA Ackerman Control__](carla_ackermann_control_ko.md) - Ackermann 명령을 조향/스로틀/브레이크로 변환하는 컨트롤러
- [__CARLA Waypoint Publisher__](carla_waypoint_ko.md) - CARLA 웨이포인트 발행 및 쿼리
- [__CARLA AD Agent__](carla_ad_agent_ko.md) - 경로를 따르고, 충돌을 피하며, 신호등을 준수하는 예제 에이전트
- [__CARLA AD Demo__](carla_ad_demo_ko.md) - AD 차량이 포함된 CARLA ROS 환경을 시작하는 데 필요한 모든 것을 제공하는 예제 패키지
- [__CARLA ROS Scenario Runner__](carla_ros_scenario_runner_ko.md) - ROS를 통해 CARLA Scenario Runner로 OpenScenarios를 실행하기 위한 래퍼
- [__CARLA Twist to Control__](carla_twist_to_control_ko.md) - Twist 제어를 CARLA 차량 제어로 변환
- [__RVIZ 플러그인__](rviz_plugin_ko.md) - CARLA를 시각화/제어하기 위한 RVIZ 플러그인
- [__RQT 플러그인__](rqt_plugin_ko.md) - CARLA를 제어하기 위한 RQT 플러그인
- [__PCL Recorder__](pcl_recorder_ko.md) - 시뮬레이션에서 캡처한 데이터로 포인트 클라우드 맵 생성

---

## 참조 자료 탐색

- [__ROS 센서__](ros_sensors_ko.md) - 다양한 센서에서 사용 가능한 참조 토픽
- [__ROS 메시지__](ros_msgs_ko.md) - CARLA ROS 메시지에서 사용 가능한 참조 파라미터 