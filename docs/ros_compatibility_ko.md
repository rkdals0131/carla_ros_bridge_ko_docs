# ROS 호환성 노드

[ROS 호환성 노드](https://github.com/carla-simulator/ros-bridge/tree/master/ros_compatibility)는 패키지가 ROS 1과 ROS 2 모두에서 원활하게 사용될 수 있도록 하는 인터페이스입니다. `ROS_VERSION` 환경 변수에 따라 동일한 API가 ROS 1 또는 ROS 2 함수를 호출합니다. `CompatibleNode`를 상속하는 클래스를 생성하여 사용합니다.

---

## ROS 파라미터

ROS 2에서는 기본적으로 파라미터를 설정하거나 접근하기 전에 선언해야 합니다. 이는 ROS 1에서는 해당되지 않습니다. ROS 1과 ROS 2 모드 모두 유사한 방식으로 작동하도록 하기 위해, `CompatibleNode`의 ROS 2 버전에서는 `allow_undeclared_parameters` 파라미터가 `True`로 설정되어, 사전에 선언하지 않고 파라미터를 사용할 수 있습니다.

---

## 서비스

ROS 2에서는 서비스를 비동기적으로 호출할 수 있습니다. 이는 ROS 1에서는 해당되지 않습니다. 결과적으로, ROS 2 버전의 `call_service()` 메서드는 ROS 1의 동기적 동작을 모방하기 위해 비동기적으로 호출한 후 서버의 응답을 기다립니다.

!!! 경고
    응답을 기다리는 동안 ROS 2 `call_service()` 메서드는 노드를 스핀(spin)합니다. 다른 스레드가 동일한 노드를 병렬로 스핀하는 경우 문제(오류 또는 교착 상태)가 발생할 수 있습니다.