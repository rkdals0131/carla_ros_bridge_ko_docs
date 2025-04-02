# 포인트 클라우드 맵 생성

[PCL 레코더 패키지](https://github.com/carla-simulator/ros-bridge/tree/master/pcl_recorder)를 사용하면 CARLA 맵에서 포인트 클라우드 맵을 생성할 수 있습니다.

---

## 시작하기 전에

`pcl-tools` 라이브러리를 설치합니다:

```sh
sudo apt install pcl-tools
```

## 실행 방법

### ROS 1

```sh
roslaunch pcl_recorder pcl_recorder.launch
```

### ROS 2

```sh
ros2 launch pcl_recorder pcl_recorder.launch.py
```

## 사용 방법

1. CARLA 시뮬레이터를 실행합니다.
2. PCL 레코더를 실행합니다.
3. 시뮬레이터에서 차량을 운전하여 맵을 스캔합니다.
4. 레코딩을 중지하려면 `Ctrl+C`를 누릅니다.

## 출력 파일

레코딩된 포인트 클라우드 맵은 `~/.ros/pcl_recorder/` 디렉토리에 저장됩니다. 파일 이름은 `map.pcd`입니다.

## ROS API

### 구독

- `/carla/<ROLE NAME>/lidar/point_cloud`: LiDAR 센서의 포인트 클라우드 데이터를 구독합니다.

### 서비스

- `/pcl_recorder/start_recording`: 레코딩을 시작합니다.
- `/pcl_recorder/stop_recording`: 레코딩을 중지합니다. 