# ROS 2 설치 가이드

이 가이드는 CARLA ROS 브릿지를 ROS 2 환경에 설치하는 방법을 설명합니다.

## 요구사항

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- CARLA 0.9.13 이상

## 설치 단계

### 1. ROS 2 Humble 설치

```sh
# ROS 저장소 설정
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 패키지 목록 업데이트
sudo apt update

# ROS 2 Humble 설치
sudo apt install ros-humble-desktop

# 의존성 설치
sudo apt install python3-colcon-common-extensions
```

### 2. ROS 2 환경 설정

```sh
# ROS 2 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# ROS 의존성 초기화
sudo rosdep init
rosdep update
```

### 3. CARLA 설치

```sh
# CARLA 저장소 클론
git clone https://github.com/carla-simulator/carla.git

# CARLA 빌드
cd carla
./Update.sh
make launch
```

### 4. CARLA ROS 브릿지 설치

```sh
# 작업 공간 생성
mkdir -p ~/carla-ros-bridge/src
cd ~/carla-ros-bridge/src

# ROS 브릿지 저장소 클론
git clone https://github.com/carla-simulator/ros-bridge.git

# 의존성 설치
cd ..
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build
source install/setup.bash
```

### 5. Python 의존성 설치

```sh
# Python 패키지 설치
pip3 install -r src/ros-bridge/requirements.txt
```

## 테스트

### 1. CARLA 서버 실행

```sh
cd ~/carla
./CarlaUE4.sh
```

### 2. ROS 브릿지 실행

새로운 터미널에서:

```sh
cd ~/carla-ros-bridge
source install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
```

### 3. 예제 실행

새로운 터미널에서:

```sh
cd ~/carla-ros-bridge
source install/setup.bash
ros2 launch carla_ad_demo carla_ad_demo.launch.py
```

## 문제 해결

### 1. 빌드 오류

빌드 중 오류가 발생하면 다음을 확인하세요:
- 모든 의존성이 올바르게 설치되었는지
- Python 버전이 호환되는지
- CARLA 버전이 호환되는지

### 2. 실행 오류

실행 중 오류가 발생하면 다음을 확인하세요:
- CARLA 서버가 실행 중인지
- ROS 2 환경이 올바르게 설정되었는지
- 필요한 포트가 사용 가능한지

### 3. 센서 데이터 문제

센서 데이터가 표시되지 않으면 다음을 확인하세요:
- 센서가 올바르게 설정되었는지
- 토픽이 발행되고 있는지
- 권한이 올바르게 설정되었는지 