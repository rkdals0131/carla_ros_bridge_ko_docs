# ROS 센서

CARLA ROS 브릿지는 다양한 센서 데이터를 ROS 토픽으로 제공합니다.

## 카메라 센서

### RGB 카메라
실제 카메라와 유사한 RGB 이미지를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/camera/rgb/front/image_color  # RGB 이미지
/carla/<ROLE NAME>/camera/rgb/front/camera_info  # 카메라 정보

# 메시지 타입
sensor_msgs/Image
sensor_msgs/CameraInfo
```

### 깊이 카메라
깊이 정보를 포함한 이미지를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/camera/depth/front/image_depth  # 깊이 이미지
/carla/<ROLE NAME>/camera/depth/front/camera_info  # 카메라 정보

# 메시지 타입
sensor_msgs/Image
sensor_msgs/CameraInfo
```

### 세그멘테이션 카메라
객체 분류 정보를 포함한 이미지를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/camera/semantic_segmentation/front/image_segmentation  # 세그멘테이션 이미지
/carla/<ROLE NAME>/camera/semantic_segmentation/front/camera_info  # 카메라 정보

# 메시지 타입
sensor_msgs/Image
sensor_msgs/CameraInfo
```

## LiDAR 센서

### 표준 LiDAR
3D 포인트 클라우드 데이터를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/lidar/front/point_cloud  # 포인트 클라우드

# 메시지 타입
sensor_msgs/PointCloud2
```

### 시맨틱 LiDAR
객체 분류 정보가 포함된 3D 포인트 클라우드 데이터를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/lidar/semantic_segmentation/front/point_cloud  # 시맨틱 포인트 클라우드

# 메시지 타입
sensor_msgs/PointCloud2
```

## GNSS 센서
위치 정보를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/gnss/fix  # GNSS 위치

# 메시지 타입
sensor_msgs/NavSatFix
```

## IMU 센서
관성 측정 데이터를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/imu/data  # IMU 데이터

# 메시지 타입
sensor_msgs/Imu
```

## 레이더 센서
레이더 데이터를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/radar/front/points  # 레이더 포인트

# 메시지 타입
sensor_msgs/PointCloud2
```

## 기타 센서

### 오도메트리
차량의 위치와 방향 정보를 제공합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/odometry  # 오도메트리 데이터

# 메시지 타입
nav_msgs/Odometry
```

### 차선 침범
차선 침범 이벤트를 감지합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/lane_invasion  # 차선 침범 이벤트

# 메시지 타입
carla_msgs/CarlaLaneInvasionEvent
```

### 충돌
충돌 이벤트를 감지합니다.

```yaml
# 토픽
/carla/<ROLE NAME>/collision  # 충돌 이벤트

# 메시지 타입
carla_msgs/CarlaCollisionEvent
```

## 센서 설정

센서는 `carla_spawn_objects` 패키지를 통해 설정할 수 있습니다. JSON 파일에서 센서의 위치와 파라미터를 정의합니다.

```json
{
  "sensors": [
    {
      "type": "sensor.camera.rgb",
      "id": "front",
      "x": 0.7,
      "y": 0.0,
      "z": 0.6,
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 0.0,
      "width": 1920,
      "height": 1080,
      "fov": 90
    }
  ]
}
```

## 센서 데이터 처리

ROS 노드에서 센서 데이터를 처리하는 예시:

```python
import rospy
from sensor_msgs.msg import Image, PointCloud2

def camera_callback(msg):
    # 카메라 이미지 처리
    pass

def lidar_callback(msg):
    # LiDAR 데이터 처리
    pass

def main():
    rospy.init_node('sensor_processor')
    
    # 토픽 구독
    rospy.Subscriber('/carla/ego_vehicle/camera/rgb/front/image_color', Image, camera_callback)
    rospy.Subscriber('/carla/ego_vehicle/lidar/front/point_cloud', PointCloud2, lidar_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main() 
    
```