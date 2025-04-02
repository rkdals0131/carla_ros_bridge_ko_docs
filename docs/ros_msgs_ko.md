# ROS 메시지

CARLA ROS 브릿지는 다양한 ROS 메시지 타입을 사용하여 데이터를 전송합니다.

## 차량 제어 메시지

### CarlaEgoVehicleControl
차량 제어 명령을 전송하는 메시지입니다.

```yaml
# 필드
float32 throttle      # 가속 페달 (0.0 ~ 1.0)
float32 steer        # 조향 (-1.0 ~ 1.0)
float32 brake        # 브레이크 (0.0 ~ 1.0)
bool hand_brake      # 핸드 브레이크
bool reverse         # 후진
bool manual_gear_shift  # 수동 기어 변속
int32 gear           # 기어 (1: 전진, -1: 후진)
```

### CarlaEgoVehicleInfo
차량 정보를 전송하는 메시지입니다.

```yaml
# 필드
uint32 id            # 차량 ID
string type          # 차량 타입
string role_name     # 차량 역할 이름
CarlaWheelInfo[] wheels  # 바퀴 정보
float32 max_rpm      # 최대 RPM
float32 mass         # 차량 질량
float32 drag_coefficient  # 항력 계수
float32 wheel_radius # 바퀴 반경
float32 wheel_mass   # 바퀴 질량
float32 max_steer_angle  # 최대 조향각
```

## 센서 메시지

### CarlaImage
카메라 이미지 데이터를 전송하는 메시지입니다.

```yaml
# 필드
std_msgs/Header header  # 헤더
uint32 height          # 이미지 높이
uint32 width           # 이미지 너비
string encoding        # 인코딩
uint8[] data          # 이미지 데이터
```

### CarlaPointCloud2
포인트 클라우드 데이터를 전송하는 메시지입니다.

```yaml
# 필드
std_msgs/Header header  # 헤더
uint32 height          # 높이
uint32 width           # 너비
PointField[] fields    # 포인트 필드
bool is_bigendian      # 엔디안
uint32 point_step      # 포인트 스텝
uint32 row_step        # 행 스텝
uint8[] data          # 포인트 클라우드 데이터
```

## 이벤트 메시지

### CarlaCollisionEvent
충돌 이벤트를 전송하는 메시지입니다.

```yaml
# 필드
std_msgs/Header header  # 헤더
geometry_msgs/Vector3 normal_impulse  # 충돌 법선 벡터
uint32 collision_id    # 충돌 ID
uint32 other_actor_id  # 다른 액터 ID
```

### CarlaLaneInvasionEvent
차선 침범 이벤트를 전송하는 메시지입니다.

```yaml
# 필드
std_msgs/Header header  # 헤더
string[] crossed_lane_markings  # 침범한 차선 표시
```

## 기타 메시지

### CarlaTrafficLightStatus
신호등 상태를 전송하는 메시지입니다.

```yaml
# 필드
uint32 id            # 신호등 ID
uint8 state          # 신호등 상태
float32 elapsed_time # 경과 시간
```

### CarlaTrafficLightInfo
신호등 정보를 전송하는 메시지입니다.

```yaml
# 필드
uint32 id            # 신호등 ID
geometry_msgs/Pose transform  # 위치 및 방향
geometry_msgs/Vector3 trigger_volume  # 트리거 볼륨
```

## 메시지 사용 예시

```python
# 차량 제어 메시지 생성 및 발행
from carla_msgs.msg import CarlaEgoVehicleControl

def publish_control():
    control_msg = CarlaEgoVehicleControl()
    control_msg.throttle = 0.5
    control_msg.steer = 0.0
    control_msg.brake = 0.0
    control_msg.hand_brake = False
    control_msg.reverse = False
    control_msg.manual_gear_shift = False
    control_msg.gear = 1
    
    control_pub.publish(control_msg)
``` 