# ROS2 패키지 구조

## 패키지 개요
FTA는 다음 ROS2 패키지들로 구성됩니다:

### fta_actuators
액추에이터 제어 노드들을 포함하는 메인 패키지

**노드:**
- `led_buzzer_node`: LED와 Buzzer 제어
- `speaker_node`: 음성 출력 및 사운드 재생
- `battery_node`: 배터리 모니터링 및 제어 (예정)
- `power_management_node`: 저전력 모드 관리 (예정)

### fta_msgs
FTA에서 사용하는 커스텀 메시지 타입 정의

**메시지:**
- `ActuatorCommand.msg`: 액추에이터 제어 명령
- `ActuatorStatus.msg`: 액추에이터 상태
- `LEDBuzzerControl.msg`: LED/Buzzer 제어 파라미터
- `SpeakerControl.msg`: Speaker 제어 파라미터

### fta_bringup
런치 파일 및 설정 파일 관리

**런치 파일:**
- `fta.launch.py`: 전체 FTA 시스템 실행
- `actuators.launch.py`: 개별 액추에이터 노드 실행

## 개발 워크플로우

### 1. 패키지 생성
```bash
# 워크스페이스 디렉토리에서
ros2 pkg create --build-type ament_cmake fta_actuators
ros2 pkg create --build-type ament_cmake fta_msgs
ros2 pkg create --build-type ament_cmake fta_bringup
```

### 2. 빌드
```bash
colcon build --packages-select fta_actuators fta_msgs fta_bringup
```

### 3. 테스트
```bash
# 환경 설정
source install/setup.bash

# 노드 실행
ros2 run fta_actuators led_buzzer_node
```

### 4. 토픽 모니터링
```bash
# /actions/event 토픽 확인
ros2 topic echo /actions/event

# /actuators/status 토픽 확인
ros2 topic echo /actuators/status
```

## 하드웨어 독립성 구현

각 액추에이터 노드는 다음 구조를 따릅니다:

```cpp
// 공통 인터페이스
class ActuatorInterface {
public:
    virtual bool initialize() = 0;
    virtual bool execute(const Command& cmd) = 0;
    virtual Status getStatus() = 0;
};

// 드라이버 구현 (제조사별)
class LEDDriver_ManufacturerA : public ActuatorInterface {
    // 구체적인 하드웨어 제어 로직
};
```

## QoS 설정
모든 액추에이터 관련 토픽은 **QoS1 (신뢰성 보장)** 사용:

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();
```
