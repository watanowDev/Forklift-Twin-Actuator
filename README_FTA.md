# FTA (Forklift Twin Actuators)

ROS2 기반 지게차 액추에이터 제어 패키지

## ?? 패키지 구조

### fta_actuators (LED/Buzzer 제어)
- `include/fta_actuators/` - 헤더 파일
- `src/` - 구현 파일
- `launch/` - Launch 파일
- `config/` - 설정 파일

### fta_interfaces (메시지 정의)
- `msg/ActionEvent.msg` - 액션 이벤트
- `msg/ActuatorStatus.msg` - 액추에이터 상태

## ?? 빌드 및 실행

```bash
# 빌드
colcon build --packages-select fta_interfaces fta_actuators
source install/setup.bash

# 테스트 실행 (디바이스 없이)
ros2 launch fta_actuators test_led_buzzer.launch.py

# 실제 실행 (디바이스 연결 시)
ros2 launch fta_actuators led_buzzer.launch.py
```

## ?? ROS2 토픽

- `/actions/event` - 액추에이터 제어 명령 (구독)
- `/actuators/status` - 액추에이터 상태 발행

## ?? 상세 문서

자세한 내용은 각 패키지의 README를 참고하세요.
