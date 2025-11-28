# 개발 규칙 및 가이드라인

## 코딩 스타일

### C++
- **표준**: C++17 이상
- **네이밍**:
  - 클래스: `PascalCase` (예: `LEDBuzzerNode`)
  - 함수/변수: `snake_case` (예: `initialize_hardware`)
  - 상수: `UPPER_SNAKE_CASE` (예: `MAX_RETRY_COUNT`)
  - 네임스페이스: `lowercase` (예: `fta_actuators`)

### Python
- **표준**: PEP 8
- **네이밍**: 
  - 클래스: `PascalCase`
  - 함수/변수: `snake_case`

### ROS2 네이밍
- **패키지명**: `fta_` 접두사 + `snake_case` (예: `fta_actuators`)
- **노드명**: `snake_case` + `_node` (예: `led_buzzer_node`)
- **토픽명**: `/` 구분자 + `snake_case` (예: `/actions/event`)

## 에러 처리

### 1. 하드웨어 통신 에러
```cpp
const int MAX_RETRY = 3;
const int RETRY_DELAY_MS = 100;

bool sendCommand(const Command& cmd) {
    for (int i = 0; i < MAX_RETRY; i++) {
        if (hardware_->send(cmd)) {
            return true;
        }
        RCLCPP_WARN(get_logger(), "Retry %d/%d", i+1, MAX_RETRY);
        std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_DELAY_MS));
    }
    RCLCPP_ERROR(get_logger(), "Failed to send command after %d retries", MAX_RETRY);
    return false;
}
```

### 2. 상태 보고
모든 에러는 `/actuators/status` 토픽으로 발행:
```cpp
status_msg.success = false;
status_msg.error_code = ERROR_HARDWARE_COMM_FAILED;
status_msg.error_message = "Failed to communicate with LED controller";
status_pub_->publish(status_msg);
```

### 3. 로깅
FTL(Logger)로 에러 전송:
```cpp
RCLCPP_ERROR(get_logger(), "Critical error: %s", error_msg.c_str());
```

## 성능 최적화

### 1. 토픽 발행 빈도 제한
```cpp
// 불필요한 고빈도 발행 방지
const auto MIN_PUBLISH_INTERVAL = std::chrono::milliseconds(100);
auto now = this->now();
if (now - last_publish_time_ < MIN_PUBLISH_INTERVAL) {
    return;  // Skip publishing
}
```

### 2. 리소스 관리
```cpp
// 사용하지 않는 드라이버는 로드하지 않음
if (config_.enable_led) {
    led_driver_ = std::make_unique<LEDDriver>();
}
```

## 테스트 가이드

### 단위 테스트
각 액추에이터 노드는 독립적으로 테스트 가능:
```bash
# Mock 하드웨어로 테스트
ros2 run fta_actuators led_buzzer_node --ros-args -p use_mock_hardware:=true
```

### 통합 테스트
```bash
# FTE 없이 직접 명령 전송 테스트
ros2 topic pub /actions/event fta_msgs/msg/ActuatorCommand "{...}"
```

## Git 워크플로우

### 브랜치 전략
- `main`: 안정 버전
- `develop`: 개발 브랜치
- `feature/*`: 기능 개발
- `bugfix/*`: 버그 수정

### 커밋 메시지
```
<type>: <subject>

[optional body]

<type>:
- feat: 새 기능
- fix: 버그 수정
- docs: 문서
- refactor: 리팩토링
- test: 테스트
```

예시:
```
feat: LED Buzzer 제어 노드 추가

- LED on/off 제어 구현
- Buzzer 주파수 제어 추가
- /actions/event 토픽 구독 구현
```

## 문서화

### 코드 주석
```cpp
/**
 * @brief LED와 Buzzer를 제어하는 ROS2 노드
 * 
 * /actions/event 토픽을 구독하여 LED 및 Buzzer를 제어하고,
 * 제어 결과를 /actuators/status 토픽으로 발행합니다.
 */
class LEDBuzzerNode : public rclcpp::Node {
    // ...
};
```

### README 업데이트
새 액추에이터 추가 시 README.md 업데이트 필수
