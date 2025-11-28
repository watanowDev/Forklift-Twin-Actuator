# Patlite LED/Buzzer 제어 모듈

## 개요

이 모듈은 C# LIS 프로젝트의 Patlite LED/Buzzer 제어 로직을 C++/ROS2로 포팅한 것입니다.

## 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                   /actions/event                         │
│              (FTE로부터 액션 명령 수신)                    │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│              PatliteController                           │
│   (액션 → LED/Buzzer 명령 변환)                          │
│                                                           │
│  - get_command_for_action(PatliteAction)                 │
│  - 사전 정의된 패턴 매핑 테이블                            │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│            Hardware Driver Interface                     │
│  (제조사별 드라이버 플러그인)                              │
│                                                           │
│  - NeUsbController (Patlite)                             │
│  - 기타 제조사 드라이버...                                 │
└──────────────────────┬──────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────┐
│                USB Hardware                              │
│           (Patlite LED/Buzzer 장치)                       │
└─────────────────────────────────────────────────────────┘
```

## 주요 컴포넌트

### 1. Enum 타입 정의 (`patlite_types.hpp`)

C# LIS 프로젝트의 enum을 C++로 포팅:
- `LEDColor`: LED 색상 (RED, GREEN, AMBER, BLUE, PURPLE, CYAN 등)
- `LEDPattern`: LED 패턴 (CONTINUOUS, PATTERN1~6)
- `BuzzerPattern`: Buzzer 패턴 (CONTINUOUS, PATTERN1~6)
- `PatliteAction`: 사전 정의된 액션 시나리오 (25가지)

### 2. PatliteController (`patlite_controller.hpp/cpp`)

C# `StatusService_WATA.cs`의 `Pattlite_Buzzer_LED` 함수를 포팅한 클래스입니다.

**주요 기능:**
- `get_command_for_action(PatliteAction)`: 액션을 LED/Buzzer 명령으로 변환
- `initialize_action_map()`: 25가지 사전 정의 패턴 초기화

**매핑 예시:**
```cpp
// DEVICE_ERROR: 빨강 깜빡임 + 긴급 부저
PatliteAction::DEVICE_ERROR 
  → LED(RED, PATTERN2) + Buzzer(PATTERN4, 1회)

// QR_MEASURE_OK: 녹색 연속 + 완료 부저
PatliteAction::QR_MEASURE_OK 
  → LED(GREEN, CONTINUOUS) + Buzzer(CONTINUOUS, 1회)
```

### 3. ROS2 메시지 타입

#### `PatliteAction.msg`
```
uint8 action  # 0~25 범위의 액션 코드
builtin_interfaces/Time timestamp
```

#### `PatliteCommand.msg`
```
uint8 led_color       # LED 색상 코드
uint8 led_pattern     # LED 패턴 코드
uint8 buzzer_pattern  # Buzzer 패턴 코드
int32 buzzer_count    # Buzzer 반복 횟수
builtin_interfaces/Time timestamp
```

## 사용 방법

### 1. 기본 사용 예제

```cpp
#include "fta_actuators/patlite_controller.hpp"
#include <rclcpp/rclcpp.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    controller_ = std::make_unique<PatliteController>(get_logger());
  }

  void trigger_device_error()
  {
    // 디바이스 에러 발생 시
    auto cmd = controller_->get_command_for_action(
      fta_actuators::PatliteAction::DEVICE_ERROR);
    
    // cmd를 하드웨어 드라이버로 전달
    send_to_hardware(cmd);
  }

  void trigger_pickup_success()
  {
    // QR 픽업 성공 시
    auto cmd = controller_->get_command_for_action(
      fta_actuators::PatliteAction::QR_PICKUP);
    
    send_to_hardware(cmd);
  }

private:
  std::unique_ptr<fta_actuators::PatliteController> controller_;
};
```

### 2. ROS2 토픽 연동 예제

```cpp
// /actions/event 토픽 구독하여 액션 수신
subscription_ = create_subscription<fta_interfaces::msg::PatliteAction>(
  "/actions/event",
  rclcpp::QoS(10).reliable(),
  [this](const fta_interfaces::msg::PatliteAction::SharedPtr msg) {
    auto action = static_cast<fta_actuators::PatliteAction>(msg->action);
    auto cmd = controller_->get_command_for_action(action);
    
    // 하드웨어 제어
    execute_command(cmd);
    
    // 상태 발행
    publish_status(cmd);
  });
```

## C# 원본 코드 참조

이 구현은 다음 C# 파일을 기반으로 포팅되었습니다:

- **원본 위치**: `reference/LIS/WATA.LIS/WATA.LIS.Core/Services/ServiceImpl/StatusService_WATA.cs`
- **핵심 함수**: `Pattlite_Buzzer_LED(ePlayBuzzerLed value)`
- **모델 정의**: `reference/LIS/WATA.LIS/WATA.LIS.Core/Model/ErrorCheck/Pattlite_LED_Buzzer_Model.cs`
- **Enum 정의**: `reference/LIS/WATA.LIS/WATA.LIS.Core/Common/Common.cs`

## 액션 목록

| 액션 코드 | 액션 이름 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|----------|----------|---------|---------|------------|------|
| 0 | CONTAINER_OK | GREEN | CONTINUOUS | CONTINUOUS | 컨테이너 정상 |
| 1 | SIZE_CHECK_START | GREEN | PATTERN3 | PATTERN2 | 크기 측정 시작 |
| 2 | SIZE_MEASURE_OK | GREEN | PATTERN6 | PATTERN1 | 크기 측정 완료 |
| 5 | QR_PICKUP | GREEN | PATTERN3 | PATTERN2 | QR 픽업 |
| 6 | QR_MEASURE_OK | GREEN | CONTINUOUS | CONTINUOUS | QR 측정 완료 |
| 7 | NO_QR_PICKUP | PURPLE | PATTERN3 | PATTERN2 | QR 없이 픽업 |
| 10 | SET_ITEM | CYAN | CONTINUOUS | CONTINUOUS | 품목 설정 |
| 17 | DROP | GREEN | CONTINUOUS | OFF | 하역 |
| 23 | DEVICE_ERROR | RED | PATTERN2 | PATTERN4 | 장치 에러 |
| 24 | DEVICE_ERROR_CLEAR | GREEN | CONTINUOUS | CONTINUOUS | 에러 해제 |
| 25 | INVALID_PLACE | RED | PATTERN6 | PATTERN3 | 잘못된 위치 |

*전체 목록은 `patlite_types.hpp` 참조*

## 하드웨어 드라이버 통합

### NeUsbController (C# 원본 사용)

C# 원본 코드에서는 `NeUsbController.dll`을 사용합니다:

```csharp
// C# 원본
NeUsbController.NE_OpenDevice();
NeUsbController.NE_SetLight(LEDColors.Green, LEDPatterns.Continuous);
NeUsbController.NE_SetBuz(BuzzerPatterns.Pattern2, volume, 1);
```

### C++ 포팅 시 고려사항

1. **NeUsbController DLL 래퍼 작성 필요**
   - P/Invoke 또는 C++ DLL 직접 호출
   - USB 통신 프로토콜 분석 필요

2. **대안: libusb 기반 직접 구현**
   - USB 패킷 캡처하여 프로토콜 리버스 엔지니어링
   - 참조: `docs/USB_PACKET_CAPTURE_GUIDE.md`

## 다음 단계

1. ✅ Enum 타입 정의
2. ✅ PatliteController 구현
3. ✅ ROS2 메시지 타입 정의
4. ⏳ 하드웨어 드라이버 인터페이스 설계
5. ⏳ NeUsbController 래퍼 구현
6. ⏳ LED/Buzzer 노드 구현
7. ⏳ 통합 테스트

## 참고 문서

- [FTA 개발 가이드](../../.github/copilot-instructions.md)
- [USB 드라이버 설계](../../docs/USB_DRIVER_DESIGN.md)
- [패키지 구조](../../docs/PACKAGE_STRUCTURE.md)
