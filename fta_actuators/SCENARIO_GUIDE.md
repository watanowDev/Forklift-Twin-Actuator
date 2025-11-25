# 🎯 Patlite 시나리오 제어 가이드

## 📋 개요

C# 프로젝트 `StatusService_WATA.cs`의 `Pattlite_Buzzer_LED` 함수를 C++로 포팅하여, **상황별 LED/Buzzer 제어 시나리오**를 함수화했습니다.

---

## 🏗️ 구조

```
fta_actuators/
├── include/fta_actuators/
│   └── patlite_scenarios.hpp    # 시나리오 정의 및 인터페이스
└── src/
    └── patlite_scenarios.cpp    # 시나리오 구현

fta_beginner/
└── src/
    └── test_scenarios.cpp       # 테스트 프로그램
```

---

## 📊 지원하는 시나리오 (21개)

### 1. 컨테이너 관련
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `CONTAINER_OK` | 초록 | 연속 | 연속 | 컨테이너 정상 |

### 2. 사이즈 측정
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `SIZE_CHECK_START` | 초록 | Pattern3 | Pattern2 | 측정 시작 |
| `SIZE_MEASURE_OK` | 초록 | Pattern6 | Pattern1 | 측정 완료 (QR 있음) |
| `NO_QR_SIZE_MEASURE_OK` | 보라 | Pattern6 | Pattern1 | 측정 완료 (QR 없음) |

### 3. QR 코드 관련
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `QR_PICKUP` | 초록 | Pattern3 | Pattern2 | QR 픽업 |
| `QR_MEASURE_OK` | 초록 | 연속 | 연속 | QR 측정 완료 |
| `NO_QR_PICKUP` | 보라 | Pattern3 | Pattern2 | QR 없이 픽업 |
| `NO_QR_MEASURE_OK` | 보라 | 연속 | 연속 | QR 없이 측정 완료 |
| `NO_QR_CHECK_COMPLETE` | 보라 | 연속 | 연속 | QR 없이 체크 완료 |

### 4. 앱 물류 선택 (SET_ITEM)
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `SET_ITEM` | 하늘색 | 연속 | 연속 | 앱에서 물류 선택 |
| `SET_ITEM_NORMAL` | 하늘색 | 연속 | OFF | 앱 물류 (일반) |
| `SET_ITEM_PICKUP` | 하늘색 | Pattern3 | Pattern2 | 앱 물류 픽업 |
| `SET_ITEM_SIZE_CHECK_START` | 하늘색 | Pattern3 | Pattern2 | 앱 물류 측정 시작 |
| `SET_ITEM_MEASURE_OK` | 하늘색 | 연속 | 연속 | 앱 물류 측정 완료 |
| `SET_ITEM_CHECK_COMPLETE` | 하늘색 | 연속 | 연속 | 앱 물류 체크 완료 |

### 5. 기타 작업
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `CLEAR_ITEM` | 하늘색 | 연속 | 연속 | 아이템 클리어 |
| `DROP` | 초록 | 연속 | OFF | 드롭 |
| `CHECK_COMPLETE` | 초록 | 연속 | 연속 | 체크 완료 |

### 6. 에러 및 경고
| 시나리오 | LED 색상 | LED 패턴 | Buzzer 패턴 | 설명 |
|---------|---------|---------|------------|------|
| `DEVICE_ERROR` | 빨강 | Pattern2 | Pattern4 | 디바이스 에러 |
| `DEVICE_ERROR_CLEAR` | 초록 | 연속 | 연속 | 에러 해제 |
| `INVALID_PLACE` | 빨강 | Pattern6 | Pattern3 | 잘못된 위치 (보행자 감지) |

---

## 🎨 색상 의미

- **🟢 GREEN (초록)**: 정상 작업 (QR 코드 있음)
- **🟣 PURPLE (보라)**: QR 코드 없이 작업
- **🔵 SKYBLUE (하늘)**: 앱에서 물류 선택
- **🔴 RED (빨강)**: 에러 또는 경고

---

## 💻 사용 방법

### 1. 빌드

```bash
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fta_beginner
source install/setup.bash
```

### 2. 테스트 프로그램 실행

```bash
# 전체 시나리오 목록 보기
ros2 run fta_beginner test_scenarios list

# 상황별 제어 함수 테스트
ros2 run fta_beginner test_scenarios test

# 문자열 변환 테스트
ros2 run fta_beginner test_scenarios string

# ROS2 사용 예제 보기
ros2 run fta_beginner test_scenarios examples

# 색상 참고표
ros2 run fta_beginner test_scenarios colors

# 모든 테스트 실행
ros2 run fta_beginner test_scenarios
```

### 3. ROS2 토픽으로 제어

**방법 1: 시나리오 이름으로 직접 제어**
```bash
# 컨테이너 OK
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'CONTAINER_OK'}"

# 디바이스 에러
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'DEVICE_ERROR'}"

# QR 픽업
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'QR_PICKUP'}"
```

**방법 2: 컨텍스트 기반 제어** (나중에 구현)
```bash
# 측정 시작 (앱 물류 선택)
ros2 topic pub --once /patlite/context std_msgs/msg/String \
  "{data: '{\"action\": \"start_measuring\", \"set_item\": true, \"qr_code\": \"\"}'}"

# 측정 완료 (QR 있음)
ros2 topic pub --once /patlite/context std_msgs/msg/String \
  "{data: '{\"action\": \"finish_measuring\", \"set_item\": false, \"qr_code\": \"wata-12345\"}'}"
```

---

## 🔧 코드 사용 예제

### C++ 코드에서 사용

```cpp
#include "fta_actuators/patlite_scenarios.hpp"

using namespace fta_actuators;

// 1. 시나리오 직접 실행
PatliteController controller;
controller.execute_scenario(PatliteScenario::CONTAINER_OK);

// 2. 상황별 제어 함수 사용
bool set_item = true;        // 앱에서 물류 선택 여부
std::string qr_code = "";    // QR 코드
bool is_error = false;       // 에러 상태
bool func_off = false;       // 기능 OFF 여부

// 측정 시작
controller.start_measuring_buzzer(set_item, qr_code, is_error, func_off);

// 측정 완료
controller.finish_measuring_buzzer(set_item, qr_code, is_error, func_off);

// 보행자 감지 경고
controller.alert_detect_person(func_off);

// 3. 시나리오 정보 조회
PatliteScenarioMapper mapper;
PatliteCommand cmd = mapper.get_command(PatliteScenario::QR_PICKUP);

std::cout << "LED Color: " << led_color_to_string(cmd.led_color) << std::endl;
std::cout << "LED Pattern: " << led_pattern_to_string(cmd.led_pattern) << std::endl;
std::cout << "Buzzer Pattern: " << buzzer_pattern_to_string(cmd.buzzer_pattern) << std::endl;
std::cout << "Buzzer Count: " << cmd.buzzer_count << std::endl;
```

---

## 📝 C# 원본 함수와의 대응

| C# 함수 | C++ 함수 | 설명 |
|---------|---------|------|
| `Pattlite_Buzzer_LED(ePlayBuzzerLed)` | `execute_scenario(PatliteScenario)` | 시나리오 실행 |
| `StartMeasuringBuzzer()` | `start_measuring_buzzer(...)` | 측정 시작 |
| `FinishMeasuringBuzzer()` | `finish_measuring_buzzer(...)` | 측정 완료 |
| `FinishMeasuringSize()` | `finish_measuring_size(...)` | 사이즈 측정 완료 |
| `AlertDetectPerson()` | `alert_detect_person(...)` | 보행자 감지 |
| `CheckExceptionBuzzer()` | `check_exception_buzzer(...)` | 예외 체크 |

---

## 🎯 다음 단계

1. **USB 드라이버 연동**
   - NeUsbController C++ 포팅 또는
   - libusb 직접 사용

2. **ROS2 노드 구현**
   - `/patlite/scenario` 토픽 구독
   - `/patlite/context` 토픽 구독 (컨텍스트 기반)
   - `/actuators/status` 발행

3. **실제 하드웨어 테스트**
   - Patlite USB 연결
   - 각 시나리오 실행 확인

4. **FTE 모듈과 통합**
   - 전체 시스템에서 이벤트 기반 제어

---

## 💡 팁

### 대소문자 구분 없음
시나리오 이름은 대소문자를 구분하지 않습니다:
```bash
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'container_ok'}"
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'CONTAINER_OK'}"
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'Container_Ok'}"
# 모두 동일하게 작동
```

### 시나리오 목록 확인
모든 사용 가능한 시나리오를 보려면:
```bash
ros2 run fta_beginner test_scenarios list
```

### 테스트 시뮬레이션
실제 하드웨어 없이도 시나리오 로직을 테스트할 수 있습니다:
```bash
ros2 run fta_beginner test_scenarios test
```

---

## 📚 참고

- C# 원본: `C:\Users\wmszz\source\repos\LIS\WATA.LIS\StatusService_WATA.cs`
- 헤더 파일: `fta_actuators/include/fta_actuators/patlite_scenarios.hpp`
- 구현 파일: `fta_actuators/src/patlite_scenarios.cpp`
- 테스트 프로그램: `fta_beginner/src/test_scenarios.cpp`
