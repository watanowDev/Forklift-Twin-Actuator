# 🎉 Patlite 시나리오 함수화 완료!

## ✅ 완료된 작업

### 1. C# 코드 분석 및 포팅
- ✅ `StatusService_WATA.cs`의 `Pattlite_Buzzer_LED` 함수 분석
- ✅ 21개 상황별 시나리오 추출
- ✅ C++로 완벽히 포팅

### 2. 생성된 파일
| 파일 | 설명 | 라인 수 |
|------|------|---------|
| `patlite_scenarios.hpp` | 헤더 파일 (시나리오 정의) | ~190줄 |
| `patlite_scenarios.cpp` | 구현 파일 (시나리오 매핑 및 제어 로직) | ~400줄 |
| `test_scenarios.cpp` | 테스트 프로그램 | ~280줄 |
| `SCENARIO_GUIDE.md` | 사용 가이드 | ~300줄 |

### 3. 주요 기능

#### 📋 21개 시나리오 지원
```cpp
enum class PatliteScenario {
  CONTAINER_OK,              // 컨테이너 정상
  SIZE_CHECK_START,          // 측정 시작
  SIZE_MEASURE_OK,           // 측정 완료 (QR 있음)
  NO_QR_SIZE_MEASURE_OK,     // 측정 완료 (QR 없음)
  QR_PICKUP,                 // QR 픽업
  QR_MEASURE_OK,             // QR 측정 완료
  NO_QR_PICKUP,              // QR 없이 픽업
  NO_QR_MEASURE_OK,          // QR 없이 측정 완료
  SET_ITEM,                  // 앱에서 물류 선택
  SET_ITEM_NORMAL,           // 앱 물류 (일반)
  SET_ITEM_PICKUP,           // 앱 물류 픽업
  SET_ITEM_SIZE_CHECK_START, // 앱 물류 측정 시작
  SET_ITEM_MEASURE_OK,       // 앱 물류 측정 완료
  CLEAR_ITEM,                // 아이템 클리어
  DROP,                      // 드롭
  DEVICE_ERROR,              // 디바이스 에러
  DEVICE_ERROR_CLEAR,        // 에러 해제
  CHECK_COMPLETE,            // 체크 완료
  NO_QR_CHECK_COMPLETE,      // QR 없이 체크 완료
  SET_ITEM_CHECK_COMPLETE,   // 앱 물류 체크 완료
  INVALID_PLACE              // 잘못된 위치
};
```

#### 🎮 상황별 제어 함수
C# 원본 함수를 그대로 재현:
```cpp
class PatliteController {
  // C#: StartMeasuringBuzzer()
  void start_measuring_buzzer(bool set_item, string qr_code, bool is_error, bool func_off);
  
  // C#: FinishMeasuringBuzzer()
  void finish_measuring_buzzer(bool set_item, string qr_code, bool is_error, bool func_off);
  
  // C#: FinishMeasuringSize()
  void finish_measuring_size(string qr_code, bool func_off);
  
  // C#: AlertDetectPerson()
  void alert_detect_person(bool func_off);
  
  // C#: CheckExceptionBuzzer()
  void check_exception_buzzer(bool set_item, string qr_code, bool func_off);
};
```

#### 🔍 헬퍼 함수
```cpp
// enum ↔ 문자열 변환
std::string led_color_to_string(LEDColor color);
LEDColor string_to_led_color(const std::string& str);
std::string scenario_to_string(PatliteScenario scenario);
PatliteScenario string_to_scenario(const std::string& str);
```

---

## 🎯 사용 예제

### 1. 간단한 시나리오 실행
```cpp
#include "fta_actuators/patlite_scenarios.hpp"

PatliteController controller;
controller.execute_scenario(PatliteScenario::CONTAINER_OK);
```

### 2. 상황별 제어
```cpp
// 측정 시작 (앱에서 물류 선택, QR 없음, 에러 없음, 기능 ON)
controller.start_measuring_buzzer(true, "", false, false);

// 측정 완료 (QR 코드 있음)
controller.finish_measuring_buzzer(false, "wata-12345", false, false);

// 보행자 감지 경고
controller.alert_detect_person(false);
```

### 3. ROS2 토픽으로 제어
```bash
# 시나리오 직접 실행
ros2 topic pub --once /patlite/scenario std_msgs/msg/String "{data: 'DEVICE_ERROR'}"

# 테스트 프로그램 실행
ros2 run fta_beginner test_scenarios list   # 전체 목록
ros2 run fta_beginner test_scenarios test   # 제어 함수 테스트
```

---

## 📊 시나리오 매핑 예시

| 시나리오 | LED | Buzzer | 사용 상황 |
|---------|-----|--------|----------|
| `CONTAINER_OK` | 🟢 연속 | 연속 1회 | 컨테이너 정상 확인 |
| `QR_PICKUP` | 🟢 Pattern3 | Pattern2 1회 | QR 코드로 픽업 |
| `NO_QR_PICKUP` | 🟣 Pattern3 | Pattern2 1회 | QR 없이 픽업 |
| `SET_ITEM_PICKUP` | 🔵 Pattern3 | Pattern2 1회 | 앱 물류 픽업 |
| `DEVICE_ERROR` | 🔴 Pattern2 | Pattern4 1회 | 에러 발생 |
| `INVALID_PLACE` | 🔴 Pattern6 | Pattern3 1회 | 보행자 감지 경고 |

**색상 의미:**
- 🟢 GREEN: 정상 (QR 있음)
- 🟣 PURPLE: QR 없이 작업
- 🔵 SKYBLUE: 앱 물류 선택
- 🔴 RED: 에러/경고

---

## 🚀 빌드 및 테스트

### 빌드
```bash
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fta_beginner
source install/setup.bash
```

### 테스트
```bash
# 1. 전체 시나리오 목록 보기
ros2 run fta_beginner test_scenarios list

# 2. 상황별 제어 함수 테스트
ros2 run fta_beginner test_scenarios test

# 3. 문자열 변환 테스트
ros2 run fta_beginner test_scenarios string

# 4. ROS2 사용 예제
ros2 run fta_beginner test_scenarios examples

# 5. 색상 참고표
ros2 run fta_beginner test_scenarios colors

# 6. 모든 테스트 실행
ros2 run fta_beginner test_scenarios
```

---

## 🔄 C# vs C++ 비교

### C# 원본
```csharp
private void Pattlite_Buzzer_LED(ePlayBuzzerLed value)
{
    if (value == ePlayBuzzerLed.CONTAINER_OK)
    {
        Pattlite_LED_Buzzer_Model model = new Pattlite_LED_Buzzer_Model();
        model.LED_Pattern = eLEDPatterns.Continuous;
        model.LED_Color = eLEDColors.Green;
        model.BuzzerPattern = eBuzzerPatterns.Continuous;
        model.BuzzerCount = 1;
        _eventAggregator.GetEvent<Pattlite_StatusLED_Event>().Publish(model);
    }
    // ... 21개 시나리오
}
```

### C++ 포팅
```cpp
// 시나리오 매핑 (초기화 시 한 번만)
scenario_map_[PatliteScenario::CONTAINER_OK] = 
  PatliteCommand(LEDColor::GREEN, LEDPattern::CONTINUOUS, 
                 BuzzerPattern::CONTINUOUS, 1);

// 실행 (간단!)
controller.execute_scenario(PatliteScenario::CONTAINER_OK);
```

---

## ⏭️ 다음 단계

### 1. USB 드라이버 연동 (우선순위 높음)
- [ ] NeUsbController C++ 포팅 OR
- [ ] libusb 직접 사용
- [ ] `execute_command()` 함수 구현

### 2. ROS2 노드 구현
- [ ] `/patlite/scenario` 토픽 구독 노드
- [ ] `/patlite/context` 토픽 구독 (컨텍스트 기반)
- [ ] `/actuators/status` 발행

### 3. 실제 하드웨어 테스트
- [ ] Patlite USB 연결
- [ ] 각 시나리오 실행 확인
- [ ] 색상 및 패턴 검증

### 4. FTE 모듈 통합
- [ ] 이벤트 시스템 연동
- [ ] 전체 워크플로우 테스트

---

## 📚 문서

- **사용 가이드**: [`SCENARIO_GUIDE.md`](./SCENARIO_GUIDE.md)
- **헤더 파일**: `fta_actuators/include/fta_actuators/patlite_scenarios.hpp`
- **구현 파일**: `fta_actuators/src/patlite_scenarios.cpp`
- **테스트 코드**: `fta_beginner/src/test_scenarios.cpp`

---

## 🎉 요약

✅ **21개 시나리오** 완벽 포팅  
✅ **상황별 제어 함수** 5개 구현  
✅ **테스트 프로그램** 준비  
✅ **ROS2 통합** 준비 완료  

이제 USB 드라이버만 연동하면 실제 하드웨어를 제어할 수 있습니다! 🚀
