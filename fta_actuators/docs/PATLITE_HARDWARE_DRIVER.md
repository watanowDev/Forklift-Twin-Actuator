# Patlite LED/Buzzer 하드웨어 드라이버 설계

## 개요

Patlite LED/Buzzer 하드웨어 제어를 위한 추상화 계층 및 드라이버 구현입니다.
제조사별 드라이버를 플러그인 형태로 교체할 수 있도록 설계되었습니다.

## 아키텍처

```
┌────────────────────────────────────────────────────────────┐
│            PatliteController (제어 로직)                     │
│       - C# StatusService 포팅                               │
│       - 25가지 사전 정의 액션 매핑                            │
└──────────────────────┬─────────────────────────────────────┘
                       │
                       ▼
┌────────────────────────────────────────────────────────────┐
│      PatliteHardwareInterface (추상 인터페이스)              │
│       - LED 제어: set_led(color, pattern)                   │
│       - Buzzer 제어: set_buzzer(pattern, volume, count)     │
│       - 상태 조회: get_device_state()                        │
└──────────────────────┬─────────────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│ USB Direct  │ │ NeUsbCtrl   │ │ Mock Driver │
│ (TODO)      │ │ DLL (TODO)  │ │ (구현 완료) │
└─────────────┘ └─────────────┘ └─────────────┘
```

## 인터페이스 정의

### PatliteHardwareInterface

모든 Patlite 드라이버가 구현해야 하는 공통 인터페이스입니다.

```cpp
class PatliteHardwareInterface
{
public:
  virtual bool open(const std::string& device_path = "") = 0;
  virtual void close() = 0;
  virtual bool is_connected() const = 0;
  
  virtual bool set_led(LEDColor color, LEDPattern pattern) = 0;
  virtual bool set_buzzer(BuzzerPattern pattern, int volume, int count) = 0;
  virtual bool execute_command(const PatliteCommand& cmd, int volume) = 0;
  
  virtual bool get_device_state(
    bool& buzzer_state, bool& led_state, bool& touch_state) const = 0;
  
  virtual std::string get_device_info() const = 0;
  virtual std::string get_driver_name() const = 0;
};
```

### PatliteHardwareFactory

드라이버 생성을 담당하는 팩토리 클래스입니다.

```cpp
class PatliteHardwareFactory
{
public:
  enum class DriverType {
    USB_DIRECT,  // libusb 기반 직접 제어
    NE_DLL,      // NeUsbController.dll 래퍼
    MOCK         // 테스트용 Mock
  };
  
  static std::unique_ptr<PatliteHardwareInterface> create_driver(
    DriverType type = DriverType::USB_DIRECT);
};
```

## 구현된 드라이버

### 1. Mock Driver (구현 완료)

하드웨어 없이 테스트할 수 있는 Mock 드라이버입니다.

**특징:**
- 실제 하드웨어 불필요
- 명령을 콘솔로 출력
- CI/CD 및 개발 환경에서 유용

**사용법:**
```cpp
auto driver = PatliteHardwareFactory::create_driver(
  PatliteHardwareFactory::DriverType::MOCK);

if (driver->open()) {
  driver->set_led(LEDColor::GREEN, LEDPattern::CONTINUOUS);
  driver->set_buzzer(BuzzerPattern::PATTERN2, 5, 1);
  driver->close();
}
```

**출력 예시:**
```
[PatliteMock] Opening device: default
[PatliteMock] LED: color=GREEN (2), pattern=CONTINUOUS (1)
[PatliteMock] Buzzer: pattern=PATTERN2 (3), volume=5, count=1
[PatliteMock] Closing device
```

## TODO: 실제 하드웨어 드라이버

### 2. USB Direct Driver (예정)

libusb를 사용한 직접 USB 통신 구현입니다.

**구현 계획:**
1. USB 패킷 캡처 (Wireshark, usbmon)
2. 프로토콜 리버스 엔지니어링
3. libusb-1.0 API 사용
4. USB VID/PID 식별
5. 제어 명령 패킷 구조 파악

**참조 문서:**
- `docs/USB_PACKET_CAPTURE_GUIDE.md`
- `docs/USB_DRIVER_DESIGN.md`

**예상 구현:**
```cpp
class PatliteUsbDirectDriver : public PatliteHardwareInterface
{
private:
  libusb_device_handle* device_handle_;
  uint16_t vendor_id_;
  uint16_t product_id_;
  
  bool send_control_transfer(const uint8_t* data, size_t len);
  bool parse_device_response(uint8_t* buffer, size_t len);
};
```

### 3. NeUsbController DLL Wrapper (예정)

C# 원본에서 사용하는 NeUsbController.dll 래퍼입니다.

**구현 계획:**
1. DLL 로드 (dlopen 또는 LoadLibrary)
2. 함수 포인터 획득
3. C++ → C 인터페이스 래핑

**예상 구현:**
```cpp
class PatliteNeDllDriver : public PatliteHardwareInterface
{
private:
  void* dll_handle_;
  
  // DLL 함수 포인터
  typedef int (*NE_OpenDevice_t)();
  typedef int (*NE_SetLight_t)(int color, int pattern);
  typedef int (*NE_SetBuz_t)(int pattern, int volume, int count);
  
  NE_OpenDevice_t NE_OpenDevice;
  NE_SetLight_t NE_SetLight;
  NE_SetBuz_t NE_SetBuz;
  
  bool load_dll(const std::string& dll_path);
  void unload_dll();
};
```

## 테스트

### 단위 테스트
```bash
# Controller 테스트 (액션 매핑)
ros2 run fta_actuators test_patlite_controller

# 통합 테스트 (Controller + Hardware)
ros2 run fta_actuators test_patlite_integration
```

### 테스트 결과
```
=== Patlite 통합 테스트 시작 ===
[PatliteMock] Opening device: default
Driver: mock
Info: Patlite Mock Driver v1.0 (No Hardware)

[시나리오: 정상 시작]
[PatliteMock] LED: color=GREEN (2), pattern=CONTINUOUS (1)
[PatliteMock] Buzzer: pattern=CONTINUOUS (1), volume=5, count=1

... (중략) ...

=== 통합 테스트 완료 ===
모든 테스트가 성공적으로 완료되었습니다!
```

## C# 원본 참조

이 구현은 다음 C# 코드를 기반으로 합니다:

### 원본 파일
```
reference/LIS/WATA.LIS/
├── Modules/WATA.LIS.INDICATOR.LED/StatusLED/
│   └── Patlite_LED_Buzzer.cs
└── WATA.LIS.Core/
    ├── Model/ErrorCheck/Pattlite_LED_Buzzer_Model.cs
    └── Services/ServiceImpl/StatusService_WATA.cs
```

### C# 원본 주요 API
```csharp
// NeUsbController DLL 호출
NeUsbController.NE_OpenDevice();
NeUsbController.NE_SetLight(LEDColors.Green, LEDPatterns.Continuous);
NeUsbController.NE_SetBuz(BuzzerPatterns.Pattern2, volume, 1);
NeUsbController.NE_GetDeviceState(out buzzer, out led, out touch);
```

## 확장 가능성

### 다른 제조사 지원
인터페이스를 구현하여 다른 제조사의 LED/Buzzer도 지원 가능:
```cpp
class GenericLedBuzzerDriver : public PatliteHardwareInterface {
  // 다른 제조사 프로토콜 구현
};
```

### 네트워크 제어
TCP/IP 기반 원격 제어도 가능:
```cpp
class PatliteNetworkDriver : public PatliteHardwareInterface {
  // REST API 또는 TCP 소켓 통신
};
```

## 다음 단계

1. ✅ Mock Driver 구현 완료
2. ⏳ USB 패킷 캡처 및 프로토콜 분석
3. ⏳ USB Direct Driver 구현
4. ⏳ NeUsbController DLL Wrapper 구현
5. ⏳ 실제 하드웨어 테스트

## 참고 문서

- [Patlite 모듈 개요](PATLITE.md)
- [USB 드라이버 설계](../../docs/USB_DRIVER_DESIGN.md)
- [USB 패킷 캡처 가이드](../../docs/USB_PACKET_CAPTURE_GUIDE.md)
- [디렉토리 구조](DIRECTORY_STRUCTURE.md)
