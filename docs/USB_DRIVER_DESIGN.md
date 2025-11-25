# 🔌 Patlite USB 드라이버 설계 가이드

## 📋 목차
1. [현재 상황 분석](#1-현재-상황-분석)
2. [USB 드라이버 옵션](#2-usb-드라이버-옵션)
3. [권장 방안: libusb](#3-권장-방안-libusb)
4. [구현 단계](#4-구현-단계)
5. [코드 예제](#5-코드-예제)

---

## 1. 현재 상황 분석

### C# 원본 코드가 사용하는 것
```csharp
// NeUsbController.dll 사용
int result = NeUsbController.NeUsbController.NE_OpenDevice();
NeUsbController.NeUsbController.NE_SetLight(LEDColors.Green, LEDPatterns.Continuous);
NeUsbController.NeUsbController.NE_SetBuz(BuzzerPatterns.Pattern6, volume, count);
NeUsbController.NeUsbController.NE_GetDeviceState(out buzzer, out led, out touch);
```

### 필요한 함수
```
NE_OpenDevice()           - USB 장치 열기
NE_CloseDevice()          - USB 장치 닫기
NE_SetLight(color, pattern) - LED 설정
NE_SetBuz(pattern, volume, count) - Buzzer 설정
NE_GetDeviceState(...)    - 장치 상태 조회
```

### Patlite USB 장치 정보
```bash
# lsusb 명령어로 확인
Bus 001 Device 005: ID 191a:XXXX Patlite Corporation
```
- **Vendor ID**: `0x191a` (Patlite)
- **Product ID**: 장치마다 다름 (확인 필요)
- **인터페이스**: HID 또는 Vendor Specific

---

## 2. USB 드라이버 옵션

### 옵션 1: libusb (⭐ 권장)

**장점:**
- ✅ Linux 표준 USB 라이브러리
- ✅ Ubuntu에 기본 설치됨
- ✅ Low-level 제어 가능
- ✅ 활발한 커뮤니티 및 문서
- ✅ C/C++ API 제공

**단점:**
- ⚠️ USB 프로토콜을 직접 구현해야 함
- ⚠️ Patlite 프로토콜 분석 필요

**설치:**
```bash
sudo apt install libusb-1.0-0-dev
```

---

### 옵션 2: hidapi

**장점:**
- ✅ HID 장치에 특화
- ✅ 크로스 플랫폼 (Windows/Linux/Mac)
- ✅ 간단한 API

**단점:**
- ⚠️ Patlite가 HID 장치인지 확인 필요
- ⚠️ Vendor Specific 장치면 사용 불가

**설치:**
```bash
sudo apt install libhidapi-dev
```

---

### 옵션 3: NeUsbController 리버스 엔지니어링

**장점:**
- ✅ 기존 프로토콜 그대로 사용

**단점:**
- ⚠️ 법적 문제 가능성
- ⚠️ Windows DLL을 Linux에서 사용 불가
- ⚠️ 시간 소모

---

### 옵션 4: NeUsbController Java/C# 소스 찾기

**장점:**
- ✅ 오픈소스라면 프로토콜 명확
- ✅ C++로 포팅만 하면 됨

**단점:**
- ⚠️ 소스 코드가 공개되지 않았을 가능성

**확인 방법:**
```bash
# NuGet 또는 GitHub에서 검색
https://github.com/search?q=NeUsbController
https://www.nuget.org/packages?q=NeUsbController
```

---

## 3. 권장 방안: libusb

### 왜 libusb인가?
1. **표준 라이브러리**: Ubuntu에서 가장 안정적
2. **Full Control**: USB 통신 프로토콜을 완전히 제어
3. **ROS2 호환성**: 다른 ROS2 패키지들도 libusb 사용

### 구현 아키텍처

```
┌─────────────────────────────────────────┐
│   patlite_scenarios.cpp                 │  ← 시나리오 제어 (이미 완성)
│   - execute_scenario()                  │
└────────────────┬────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────┐
│   patlite_driver.cpp (새로 구현)        │  ← USB 통신 레이어
│   - open_device()                       │
│   - set_light(color, pattern)           │
│   - set_buzzer(pattern, volume, count)  │
│   - get_device_state()                  │
│   - close_device()                      │
└────────────────┬────────────────────────┘
                 │
                 ↓ libusb API 호출
┌─────────────────────────────────────────┐
│   libusb-1.0                            │  ← Linux USB 스택
│   - libusb_open()                       │
│   - libusb_control_transfer()           │
│   - libusb_interrupt_transfer()         │
└────────────────┬────────────────────────┘
                 │
                 ↓
┌─────────────────────────────────────────┐
│   Patlite USB LED Buzzer                │  ← 실제 하드웨어
│   Vendor ID: 0x191a                     │
└─────────────────────────────────────────┘
```

---

## 4. 구현 단계

### Phase 1: USB 장치 정보 수집 (1-2시간)

**목표**: Patlite USB 프로토콜 파악

```bash
# 1. USB 장치 연결 후 확인
lsusb -v -d 191a:

# 2. USB 통신 모니터링
sudo modprobe usbmon
sudo wireshark  # USB traffic 캡처

# 3. Windows에서 C# 프로그램 실행 중 USB 패킷 캡처
# USBPcap 또는 Wireshark 사용
```

**수집할 정보:**
- Vendor ID / Product ID
- Endpoint 주소
- Interface Number
- 제어 명령 바이트 구조

---

### Phase 2: libusb 기본 구현 (2-3시간)

**파일 구조:**
```
fta_actuators/
├── include/fta_actuators/
│   ├── patlite_usb_driver.hpp      # libusb wrapper
│   └── patlite_scenarios.hpp       # (기존)
└── src/
    ├── patlite_usb_driver.cpp      # libusb 구현
    └── patlite_scenarios.cpp       # (기존)
```

**핵심 클래스:**
```cpp
class PatliteUsbDriver
{
public:
  PatliteUsbDriver();
  ~PatliteUsbDriver();
  
  // 장치 제어
  bool open_device(uint16_t vendor_id = 0x191a, uint16_t product_id = 0x0000);
  void close_device();
  bool is_open() const;
  
  // LED 제어
  bool set_light(LEDColor color, LEDPattern pattern);
  
  // Buzzer 제어
  bool set_buzzer(BuzzerPattern pattern, int volume, int count);
  
  // 상태 조회
  bool get_device_state(bool& buzzer_state, bool& led_state, bool& touch_state);

private:
  libusb_context* context_;
  libusb_device_handle* device_handle_;
  
  // USB 통신 헬퍼
  bool send_control_message(uint8_t request, uint16_t value, uint16_t index, 
                            uint8_t* data, uint16_t length);
  bool send_interrupt_message(uint8_t endpoint, uint8_t* data, int length);
};
```

---

### Phase 3: 프로토콜 구현 (4-6시간)

**예상 프로토콜 구조:**
```
LED 제어 명령:
┌────────┬────────┬────────┬────────┐
│ CMD    │ COLOR  │ PATTERN│ RESERVED│
│ 0x01   │ 0x04   │ 0x01   │ 0x00    │
└────────┴────────┴────────┴────────┘
  1 byte   1 byte   1 byte   1 byte

Buzzer 제어 명령:
┌────────┬────────┬────────┬────────┬────────┐
│ CMD    │ PATTERN│ VOLUME │ COUNT  │ RESERVED│
│ 0x02   │ 0x06   │ 50     │ 3      │ 0x00    │
└────────┴────────┴────────┴────────┴────────┘
  1 byte   1 byte   1 byte   1 byte   1 byte
```

**구현 예제:**
```cpp
bool PatliteUsbDriver::set_light(LEDColor color, LEDPattern pattern)
{
  if (!is_open()) return false;
  
  uint8_t cmd[4] = {
    0x01,                              // 명령: LED 제어
    static_cast<uint8_t>(color),       // 색상
    static_cast<uint8_t>(pattern),     // 패턴
    0x00                               // Reserved
  };
  
  // Control Transfer 또는 Interrupt Transfer 사용
  return send_control_message(0x09, 0x0200, 0x0000, cmd, sizeof(cmd));
}
```

---

### Phase 4: 시나리오와 통합 (1시간)

**`patlite_scenarios.cpp` 수정:**
```cpp
#include "fta_actuators/patlite_usb_driver.hpp"

class PatliteController
{
private:
  PatliteUsbDriver usb_driver_;  // USB 드라이버 추가
  
public:
  PatliteController() {
    // USB 장치 열기
    if (!usb_driver_.open_device()) {
      // 에러 처리
    }
  }
  
  void execute_command(const PatliteCommand& command) {
    // USB 드라이버로 실제 제어
    usb_driver_.set_light(command.led_color, command.led_pattern);
    usb_driver_.set_buzzer(command.buzzer_pattern, 50, command.buzzer_count);
  }
};
```

---

## 5. 코드 예제

### 5.1 libusb 기본 예제

```cpp
#include <libusb-1.0/libusb.h>
#include <iostream>

int main()
{
  libusb_context* ctx = nullptr;
  libusb_device_handle* handle = nullptr;
  
  // libusb 초기화
  int ret = libusb_init(&ctx);
  if (ret < 0) {
    std::cerr << "libusb 초기화 실패: " << libusb_error_name(ret) << std::endl;
    return 1;
  }
  
  // Patlite 장치 열기 (Vendor ID: 0x191a)
  handle = libusb_open_device_with_vid_pid(ctx, 0x191a, 0x0000);
  if (!handle) {
    std::cerr << "Patlite 장치를 찾을 수 없습니다" << std::endl;
    libusb_exit(ctx);
    return 1;
  }
  
  std::cout << "Patlite 장치 연결 성공!" << std::endl;
  
  // 장치 제어 (예: LED 켜기)
  uint8_t data[4] = {0x01, 0x04, 0x01, 0x00}; // 초록색 연속
  ret = libusb_control_transfer(
    handle,
    0x21,          // bmRequestType: Host to Device, Class, Interface
    0x09,          // bRequest: SET_REPORT
    0x0200,        // wValue: Report Type (Output)
    0x0000,        // wIndex: Interface 0
    data,
    sizeof(data),
    1000           // timeout (ms)
  );
  
  if (ret < 0) {
    std::cerr << "LED 제어 실패: " << libusb_error_name(ret) << std::endl;
  } else {
    std::cout << "LED 제어 성공! (" << ret << " bytes)" << std::endl;
  }
  
  // 정리
  libusb_close(handle);
  libusb_exit(ctx);
  
  return 0;
}
```

**컴파일:**
```bash
g++ -o patlite_test patlite_test.cpp -lusb-1.0
sudo ./patlite_test  # USB 접근 권한 필요
```

---

### 5.2 USB 권한 설정

**udev 규칙 생성:**
```bash
# /etc/udev/rules.d/99-patlite.rules
sudo nano /etc/udev/rules.d/99-patlite.rules
```

**내용:**
```
# Patlite USB LED Buzzer
SUBSYSTEM=="usb", ATTRS{idVendor}=="191a", MODE="0666", GROUP="plugdev"
```

**적용:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# USB 재연결
```

---

### 5.3 USB 정보 확인 스크립트

```bash
#!/bin/bash
# check_patlite_usb.sh

echo "===== Patlite USB 장치 검색 ====="
lsusb | grep -i patlite || lsusb | grep "191a"

echo ""
echo "===== 상세 정보 ====="
lsusb -v -d 191a: 2>/dev/null

echo ""
echo "===== Endpoint 정보 ====="
lsusb -v -d 191a: 2>/dev/null | grep -A 10 "Endpoint Descriptor"
```

---

## 6. 트러블슈팅

### 문제 1: "Permission denied"
```bash
# 해결 방법 1: sudo로 실행
sudo ./patlite_test

# 해결 방법 2: udev 규칙 설정 (위 참고)

# 해결 방법 3: 사용자를 plugdev 그룹에 추가
sudo usermod -a -G plugdev $USER
# 로그아웃 후 재로그인
```

### 문제 2: "Device not found"
```bash
# USB 연결 확인
lsusb | grep -i patlite

# dmesg로 연결 로그 확인
dmesg | tail -20

# 다른 USB 포트 시도
```

### 문제 3: "Control transfer failed"
```bash
# 장치가 다른 드라이버에 의해 사용 중일 수 있음
# 커널 드라이버 detach 필요
libusb_detach_kernel_driver(handle, 0);
```

---

## 7. 다음 단계

### 즉시 할 수 있는 것
1. ✅ **USB 장치 정보 수집**
   ```bash
   ssh wmszz@172.30.1.102
   lsusb -v -d 191a: > patlite_usb_info.txt
   ```

2. ✅ **libusb 예제 테스트**
   ```bash
   # libusb 설치
   sudo apt install libusb-1.0-0-dev
   
   # 예제 컴파일 및 실행
   g++ -o test test.cpp -lusb-1.0
   sudo ./test
   ```

### 원격 PC 사용 가능할 때
3. ⏳ **프로토콜 분석**
   - Windows에서 C# 프로그램 실행
   - USB 패킷 캡처 (Wireshark + USBPcap)
   - 명령 바이트 구조 파악

4. ⏳ **드라이버 구현**
   - `patlite_usb_driver.hpp/cpp` 작성
   - 시나리오와 통합
   - 실제 하드웨어 테스트

---

## 📚 참고 자료

- **libusb 공식 문서**: https://libusb.info/
- **libusb API**: https://libusb.sourceforge.io/api-1.0/
- **USB HID 프로토콜**: https://www.usb.org/hid
- **Wireshark USB 캡처**: https://wiki.wireshark.org/CaptureSetup/USB

---

## 💡 요약

**권장 방안**: **libusb 사용**

**구현 순서**:
1. USB 장치 정보 수집 (lsusb)
2. libusb 기본 예제 테스트
3. 프로토콜 분석 (Windows USB 캡처)
4. PatliteUsbDriver 클래스 구현
5. patlite_scenarios와 통합
6. 실제 하드웨어 테스트

**예상 소요 시간**: 8-12시간 (프로토콜 분석 포함)

**즉시 시작 가능**: USB 정보 수집 및 libusb 예제 테스트
