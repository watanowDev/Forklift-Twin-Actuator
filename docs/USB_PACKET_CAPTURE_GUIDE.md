# 🔬 USB 패킷 캡처 가이드

Patlite USB 장치의 통신 프로토콜을 분석하기 위한 USB 패킷 캡처 방법입니다.

---

## 📋 준비물

### Windows PC (필수)
- C# 프로그램이 동작하는 환경
- Patlite USB 장치 연결

### 설치 프로그램
1. **USBPcap** - USB 패킷 캡처 드라이버
   - 다운로드: https://desowin.org/usbpcap/
   - 설치 후 재부팅 권장

2. **Wireshark** - 패킷 분석 도구
   - 다운로드: https://www.wireshark.org/
   - 최신 버전 사용 권장

---

## 🎯 캡처 절차

### 1단계: Wireshark 실행 및 캡처 시작

```powershell
# 1. Wireshark 실행
# 2. Capture -> USBPcap1 (또는 USBPcap2...) 선택
# 3. Start Capture 클릭
```

**필터 설정** (선택사항):
```
usb.idVendor == 0x191a
```

### 2단계: C# 프로그램으로 명령 실행

각 동작을 **한 번에 하나씩** 실행하고, 사이에 2-3초 대기:

#### LED 제어 테스트
```csharp
// 1. LED 빨강 연속
NE_SetLight(LEDColors.Red, LEDPatterns.Continuous);
Thread.Sleep(3000);

// 2. LED 초록 연속
NE_SetLight(LEDColors.Green, LEDPatterns.Continuous);
Thread.Sleep(3000);

// 3. LED 파랑 연속
NE_SetLight(LEDColors.Blue, LEDPatterns.Continuous);
Thread.Sleep(3000);

// 4. LED 꺼짐
NE_SetLight(LEDColors.Clear, LEDPatterns.OFF);
Thread.Sleep(3000);
```

#### Buzzer 제어 테스트
```csharp
// 5. Buzzer 패턴1 1회
NE_SetBuz(BuzzerPatterns.Pattern1, 50, 1);
Thread.Sleep(5000);

// 6. Buzzer 패턴2 2회
NE_SetBuz(BuzzerPatterns.Pattern2, 50, 2);
Thread.Sleep(5000);

// 7. Buzzer 꺼짐
NE_SetBuz(BuzzerPatterns.OFF, 0, 0);
Thread.Sleep(3000);
```

#### 복합 제어 테스트
```csharp
// 8. LED 빨강 + Buzzer 동시
NE_SetLight(LEDColors.Red, LEDPatterns.Continuous);
NE_SetBuz(BuzzerPatterns.Pattern6, 50, 3);
Thread.Sleep(5000);
```

### 3단계: 캡처 중지 및 저장

```
Stop Capture -> File -> Save As -> patlite_protocol.pcapng
```

---

## 🔍 패킷 분석 방법

### Wireshark 필터 사용

#### USB Control Transfer만 보기
```
usb.transfer_type == 0x02
```

#### Host → Device (명령 전송)만 보기
```
usb.endpoint_address.direction == 0
```

#### Device → Host (응답 수신)만 보기
```
usb.endpoint_address.direction == 1
```

### 분석할 항목

각 패킷에서 다음 정보를 기록:

```
┌──────────────────────────────────────┐
│ Control Transfer                      │
├──────────────────────────────────────┤
│ bmRequestType: 0x??                  │ ← 기록 필요
│ bRequest: 0x??                       │ ← 기록 필요
│ wValue: 0x????                       │ ← 기록 필요
│ wIndex: 0x????                       │ ← 기록 필요
│ wLength: ??                          │ ← 기록 필요
│                                      │
│ Data Payload:                        │
│   [0x??, 0x??, 0x??, ...]           │ ← 가장 중요!
└──────────────────────────────────────┘
```

---

## 📊 분석 결과 정리 템플릿

### LED 제어 프로토콜

| 동작 | bmRequestType | bRequest | wValue | wIndex | Data Payload |
|------|---------------|----------|--------|--------|--------------|
| LED 빨강 ON | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |
| LED 초록 ON | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |
| LED OFF | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |

### Buzzer 제어 프로토콜

| 동작 | bmRequestType | bRequest | wValue | wIndex | Data Payload |
|------|---------------|----------|--------|--------|--------------|
| Buzzer 패턴1 | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |
| Buzzer 패턴2 | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |
| Buzzer OFF | 0x?? | 0x?? | 0x???? | 0x???? | [0x??, 0x??, ...] |

---

## 🛠️ 프로토콜 구현

### 분석 결과를 코드에 반영

`patlite_usb_driver.cpp`의 `encode_led_command()` 함수 수정:

```cpp
std::vector<uint8_t> PatliteUsbDriver::encode_led_command(LEDColor color, LEDPattern pattern)
{
    // Wireshark 분석 결과 반영
    std::vector<uint8_t> data;
    
    data.push_back(0x??);  // ← Wireshark에서 확인한 값
    data.push_back(static_cast<uint8_t>(color));
    data.push_back(static_cast<uint8_t>(pattern));
    data.push_back(0x??);  // ← 추가 바이트 (분석 결과에 따라)
    
    return data;
}
```

### UsbProtocol 구조체 업데이트

`patlite_usb_driver.hpp`의 `UsbProtocol` 수정:

```cpp
UsbProtocol()
    : request_type(0x??),  // ← bmRequestType
      request(0x??),       // ← bRequest
      value(0x????),       // ← wValue
      index(0x????),       // ← wIndex
      timeout_ms(1000)
{ }
```

---

## ✅ 체크리스트

분석 완료 시 확인:

- [ ] LED 색상별 Data Payload 패턴 확인
- [ ] LED 패턴별 Data Payload 차이 확인
- [ ] Buzzer 패턴별 Data Payload 확인
- [ ] 볼륨/반복 횟수 인코딩 방식 확인
- [ ] Control Transfer 파라미터 (bmRequestType, bRequest, wValue, wIndex) 확인
- [ ] 장치 상태 조회(GET) 프로토콜 확인 (있다면)

---

## 🚀 다음 단계

### 1. 프로토콜 구현
```bash
# patlite_usb_driver.cpp 수정
# encode_led_command()
# encode_buzzer_command()
# encode_combined_command()
```

### 2. 원격 PC에서 빌드
```bash
ssh wmszz@172.30.1.102
cd ~/Forklift-Twin-Actuator
colcon build --packages-select fta_actuators
```

### 3. 하드웨어 테스트
```bash
ros2 run fta_actuators led_buzzer_node
```

---

## 📚 참고 자료

### USB HID 표준
- **bmRequestType = 0x21**: Host to Device, Class, Interface
- **bRequest = 0x09**: SET_REPORT (HID)
- **bRequest = 0x01**: GET_REPORT (HID)

### Wireshark 사용법
- USB 캡처: https://wiki.wireshark.org/CaptureSetup/USB
- 필터 문법: https://wiki.wireshark.org/DisplayFilters

### 예상 프로토콜 (추정)
```
LED 제어:
  Data[0] = 명령 코드 (예: 0x57)
  Data[1] = LED 색상 (0-9)
  Data[2] = LED 패턴 (0-6)
  Data[3-4] = 패딩

Buzzer 제어:
  Data[0] = 명령 코드 (예: 0x58)
  Data[1] = Buzzer 패턴 (0-7)
  Data[2] = 볼륨 (0-100)
  Data[3] = 반복 횟수
  Data[4] = 패딩
```

---

## 💡 팁

### 캡처가 안 보일 때
- Patlite 장치를 다시 연결
- USBPcap 재설치
- 다른 USB 포트 사용

### 패킷이 너무 많을 때
- 필터 사용: `usb.idVendor == 0x191a && usb.transfer_type == 0x02`
- 한 번에 하나의 명령만 실행
- 명령 사이에 충분한 대기 시간

### 프로토콜 패턴 찾기
- 같은 명령을 여러 번 실행하여 일관성 확인
- 색상/패턴만 바꿔가며 차이점 비교
- Data Payload의 바이트 위치별 의미 추론
