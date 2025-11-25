# 🔌 USB 드라이버 개발 가이드

Patlite USB LED Buzzer를 제어하기 위한 USB 드라이버 개발 문서입니다.

## 📚 문서 구조

1. **[USB_DRIVER_DESIGN.md](../docs/USB_DRIVER_DESIGN.md)** - 전체 설계 및 옵션
2. **[USB_QUICK_START.md](./USB_QUICK_START.md)** - 5분 빠른 시작
3. **이 파일** - 개발 워크플로우

---

## 🎯 개발 워크플로우

### Phase 1: USB 장치 정보 수집 ✅ (완료)

**파일**: `usb_device_info.cpp`

**컴파일**:
```bash
cd ~/Forklift-Twin-Actuator/fta_beginner/src
g++ -o usb_device_info usb_device_info.cpp -lusb-1.0
```

**실행**:
```bash
# 모든 정보 출력
sudo ./usb_device_info

# Patlite 장치만
sudo ./usb_device_info patlite

# 전체 USB 장치 스캔
sudo ./usb_device_info scan
```

**수집할 정보**:
- [ ] Vendor ID (예: 0x191a)
- [ ] Product ID (예: 0x8003)
- [ ] Interface Class (HID / Vendor Specific)
- [ ] Endpoint Address (IN/OUT)
- [ ] Transfer Type (Interrupt / Bulk / Control)

---

### Phase 2: 프로토콜 분석 ⏳ (예정)

**Windows에서 USB 패킷 캡처**:
1. USBPcap + Wireshark 설치
2. C# 프로그램 실행하며 패킷 캡처
3. LED 제어 명령 분석
4. Buzzer 제어 명령 분석

**분석할 내용**:
- Control Transfer 구조
- Data 페이로드 포맷
- 명령 바이트 매핑

---

### Phase 3: 드라이버 구현 ⏳ (예정)

**파일 생성 예정**:
```
fta_actuators/
├── include/fta_actuators/
│   └── patlite_usb_driver.hpp
└── src/
    └── patlite_usb_driver.cpp
```

**핵심 클래스**:
```cpp
class PatliteUsbDriver
{
public:
  bool open_device(uint16_t vendor_id, uint16_t product_id);
  void close_device();
  bool set_light(LEDColor color, LEDPattern pattern);
  bool set_buzzer(BuzzerPattern pattern, int volume, int count);
  bool get_device_state(bool& buzzer, bool& led, bool& touch);
};
```

---

### Phase 4: 시나리오와 통합 ⏳ (예정)

**`patlite_scenarios.cpp` 수정**:
```cpp
void PatliteController::execute_command(const PatliteCommand& cmd)
{
  // TODO: USB 드라이버 호출로 변경
  usb_driver_.set_light(cmd.led_color, cmd.led_pattern);
  usb_driver_.set_buzzer(cmd.buzzer_pattern, 50, cmd.buzzer_count);
}
```

---

### Phase 5: ROS2 노드 구현 ⏳ (예정)

**토픽 구독**:
- `/patlite/scenario` - 시나리오 이름으로 제어
- `/patlite/context` - 컨텍스트 기반 제어

**토픽 발행**:
- `/actuators/status` - 제어 결과 및 상태

---

## 🛠️ 도구

### USB 정보 수집 프로그램
```bash
# 컴파일
cd fta_beginner/src
g++ -o usb_device_info usb_device_info.cpp -lusb-1.0

# 실행
sudo ./usb_device_info
```

### udev 규칙 설정
```bash
# /etc/udev/rules.d/99-patlite.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="191a", MODE="0666", GROUP="plugdev"

# 적용
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 📊 현재 상태

| Phase | 상태 | 파일 |
|-------|------|------|
| 1. USB 정보 수집 | ✅ 완료 | `usb_device_info.cpp` |
| 2. 프로토콜 분석 | ⏳ 대기 | - |
| 3. 드라이버 구현 | ⏳ 대기 | `patlite_usb_driver.cpp` |
| 4. 시나리오 통합 | ⏳ 대기 | `patlite_scenarios.cpp` 수정 |
| 5. ROS2 노드 | ⏳ 대기 | `patlite_node.cpp` |

---

## 🚀 다음 작업

### 즉시 가능 (원격 PC 사용 가능 시)
1. **USB 정보 수집**
   ```bash
   ssh wmszz@172.30.1.102
   cd ~/Forklift-Twin-Actuator/fta_beginner/src
   g++ -o usb_device_info usb_device_info.cpp -lusb-1.0
   sudo ./usb_device_info patlite > patlite_info.txt
   cat patlite_info.txt
   ```

2. **Product ID 확인**
   - 출력에서 `Product ID: 0x????` 값 기록
   - 이 값을 드라이버 코드에 사용

### 필요 시 (프로토콜 분석)
3. **Windows USB 패킷 캡처**
   - USBPcap 설치
   - C# 프로그램 실행
   - LED/Buzzer 제어 시 패킷 캡처
   - 명령 구조 분석

---

## 📚 참고

- **libusb**: https://libusb.info/
- **hidapi**: https://github.com/libusb/hidapi
- **USB HID**: https://www.usb.org/hid
- **Wireshark USB**: https://wiki.wireshark.org/CaptureSetup/USB
