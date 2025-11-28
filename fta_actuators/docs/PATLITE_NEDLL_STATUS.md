# Patlite NeUsbController.dll 드라이버 구현 상태

## 날짜: 2025-01-28

## 문제 상황

NeUsbController.dll은 **Windows .NET DLL** (PE32 Mono/.Net assembly)입니다.
Linux 환경에서 직접 로드할 수 없습니다.

```bash
$ file NeUsbController.dll
PE32 executable (DLL) (console) Intel 80386 Mono/.Net assembly, for MS Windows
```

## 구현 완료 항목

✅ **PatliteNeDllDriver 클래스 구조**
- 헤더: `include/fta_actuators/patlite_led_buzzer/patlite_led_buzzer_nedll_driver.hpp`
- 구현: `src/patlite/patlite_nedll_driver.cpp`
- DLL 함수 포인터 타입 정의:
  - `NE_OpenDevice()` → int
  - `NE_CloseDevice()` → int
  - `NE_SetLight(int color, int pattern)` → int
  - `NE_SetBuz(int pattern, int volume, int count)` → int
  - `NE_GetDeviceState(bool* buzzer, bool* led, bool* touch)` → int

✅ **PatliteHardwareFactory 통합**
- `DriverType::NE_DLL` 케이스 활성화
- `#include "patlite_nedll_driver.hpp"` 추가

✅ **CMakeLists.txt 업데이트**
- `patlite_nedll_driver.cpp` 빌드 타겟 추가
- `dl` 라이브러리 링크 설정

## 해결 방안

### 방안 1: Wine을 통한 DLL 실행 ⚠️
**장점**: C# 원본 DLL 그대로 사용 가능
**단점**: 
- 현재 시스템에 Wine 미설치
- 성능 오버헤드
- 안정성 문제 (ROS2 런타임에서 Wine 호출)
- 추가 의존성

**구현 난이도**: 중 (Wine 설치 + 브리지 코드)

### 방안 2: USB 직접 제어 (libusb) ✅ 추천
**장점**:
- Linux 네이티브 실행
- 추가 의존성 없음 (libusb는 이미 설치됨)
- 성능 우수
- 안정성 높음

**단점**:
- USB 프로토콜 리버스 엔지니어링 필요
- VID/PID 확인 필요
- 개발 시간 1-2일

**구현 난이도**: 중상 (USB 패킷 분석 필요)

### 방안 3: Mono/.NET Core 브리지
**장점**: .NET DLL 직접 호출 가능
**단점**:
- Mono 또는 .NET Core 런타임 설치 필요
- ROS2와 .NET 간 브리지 복잡
- 디버깅 어려움

**구현 난이도**: 상

## 권장 사항

### 단기: Mock 드라이버로 개발 진행
현재 `PatliteMockDriver`가 완전히 동작합니다.
하드웨어 없이 다음 작업 진행 가능:
- ROS2 토픽 통신 테스트
- PatliteNode 동작 검증
- 통합 테스트

```bash
# Mock 드라이버로 실행
ros2 run fta_actuators patlite_node --ros-args -p driver_type:=mock
```

### 중기: USB 직접 제어 구현
실제 하드웨어 테스트 필요 시:

1. **Patlite 디바이스 VID/PID 확인**
   ```bash
   lsusb
   # 예: Bus 001 Device 005: ID 191a:8003 Patlite Corp.
   ```

2. **USB 패킷 캡처** (Windows에서)
   - USBPcap 또는 Wireshark USB 캡처
   - C# 프로그램 실행하면서 USB 통신 기록
   - 문서: `docs/USB_PACKET_CAPTURE_GUIDE.md`

3. **PatliteUsbDirectDriver 구현**
   - libusb-1.0 사용
   - 캡처한 프로토콜 재현

## 다음 단계

사용자 선택에 따라:

**Option A: Mock으로 계속 진행**
→ 다른 액추에이터(Speaker, Battery 등) 구현

**Option B: USB 직접 제어 시작**
→ 하드웨어 연결 + VID/PID 확인 + 패킷 캡처

**Option C: Wine 설치 시도**
→ Wine 설치 + P/Invoke 브리지 구현

## 참고 파일

- C# 원본: `reference/LIS/WATA.LIS/WATA.LIS.Core/Services/ServiceImpl/StatusService_WATA.cs`
- DLL 위치: `fta_actuators/lib/NeUsbController.dll`
- USB 드라이버 설계: `docs/USB_DRIVER_DESIGN.md`
- 패킷 캡처 가이드: `docs/USB_PACKET_CAPTURE_GUIDE.md`
