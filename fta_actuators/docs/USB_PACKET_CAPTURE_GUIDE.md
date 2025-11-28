# USB 패킷 캡처 가이드

## 목적
Patlite USB Signal Beacon의 정확한 USB 통신 프로토콜을 파악하기 위해 Windows에서 실제 C# 프로그램이 보내는 USB 패킷을 캡처합니다.

## 준비물
- Windows PC
- Patlite USB Signal Beacon 연결
- C# 원본 프로그램 (WATA.LIS)
- Wireshark (USB 캡처 지원 버전)
- USBPcap 플러그인

## 방법 1: Wireshark + USBPcap

### 1단계: USBPcap 설치
```
1. Wireshark 다운로드: https://www.wireshark.org/download.html
2. 설치 시 "USBPcap" 체크박스 선택
3. 설치 완료 후 재부팅
```

### 2단계: Patlite 디바이스 확인
```
1. 장치 관리자 열기
2. "범용 직렬 버스 컨트롤러" 확장
3. "PATLITE Corporation USB Signal Beacon" 확인
```

### 3단계: Wireshark 캡처 시작
```
1. Wireshark 실행 (관리자 권한)
2. "Capture" → "USBPcap" 인터페이스 선택
3. 필터 설정: usb.device_address == <Patlite 주소>
4. "Start" 클릭
```

### 4단계: C# 프로그램 실행
```
1. WATA.LIS 프로그램 실행
2. LED/Buzzer 제어 기능 순차 실행:
   - 빨간 LED 켜기
   - 초록 LED 켜기
   - 노란(Amber) LED 켜기
   - LED 깜박임 패턴
   - Buzzer 소리 1
   - Buzzer 소리 2
   - 전체 OFF
```

### 5단계: 캡처 분석
```
1. Wireshark에서 "Stop" 클릭
2. 필터: usb.transfer_type == 0x01 (Interrupt Transfer)
3. Endpoint 0x01 (OUT) 패킷 확인
4. "HID Data" 필드에서 실제 전송된 바이트 확인
```

## 방법 2: USBlyzer (유료, 더 편리)

### 1단계: USBlyzer 설치
```
웹사이트: http://www.usblyzer.com/
가격: $99.95 (30일 체험판 사용 가능)
```

### 2단계: 캡처
```
1. USBlyzer 실행
2. Patlite 디바이스 선택
3. "Capture" 시작
4. C# 프로그램으로 LED/Buzzer 제어
5. "Stop" 후 패킷 분석
```

## 예상 패킷 형식

현재 구현된 형식 (추정):
```
LED 제어:
[0] = 0x00  (Report ID)
[1] = 0x01  (Command: LED)
[2] = color (LEDColor enum)
[3] = pattern (LEDPattern enum)
[4-7] = 0x00 (Reserved)

Buzzer 제어:
[0] = 0x00  (Report ID)
[1] = 0x02  (Command: Buzzer)
[2] = pattern (BuzzerPattern enum)
[3] = volume (0-100)
[4] = count (반복 횟수)
[5-7] = 0x00 (Reserved)
```

## 캡처 후 작업

1. **패킷 분석**
   - Wireshark에서 캡처한 바이트 시퀀스 기록
   - 각 명령(LED ON, LED Pattern, Buzzer 등)별로 정리

2. **프로토콜 역설계**
   - 공통 헤더 구조 파악
   - 명령어 코드 확인
   - 파라미터 인코딩 방식 분석

3. **코드 업데이트**
   - `patlite_usb_direct_driver.cpp`의 `build_led_command()` 수정
   - `build_buzzer_command()` 수정
   - 실제 프로토콜에 맞게 바이트 배열 구성

4. **검증**
   - Linux에서 수정된 드라이버로 재테스트
   - 실제 LED/Buzzer 동작 확인

## 참고: 현재 테스트 결과

Linux에서 USB 통신은 성공:
```bash
[PatliteUsb] Device opened successfully
[PatliteUsb] Sent 8 bytes: 00 01 01 01 00 00 00 00  # RED LED
[PatliteUsb] Sent 8 bytes: 00 01 03 01 00 00 00 00  # AMBER LED
[PatliteUsb] Sent 8 bytes: 00 02 02 32 01 00 00 00  # Buzzer
```

하지만 실제 하드웨어 동작 여부는 미확인.
→ USB 패킷 캡처로 정확한 프로토콜 확인 필요

## 문의사항
패킷 캡처가 어려운 경우:
1. Patlite 공식 문서 확인
2. NeUsbController.dll 디컴파일 (ILSpy 사용)
3. 하드웨어 매뉴얼 참조

## 다음 단계
[ ] Windows에서 USB 패킷 캡처
[ ] 프로토콜 분석 및 문서화
[ ] Linux 드라이버 코드 업데이트
[ ] 실제 하드웨어 동작 검증
