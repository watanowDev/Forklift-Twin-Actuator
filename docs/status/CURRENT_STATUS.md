# 📋 작업 체크리스트 - ROS2 + Patlite LED 제어

현재 작업 상태를 확인하고 다음 단계를 안내합니다.

---

## ✅ 완료된 작업

### 1. 프로젝트 초기 설정
- [x] `.github/copilot-instructions.md` 작성
- [x] 프로젝트 구조 문서화 (docs/)
- [x] 기본 ROS2 패키지 구조 생성 (fta_interfaces, fta_actuators)

### 2. 초보자용 학습 자료
- [x] `BEGINNER_GUIDE.md` 작성 (상세한 단계별 가이드)
- [x] `fta_beginner` 패키지 생성
- [x] 간단한 Publisher/Subscriber 노드 (`simple_publisher.cpp`, `simple_subscriber.cpp`)
- [x] Patlite 테스트 노드 (`patlite_test.cpp`)
- [x] 빠른 시작 가이드 (`fta_beginner/README.md`)

### 3. 개발 환경
- [x] SSH 원격 접속 확인 (172.30.1.102)
- [x] Ubuntu 24.04 + ROS2 Jazzy 확인됨

---

## 🔄 진행 중인 작업

### 현재 단계: 원격 빌드 및 테스트 준비

**다음에 해야 할 일:**
1. 파일을 Ubuntu 원격 PC로 전송
2. 빌드 테스트
3. ROS2 통신 테스트
4. Patlite USB 장치 확인

---

## ⏳ 대기 중인 작업

### 단기 (1-2일)
- [ ] **Patlite USB 장치 인식 확인**
  - `lsusb` 명령어로 Vendor ID/Product ID 확인
  - USB 권한 설정 (udev rules)

- [ ] **ROS2 통신 테스트**
  - `simple_publisher`와 `simple_subscriber` 실행
  - `patlite_test` 노드로 명령 수신 확인

- [ ] **C# 코드 분석**
  - `reference/WATA.LIS.INDICATOR.LED/StatusLED/Patlite_LED_Buzzer.cs` 분석
  - USB 통신 방법 파악 (NeUsbController 사용법)
  - 각 함수별 기능 정리

### 중기 (1주)
- [ ] **USB 라이브러리 선택 및 구현**
  - 옵션 1: `libusb` 사용 (권장)
  - 옵션 2: `hidapi` 사용
  - 옵션 3: NeUsbController C++ 포팅

- [ ] **실제 Patlite 제어 구현**
  - LED 제어 함수 (Red, Green, Yellow)
  - Buzzer 제어 함수 (On/Off, Pattern)
  - 에러 처리 (USB 통신 실패, 장치 미연결 등)

- [ ] **상황별 제어 함수 포팅**
  - C# 코드에서 각 상황별 제어 로직 추출
  - C++로 변환하여 함수화
  - ROS2 토픽으로 호출 가능하게 구현

### 장기 (2-4주)
- [ ] **FTE 모듈과 통합**
  - `/actions/event` 토픽 구독
  - `/actuators/status` 토픽 발행
  - JSON 메시지 파싱

- [ ] **FTL(Logger) 연동**
  - 제어 이력 로깅
  - 에러 로그 전송

- [ ] **Speaker 모듈 구현**
  - ALSA 기반 음성 출력
  - 다국어 지원 (한국어, 일본어, 영어)

- [ ] **배터리 및 전원 관리** (11.23 이후)
  - 배터리 상태 모니터링
  - 저전력 모드 구현

---

## 🎯 이번 세션 목표

### 목표: ROS2 기본 통신 확인 및 Patlite USB 인식 확인

**Step 1: 파일 전송**
```powershell
# Windows PowerShell
cd C:\Users\wmszz\source\repos\Forklift-Twin-Actuator
scp -r fta_beginner fta_interfaces wmszz@172.30.1.102:~/Forklift-Twin-Actuator/
```

**Step 2: 빌드**
```bash
# Ubuntu SSH
ssh wmszz@172.30.1.102
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fta_interfaces fta_beginner
source install/setup.bash
```

**Step 3: 통신 테스트**
```bash
# 터미널 1
ros2 run fta_beginner simple_publisher

# 터미널 2
ros2 run fta_beginner simple_subscriber
```

**Step 4: Patlite 테스트**
```bash
# Patlite USB 연결 후
ros2 run fta_beginner patlite_test

# 다른 터미널에서 명령 전송
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'LED_RED_ON'}"
```

---

## 📚 참고 문서

### 학습 자료
- [BEGINNER_GUIDE.md](./BEGINNER_GUIDE.md) - ROS2 기초 개념 및 단계별 가이드
- [fta_beginner/README.md](./fta_beginner/README.md) - 빠른 실행 방법

### 기술 문서
- [docs/ARCHITECTURE.md](./docs/ARCHITECTURE.md) - 시스템 아키텍처
- [docs/PACKAGE_STRUCTURE.md](./docs/PACKAGE_STRUCTURE.md) - 패키지 구조
- [docs/DEVELOPMENT_GUIDE.md](./docs/DEVELOPMENT_GUIDE.md) - 개발 가이드

### 참고 코드
- `reference/WATA.LIS.INDICATOR.LED/` - C# 구현체 (원본)
- `fta_beginner/src/` - 초보자용 간단한 예제
- `fta_actuators/src/` - 실제 구현 (진행 중)

---

## 🐛 알려진 문제

### 1. NeUsbController 의존성
- **문제**: C# 코드가 NeUsbController를 사용하는데, C++ 버전이 없음
- **해결 방안**: 
  - 임시: `libusb` 또는 `hidapi`로 직접 USB 통신 구현
  - 장기: NeUsbController Java/C# 코드를 C++로 포팅

### 2. Ubuntu 버전 불일치
- **문제**: 문서는 Ubuntu 22.04 기준인데, 실제는 24.04
- **영향**: 거의 없음 (ROS2 Jazzy는 24.04에서 더 잘 작동)
- **조치**: 문서 업데이트 필요

---

## 💬 질문이 있으신가요?

### Q1: C++을 몰라도 괜찮나요?
**A**: 네! `fta_beginner` 패키지의 코드는 상세한 한글 주석이 달려 있어 학습하면서 진행할 수 있습니다.

### Q2: ROS2를 처음 사용하는데 어디서부터 시작해야 하나요?
**A**: [BEGINNER_GUIDE.md](./BEGINNER_GUIDE.md)의 순서대로 따라하시면 됩니다.

### Q3: 빌드가 실패하면 어떻게 하나요?
**A**: `fta_beginner/README.md`의 "트러블슈팅" 섹션을 참고하세요.

---

## 📅 다음 세션 계획

1. **현재 세션**: ROS2 통신 테스트 및 USB 장치 확인
2. **다음 세션**: C# 코드 분석 및 USB 라이브러리 선택
3. **그 다음 세션**: 실제 Patlite 제어 구현

---

**마지막 업데이트**: 2025-11-25
