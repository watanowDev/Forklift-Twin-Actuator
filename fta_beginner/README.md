# 🚀 빠른 시작 가이드 - Patlite LED 제어

이 문서는 **가장 빠르게** ROS2 + Patlite를 테스트하는 방법을 설명합니다.

---

## 📦 1단계: 파일 전송 (Windows → Ubuntu)

### 방법 1: SCP 사용 (PowerShell)
```powershell
# 현재 위치 확인
cd C:\Users\wmszz\source\repos\Forklift-Twin-Actuator

# 전체 프로젝트 전송 (reference 폴더 제외)
scp -r fta_beginner fta_interfaces wmszz@172.30.1.102:~/Forklift-Twin-Actuator/
```

### 방법 2: Git 사용 (권장)
```powershell
# Windows에서
git add .
git commit -m "초보자용 테스트 노드 추가"
git push

# Ubuntu에서
ssh wmszz@172.30.1.102
cd ~/Forklift-Twin-Actuator
git pull
```

---

## 🔨 2단계: 빌드 (Ubuntu SSH 접속 후)

```bash
# SSH 접속
ssh wmszz@172.30.1.102

# 프로젝트 디렉토리로 이동
cd ~/Forklift-Twin-Actuator

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash

# 빌드 (인터페이스 먼저)
colcon build --packages-select fta_interfaces

# 환경 적용
source install/setup.bash

# 초보자 패키지 빌드
colcon build --packages-select fta_beginner

# 다시 환경 적용
source install/setup.bash
```

### 빌드 에러가 나면?
```bash
# 캐시 삭제 후 재빌드
rm -rf build/ install/ log/
colcon build --packages-select fta_beginner
```

---

## 🧪 3단계: 간단한 통신 테스트

### 테스트 1: Publisher와 Subscriber

**터미널 1: Publisher 실행**
```bash
source install/setup.bash
ros2 run fta_beginner simple_publisher
```
출력 예시:
```
[INFO] [1700000000.123456789] [simple_publisher]: SimplePublisher 노드가 시작되었습니다!
[INFO] [1700000000.123456789] [simple_publisher]: 1초마다 메시지를 발행합니다: /test_topic
[INFO] [1700000001.123456789] [simple_publisher]: 발행 중: '안녕하세요! 메시지 번호: 0'
[INFO] [1700000002.123456789] [simple_publisher]: 발행 중: '안녕하세요! 메시지 번호: 1'
```

**터미널 2: Subscriber 실행** (새 터미널)
```bash
ssh wmszz@172.30.1.102
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run fta_beginner simple_subscriber
```
출력 예시:
```
[INFO] [1700000000.123456789] [simple_subscriber]: SimpleSubscriber 노드가 시작되었습니다!
[INFO] [1700000000.123456789] [simple_subscriber]: 토픽 구독 중: /test_topic
[INFO] [1700000001.123456789] [simple_subscriber]: 받은 메시지: '안녕하세요! 메시지 번호: 0'
[INFO] [1700000002.123456789] [simple_subscriber]: 받은 메시지: '안녕하세요! 메시지 번호: 1'
```

✅ **성공!** Publisher가 보낸 메시지를 Subscriber가 받고 있습니다!

---

## 🔌 4단계: Patlite USB 장치 확인

### Patlite 장치 연결
1. Patlite USB LED Buzzer를 Ubuntu PC에 연결
2. 다음 명령어로 인식 확인:

```bash
lsusb
```

예상 출력:
```
Bus 001 Device 005: ID 191a:XXXX Patlite Corporation
```

`191a`가 Patlite의 Vendor ID입니다.

### USB 권한 설정 (필요시)
```bash
# Vendor ID 확인 후
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="191a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-patlite.rules

# udev 재시작
sudo udevadm control --reload-rules
sudo udevadm trigger

# USB 재연결 또는 재부팅
```

---

## 🎮 5단계: Patlite 테스트 노드 실행

**터미널 1: Patlite 테스트 노드 실행**
```bash
source install/setup.bash
ros2 run fta_beginner patlite_test
```

노드가 시작되면 USB 장치 목록이 출력됩니다.

**터미널 2: 명령 전송**
```bash
# 빨간색 LED 켜기
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'LED_RED_ON'}"

# 초록색 LED 켜기
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'LED_GREEN_ON'}"

# 부저 켜기
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'BUZZER_ON'}"

# 부저 끄기
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'BUZZER_OFF'}"

# 모든 LED 끄기
ros2 topic pub --once /patlite/command std_msgs/msg/String "{data: 'LED_OFF'}"
```

### 예상 출력 (터미널 1)
```
========================================
명령 수신: 'LED_RED_ON'
→ 빨간색 LED를 켭니다 (시뮬레이션)
========================================
```

---

## 🔍 6단계: ROS2 디버깅 명령어

```bash
# 실행 중인 노드 확인
ros2 node list

# 토픽 목록
ros2 topic list

# 토픽 메시지 확인
ros2 topic echo /patlite/command

# 토픽 정보
ros2 topic info /patlite/command

# 노드 정보
ros2 node info /patlite_test_node
```

---

## 📝 다음 단계

현재 `patlite_test` 노드는 **시뮬레이션만** 합니다. 실제 Patlite 하드웨어를 제어하려면:

1. **C# 코드 분석**: `reference/WATA.LIS.INDICATOR.LED/StatusLED/Patlite_LED_Buzzer.cs` 파일을 보고 USB 통신 방법 파악
2. **USB 라이브러리 선택**: 
   - `libusb` (C/C++ 표준 라이브러리)
   - `hidapi` (HID 장치용)
   - NeUsbController C++ 포팅
3. **실제 제어 코드 구현**: `patlite_test.cpp`의 `command_callback` 함수 내부에 USB 통신 코드 추가

---

## ❓ 트러블슈팅

### 빌드 에러: "package 'fta_interfaces' not found"
```bash
# 먼저 인터페이스 빌드
colcon build --packages-select fta_interfaces
source install/setup.bash
```

### 빌드 에러: "std_msgs not found"
```bash
# std_msgs 설치
sudo apt install ros-jazzy-std-msgs
```

### SSH 비밀번호 매번 입력이 귀찮다면
```powershell
# Windows에서 SSH Key 생성
ssh-keygen -t rsa

# 공개키 전송
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh wmszz@172.30.1.102 "cat >> ~/.ssh/authorized_keys"
```

### 노드 실행 시 "command not found"
```bash
# ROS2 환경이 제대로 설정되지 않음
source /opt/ros/jazzy/setup.bash
source ~/Forklift-Twin-Actuator/install/setup.bash
```

---

## 📚 추가 학습

- [ROS2 공식 튜토리얼](https://docs.ros.org/en/jazzy/Tutorials.html)
- [C++ 기초](https://learn.microsoft.com/ko-kr/cpp/)
- [libusb 문서](https://libusb.info/)
- [BEGINNER_GUIDE.md](./BEGINNER_GUIDE.md) - 더 자세한 설명
