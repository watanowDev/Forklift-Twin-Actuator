# 🚀 초보자를 위한 ROS2 + Patlite LED 제어 가이드

이 문서는 C++와 ROS2가 처음인 개발자를 위한 단계별 가이드입니다.

## 📋 목차
1. [환경 이해하기](#1-환경-이해하기)
2. [SSH 원격 접속 설정](#2-ssh-원격-접속-설정)
3. [Patlite USB 디바이스 확인](#3-patlite-usb-디바이스-확인)
4. [간단한 ROS2 노드 만들기](#4-간단한-ros2-노드-만들기)
5. [원격 빌드하기](#5-원격-빌드하기)
6. [ROS2 통신 테스트](#6-ros2-통신-테스트)
7. [Patlite 제어 함수 추가](#7-patlite-제어-함수-추가)

---

## 1. 환경 이해하기

### 🖥️ 개발 환경 구성
```
[Windows PC] ──SSH──> [Ubuntu 172.30.1.102]
   (VS Code)              (ROS2 Jazzy)
                          (Patlite USB 장치)
```

### 📦 프로젝트 구조 (간단 버전)
```
Forklift-Twin-Actuator/
├── fta_interfaces/           # 메시지 정의 (ActionEvent, ActuatorStatus)
│   ├── msg/
│   │   ├── ActionEvent.msg
│   │   └── ActuatorStatus.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
└── fta_actuators/            # 실제 제어 코드
    ├── config/
    │   └── led_buzzer.yaml   # 설정 파일
    ├── include/fta_actuators/
    │   └── led_buzzer_node.hpp  # 헤더 파일 (선언)
    ├── src/
    │   ├── led_buzzer_node.cpp      # 구현 파일
    │   └── led_buzzer_node_main.cpp # 실행 파일
    ├── launch/
    │   └── led_buzzer.launch.py     # 실행 스크립트
    ├── CMakeLists.txt        # 빌드 설정
    └── package.xml           # 패키지 정보
```

---

## 2. SSH 원격 접속 설정

### 2.1 기본 SSH 접속 테스트
Windows PowerShell에서:
```powershell
ssh wmszz@172.30.1.102
```

### 2.2 비밀번호 없이 접속하기 (선택사항)
매번 비밀번호 입력이 귀찮다면 SSH Key를 설정하세요:

```powershell
# 1. SSH 키 생성 (Windows)
ssh-keygen -t rsa -b 4096

# 2. 공개키를 원격 서버에 복사
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh wmszz@172.30.1.102 "cat >> ~/.ssh/authorized_keys"
```

---

## 3. Patlite USB 디바이스 확인

### 3.1 USB 디바이스 확인
```bash
# SSH로 접속 후
ssh wmszz@172.30.1.102

# USB 장치 목록 확인
lsusb

# 예상 출력:
# Bus 001 Device 005: ID 191a:XXXX Patlite Corporation
```

### 3.2 USB 권한 설정
Patlite를 일반 사용자가 접근하려면 권한 설정이 필요합니다:

```bash
# 1. USB 장치의 Vendor ID와 Product ID 확인
lsusb | grep -i patlite

# 2. udev 규칙 생성 (예: Vendor ID가 191a인 경우)
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="191a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-patlite.rules

# 3. udev 재시작
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. USB 재연결 또는 재부팅
```

---

## 4. 간단한 ROS2 노드 만들기

### 4.1 ROS2 노드란?
- **노드(Node)**: ROS2에서 실행되는 하나의 프로그램
- **토픽(Topic)**: 노드들이 메시지를 주고받는 통로
- **메시지(Message)**: 토픽을 통해 전달되는 데이터

### 4.2 개념 이해 - 예시
```
[Publisher 노드]  ──메시지──>  [토픽: /actions/event]  ──메시지──>  [Subscriber 노드]
  (명령 전송)                                                        (명령 수신 & 실행)
```

### 4.3 우리가 만들 노드
- **이름**: `led_buzzer_node`
- **역할**: `/actions/event` 토픽을 구독하여 Patlite LED/Buzzer 제어
- **언어**: C++17

---

## 5. 원격 빌드하기

### 5.1 프로젝트 파일 전송
Windows에서 원격 Ubuntu로 파일 전송:

```powershell
# SCP로 전체 프로젝트 복사
scp -r C:\Users\wmszz\source\repos\Forklift-Twin-Actuator wmszz@172.30.1.102:~/

# 또는 rsync 사용 (더 빠름)
# WSL에서: rsync -avz --exclude 'reference/' ./ wmszz@172.30.1.102:~/Forklift-Twin-Actuator/
```

### 5.2 원격에서 빌드
```bash
# SSH 접속
ssh wmszz@172.30.1.102

# 프로젝트 디렉토리로 이동
cd ~/Forklift-Twin-Actuator

# ROS2 환경 설정 (매번 터미널 열 때마다 필요)
source /opt/ros/jazzy/setup.bash

# 빌드 (처음에는 인터페이스부터)
colcon build --packages-select fta_interfaces

# 빌드된 환경 적용
source install/setup.bash

# 액추에이터 패키지 빌드
colcon build --packages-select fta_actuators

# 다시 환경 적용
source install/setup.bash
```

### 5.3 빌드 과정 이해
```
소스 코드 (.cpp, .hpp)
    ↓ [CMake 설정 읽기]
    ↓ [컴파일러 실행 (g++)]
    ↓
실행 파일 (led_buzzer_node)
    ↓
install/ 폴더에 설치
```

---

## 6. ROS2 통신 테스트

### 6.1 노드 실행
```bash
# 터미널 1: LED Buzzer 노드 실행
ros2 run fta_actuators led_buzzer_node
```

### 6.2 다른 터미널에서 명령 전송
```bash
# 터미널 2: 토픽으로 메시지 발행 (JSON 형식)
ros2 topic pub --once /actions/event std_msgs/msg/String \
  "{data: '{\"device\": \"led_buzzer\", \"action\": \"led_on\", \"params\": {\"color\": \"red\"}}'}"
```

### 6.3 상태 확인
```bash
# 터미널 3: 상태 토픽 구독
ros2 topic echo /actuators/status
```

### 6.4 ROS2 명령어 정리
```bash
# 실행 중인 노드 목록
ros2 node list

# 토픽 목록 보기
ros2 topic list

# 토픽에 흐르는 메시지 보기
ros2 topic echo /actions/event

# 토픽 정보 보기
ros2 topic info /actions/event

# 노드 정보 보기
ros2 node info /led_buzzer_node
```

---

## 7. Patlite 제어 함수 추가

### 7.1 C# 코드에서 함수 찾기
`reference/WATA.LIS.INDICATOR.LED/StatusLED/Patlite_LED_Buzzer.cs` 파일에서:
- LED 제어 함수
- Buzzer 제어 함수
- 상태별 제어 로직

이것들을 C++로 포팅합니다.

### 7.2 함수 포팅 예시
C# 코드:
```csharp
public void SetRedLED()
{
    // NeUsbController 사용
    controller.SetLED(Color.Red);
}
```

C++로 변환:
```cpp
void PatliteLEDBuzzerDriver::setRedLED()
{
    // libusb 또는 NeUsbController C++ 포트 사용
    // TODO: 실제 USB 통신 구현
}
```

---

## 🎯 다음 단계

1. ✅ SSH 접속 확인
2. ✅ Patlite USB 인식 확인
3. ⏳ 간단한 테스트 노드 실행
4. ⏳ C# 코드 분석 및 함수 포팅
5. ⏳ 실제 하드웨어 제어 테스트

---

## 💡 자주 묻는 질문

### Q: 빌드가 실패하면?
```bash
# 로그 자세히 보기
colcon build --packages-select fta_actuators --event-handlers console_direct+

# 특정 패키지만 재빌드
colcon build --packages-select fta_actuators --cmake-clean-cache
```

### Q: ROS2 환경이 안 잡히면?
```bash
# 항상 실행 필요
source /opt/ros/jazzy/setup.bash
source ~/Forklift-Twin-Actuator/install/setup.bash

# 자동화하려면 ~/.bashrc에 추가
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/Forklift-Twin-Actuator/install/setup.bash" >> ~/.bashrc
```

### Q: C++와 Python 중 뭘 써야 하나?
- **C++**: 성능이 중요한 제어 시스템 (우리 프로젝트)
- **Python**: 빠른 프로토타이핑, 데이터 분석

---

## 📚 추가 학습 자료

- [ROS2 공식 튜토리얼 (한국어)](https://docs.ros.org/en/jazzy/Tutorials.html)
- [C++ 기초 문법](https://learn.microsoft.com/ko-kr/cpp/)
- [CMake 튜토리얼](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
