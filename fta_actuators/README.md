# FTA Actuators Package

## 개요

FTA (Forklift Twin Actuators)는 지게차용 액추에이터 제어를 담당하는 ROS2 패키지입니다.
여러 종류의 액추에이터를 모듈화하여 통합 관리합니다.

## 지원 액추에이터

### 현재 구현됨
- **PatliteLedBuzzer LED/Buzzer**: 시각/청각 알람 제어 (C# LIS 프로젝트 포팅)

### 예정
- **Speaker**: 음성 출력 및 TTS (11.23 이후)
- **Battery Controller**: 전력계 MCU 연동 (11.23 이후)
- **Power Manager**: 저전력 모드 전환 (11.23 이후)

## 디렉토리 구조

```
fta_actuators/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── config/                          # 설정 파일
│   └── actuators_config.yaml
│
├── launch/                          # 런치 파일
│   └── patlite.launch.py
│
├── docs/                            # 문서
│   ├── DIRECTORY_STRUCTURE.md       # 디렉토리 구조 설명
│   ├── PATLITE.md                   # PatliteLedBuzzer 모듈 상세 문서
│   └── PATLITE_SCENARIOS.md         # PatliteLedBuzzer 시나리오 가이드
│
├── include/fta_actuators/
│   └── patlite/                     # PatliteLedBuzzer 모듈
│       ├── patlite_types.hpp        # 타입 정의 (enum)
│       ├── patlite_controller.hpp   # 제어 로직 (C# 포팅)
│       ├── patlite_driver.hpp       # 드라이버 인터페이스
│       ├── patlite_usb_driver.hpp   # USB 통신 구현
│       ├── patlite_scenarios.hpp    # 시나리오 매핑
│       └── patlite_node.hpp         # ROS2 노드
│
└── src/
    ├── patlite/                     # PatliteLedBuzzer 구현
    │   ├── patlite_controller.cpp
    │   ├── patlite_driver.cpp
    │   ├── patlite_usb_driver.cpp
    │   ├── patlite_scenarios.cpp
    │   ├── patlite_node.cpp
    │   └── patlite_node_main.cpp
    │
    └── tests/                       # 테스트 파일
        ├── test_patlite_action_publisher.cpp
        └── test_patlite_buzzer_sound.cpp
```

## 빌드 및 실행

### 빌드
```bash
cd ~/ros2_ws/src/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fta_interfaces fta_actuators
```

### 실행
```bash
source install/setup.bash

# PatliteLedBuzzer 노드 실행
ros2 run fta_actuators patlite_node

# 또는 런치 파일 사용
ros2 launch fta_actuators patlite.launch.py
```

### 테스트
```bash
# 액션 테스트 퍼블리셔
ros2 run fta_actuators test_patlite_action_publisher

# 부저 사운드 테스트
ros2 run fta_actuators test_patlite_buzzer_sound
```

## ROS2 통신

### 구독 토픽
- **`/actions/event`** (QoS1): FTE로부터 액추에이터 제어 명령 수신
  - 메시지 타입: `fta_interfaces/msg/ActionEvent`

### 발행 토픽
- **`/actuators/status`** (QoS1): 액추에이터 상태 및 제어 결과 발행
  - 메시지 타입: `fta_interfaces/msg/ActuatorStatus`

## 액추에이터 추가 가이드

새로운 액추에이터를 추가할 때는 다음 구조를 따르세요:

```
1. include/fta_actuators/{module}/
   - {module}_types.hpp      : 타입 정의
   - {module}_controller.hpp : 제어 로직
   - {module}_driver.hpp     : 하드웨어 드라이버
   - {module}_node.hpp       : ROS2 노드

2. src/{module}/
   - {module}_controller.cpp
   - {module}_driver.cpp
   - {module}_node.cpp
   - {module}_node_main.cpp

3. docs/{MODULE}.md          : 모듈 문서

4. CMakeLists.txt 업데이트
   - 라이브러리 추가
   - 노드 추가
   - 설치 규칙 추가
```

자세한 내용은 `docs/DIRECTORY_STRUCTURE.md`를 참조하세요.

## 참고 문서

- [디렉토리 구조 상세](docs/DIRECTORY_STRUCTURE.md)
- [PatliteLedBuzzer 모듈 상세](docs/PATLITE.md)
- [PatliteLedBuzzer 시나리오](docs/PATLITE_SCENARIOS.md)
- [FTA 개발 가이드](../.github/copilot-instructions.md)

## 라이선스

이 프로젝트는 WATA 사의 소유이며, 내부 사용 전용입니다.
