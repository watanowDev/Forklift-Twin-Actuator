# FTA - Forklift Twin Actuators

## 개요
FTA(Forklift Twin Actuators)는 지게차 액추에이터를 제어하는 ROS2 기반 모듈입니다.

## 🚀 빠른 시작
- **처음 사용하시나요?** → [초보자 가이드 (BEGINNER_GUIDE.md)](./BEGINNER_GUIDE.md)
- **바로 테스트하고 싶으신가요?** → [빠른 시작 가이드 (fta_beginner/README.md)](./fta_beginner/README.md)

## 주요 기능
- LED Buzzer 제어 (시각/청각 알람)
- Speaker 제어 (음성 출력 및 사운드 재생)
- 배터리 제어 (전력계 MCU 연동, 11.23 이후)
- 저전력 모드 (절전 모드 전환, 11.23 이후)

## 시스템 아키텍처
FTA는 Forklift Twin 시스템의 5개 독립 프로젝트 중 하나로, 느슨한 결합(Loose Coupling) 원칙을 따릅니다.

### 통신 구조
- **구독**: `/actions/event` (QoS1) - FTE로부터 제어 명령 수신
- **발행**: `/actuators/status` (QoS1) - 액추에이터 상태 전송

## 개발 환경
- **OS**: Ubuntu 24.04
- **ROS**: ROS 2 Jazzy
- **언어**: C++, Python, C#, JAVA 등

## 프로젝트 구조
```
fta_actuators/          # 액추에이터 노드 구현
├── led_buzzer/         # LED Buzzer 제어
├── speaker/            # Speaker 제어
├── battery/            # 배터리 제어 (예정)
└── power_management/   # 저전력 모드 (예정)

fta_msgs/               # 커스텀 메시지 정의
fta_bringup/            # 런치 파일 및 설정
```

## 빌드 및 실행

### 빌드
```bash
colcon build --packages-select fta_*
source install/setup.bash
```

### 실행
```bash
# 전체 FTA 실행
ros2 launch fta_bringup fta.launch.py

# 개별 노드 실행
ros2 run fta_actuators led_buzzer_node
ros2 run fta_actuators speaker_node
```
