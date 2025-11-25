# FTA (Forklift Twin Actuators) 개발 가이드

## 프로젝트 개요
FTA는 지게차 액추에이터 제어를 담당하는 ROS2 기반 모듈입니다. `/actions/event` 토픽을 구독하여 물리적 제어를 수행하고, 결과를 `/actuators/status` 토픽으로 발행합니다.

## 시스템 아키텍처 이해

### 전체 시스템 컨텍스트
FTA는 Forklift Twin 시스템의 5개 독립 프로젝트 중 하나입니다:
- **FTE (Engine)**: 데이터 허브 & 중앙 관제 - ROS2 ↔ 내/외부망 게이트웨이
- **FTA (Actuators)**: 액추에이터 제어 (이 프로젝트)
- **FTL (Logger)**: SQLite 기반 시스템 로거
- **FTC (Console)**: 작업자용 태블릿 UI (WebView)
- **FTV (Viewer)**: 엔지니어용 원격 모니터링 UI (WebView)

### 느슨한 결합 원칙
- 각 모듈은 독립적으로 실행 가능
- ROS2 토픽을 통한 표준화된 통신만 사용
- 하드웨어 제조사별 드라이버는 분리하여 관리

## ROS2 통신 패턴

### 구독 토픽
- **`/actions/event`** (QoS1): FTE로부터 액추에이터 제어 명령 수신
  - 메시지 타입: 사전 정의된 액션 이벤트 구조
  - 예: LED 제어, Buzzer 알람, Speaker 음성 출력

### 발행 토픽
- **`/actuators/status`** (QoS1): 액추에이터 상태 및 제어 결과 발행
  - 현재 상태, 제어 성공/실패, 에러 코드 포함

### QoS 정책
- **센서**: QoS0 (최선 노력)
- **액추에이터 및 제어 토픽**: QoS1 (신뢰성 보장) ← FTA는 이것 사용

## 액추에이터 모듈 구조

### 현재 지원 액추에이터
1. **LED Buzzer**: 시각/청각 알람 제어
2. **Speaker**: 음성 출력 및 사운드 재생
3. **배터리 제어**: 전력계 MCU 연동 (11.23 이후 개발)
4. **저전력 모드**: 절전 모드 전환 (11.23 이후 개발)

### 하드웨어 독립성 패턴
```
액추에이터 노드
├── interface/ (공통 인터페이스)
├── drivers/   (제조사별 드라이버)
│   ├── manufacturer_a/
│   └── manufacturer_b/
└── control/   (제어 로직)
```

각 액추에이터는:
- 공통 인터페이스를 구현
- 드라이버는 플러그인 형태로 교체 가능
- `/actions/event` 구독 → 제어 수행 → `/actuators/status` 발행 패턴 준수

## 개발 환경

### 타겟 환경
- **OS**: Linux (Ubuntu 24.04)
- **ROS 버전**: ROS 2 Jazzy
- **개발 PC**: Windows (SSH 원격 빌드)
- **원격 PC**: Ubuntu 24.04 (172.30.1.102)
  - **호스트명**: wata
  - **사용자명**: wata
  - **SSH 접속**: `ssh wata@172.30.1.102`

### 빌드 및 실행
```bash
# 원격 PC SSH 접속
ssh wata@172.30.1.102

# 워크스페이스 빌드
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fta_interfaces fta_actuators

# 환경 설정 및 실행
source install/setup.bash
ros2 run fta_actuators led_buzzer_node

# 전체 FTA 실행 (런치 파일)
ros2 launch fta_bringup fta.launch.py
```

## 코딩 규칙

### 언어
- **C++**: C++17 이상 사용
- ROS2 C++ 노드로 구현

### 네이밍 컨벤션
- 패키지명: `fta_` 접두사 사용 (예: `fta_actuators`, `fta_bringup`)
- 토픽명: 슬래시로 구분된 소문자 (예: `/actions/event`, `/actuators/status`)
- 노드명: `snake_case` (예: `led_buzzer_node`, `speaker_node`)
- 클래스명: `PascalCase` (예: `LEDBuzzerNode`, `SpeakerNode`)
- 함수/변수명: `snake_case` (예: `initialize_hardware`, `send_command`)

### Git 커밋 메시지
**한국어 사용**: 한국인 개발자 팀이므로 커밋 메시지는 한국어로 작성하며, 필요시 영문을 병기할 수 있습니다.

### 에러 처리
- 하드웨어 통신 실패 시 재시도 로직 구현
- `/actuators/status`에 에러 코드 및 상세 메시지 포함
- FTL(Logger)에 에러 로그 전송

### 성능 고려사항
- 시스템 경량화: FTA는 ROS2 런타임에서만 실행
- 과부하 방지: 불필요한 고빈도 발행 자제
- 리소스 관리: 사용하지 않는 드라이버는 로드하지 않음

## 확장 가능성

### 센서 추가 시
FTA는 센서 데이터를 직접 다루지 않습니다. 센서는 별도 노드에서 구현하고 융합/판단 모듈을 통해 `/actions/event`로 명령이 전달됩니다.

### 새 액추에이터 추가 시
1. `fta_actuators` 패키지에 새 노드 추가
2. 공통 인터페이스 상속
3. `/actions/event` 구독 및 `/actuators/status` 발행 구현
4. 런치 파일에 노드 추가
