# ?? FTA 프로젝트 재구성 완료!

## ? 완료된 작업

### 1. 불필요한 파일 정리
- ? `cpp/LEDModule/` 삭제 (독립 C++ 프로젝트)
- ? ROS2 패키지 구조로 통합

### 2. ROS2 메시지 인터페이스 생성
- ? `fta_interfaces/msg/ActionEvent.msg` - 액추에이터 제어 명령
- ? `fta_interfaces/msg/ActuatorStatus.msg` - 액추에이터 상태

### 3. Patlite LED/Buzzer 드라이버 구현
- ? `fta_actuators/include/fta_actuators/patlite_driver.hpp`
- ? `fta_actuators/src/patlite_driver.cpp`
- Windows/Linux 호환 (DLL/SO 동적 로딩)

### 4. ROS2 노드 구현
- ? `LEDBuzzerNode` - Patlite 제어 노드
- ? `/actions/event` 구독 (QoS1)
- ? `/actuators/status` 발행 (QoS1)
- ? JSON 파라미터 파싱 (nlohmann_json)

### 5. 테스트 노드 구현
- ? `test_action_publisher` - FTE 없이 테스트 가능
- ? 5초마다 자동으로 테스트 시나리오 실행

### 6. Launch 파일
- ? `led_buzzer.launch.py` - 실제 운영용
- ? `test_led_buzzer.launch.py` - 테스트용 (추천)

### 7. 설정 파일
- ? `config/led_buzzer.yaml` - 파라미터 설정

---

## ?? 최종 프로젝트 구조

```
Forklift-Twin-Actuator/
├── .github/
│   └── copilot-instructions.md       # FTA 개발 가이드
├── fta_interfaces/                   # ROS2 메시지 정의
│   ├── msg/
│   │   ├── ActionEvent.msg
│   │   └── ActuatorStatus.msg
│   ├── CMakeLists.txt
│   └── package.xml
├── fta_actuators/                    # LED/Buzzer 액추에이터 패키지
│   ├── include/fta_actuators/
│   │   ├── patlite_driver.hpp
│   │   └── led_buzzer_node.hpp
│   ├── src/
│   │   ├── patlite_driver.cpp
│   │   ├── led_buzzer_node.cpp
│   │   ├── led_buzzer_node_main.cpp
│   │   └── test_action_publisher.cpp
│   ├── launch/
│   │   ├── led_buzzer.launch.py
│   │   └── test_led_buzzer.launch.py
│   ├── config/
│   │   └── led_buzzer.yaml
│   ├── CMakeLists.txt
│   └── package.xml
├── reference/                        # C# 원본 프로젝트 (참고용)
│   └── WATA.LIS.INDICATOR.LED/
│       └── Libary/
│           └── NeUsbController.dll   ?? 이 DLL이 필요합니다
└── README_FTA.md                     # 프로젝트 README
```

---

## ?? 빠른 시작 가이드

### 1단계: 의존성 설치

```bash
# ROS2 Humble (Ubuntu 22.04)
source /opt/ros/humble/setup.bash

# nlohmann_json 설치
sudo apt-get install nlohmann-json3-dev

# rosdep 의존성 설치
cd ~/fta_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2단계: 빌드

```bash
cd ~/fta_ws
colcon build --packages-select fta_interfaces fta_actuators
source install/setup.bash
```

### 3단계: 테스트 실행

```bash
# 방법 A: Launch 파일 사용 (추천)
ros2 launch fta_actuators test_led_buzzer.launch.py

# 방법 B: 개별 노드 실행
# 터미널 1
ros2 run fta_actuators led_buzzer_node

# 터미널 2
ros2 run fta_actuators test_action_publisher
```

### 4단계: 상태 모니터링

```bash
# 다른 터미널에서
ros2 topic echo /actuators/status
```

---

## ?? 사용 예제

### 예제 1: LED 빨간색 점멸

```bash
ros2 topic pub --once /actions/event fta_interfaces/msg/ActionEvent "{
  actuator_type: 'led_buzzer',
  action: 'set_light',
  parameters: '{\"color\": \"red\", \"pattern\": \"flash1\"}'
}"
```

### 예제 2: 부저 울리기

```bash
ros2 topic pub --once /actions/event fta_interfaces/msg/ActionEvent "{
  actuator_type: 'led_buzzer',
  action: 'set_buzzer',
  parameters: '{\"pattern\": \"pattern6\", \"count\": 3}'
}"
```

### 예제 3: LED + 부저 동시 제어

```bash
ros2 topic pub --once /actions/event fta_interfaces/msg/ActionEvent "{
  actuator_type: 'led_buzzer',
  action: 'set_all',
  parameters: '{
    \"color\": \"yellow\",
    \"led_pattern\": \"continuous\",
    \"buzzer_pattern\": \"pattern1\",
    \"count\": 5
  }'
}"
```

### 예제 4: 모두 끄기

```bash
ros2 topic pub --once /actions/event fta_interfaces/msg/ActionEvent "{
  actuator_type: 'led_buzzer',
  action: 'clear',
  parameters: '{}'
}"
```

---

## ?? 지원하는 제어 명령

### 액션 (action)
- `set_light` - LED만 제어
- `set_buzzer` - 부저만 제어
- `set_all` - LED + 부저 동시 제어
- `clear` - 모두 끄기

### LED 색상 (color)
`"red"`, `"yellow"`, `"green"`, `"blue"`, `"white"`, `"skyblue"`, `"purple"`, `"clear"`

### LED 패턴 (pattern / led_pattern)
`"continuous"`, `"flash1"`, `"flash2"`, `"flash3"`, `"off"`

### 부저 패턴 (buzzer_pattern)
`"pattern1"` ~ `"pattern11"`, `"stop"`

### 부저 횟수 (count)
정수 (예: 1, 3, 5)

---

## ?? NeUsbController.dll 준비 (Windows)

### DLL 위치
```
reference/WATA.LIS.INDICATOR.LED/Libary/NeUsbController.dll
```

### DLL 배치 방법

**방법 1: PATH에 추가**
```bash
# 환경 변수 설정 (PowerShell)
$env:PATH += ";C:\Users\wmszz\source\repos\Forklift-Twin-Actuator\reference\WATA.LIS.INDICATOR.LED\Libary"
```

**방법 2: 실행 파일 옆에 복사**
```bash
# 빌드 후
cp reference/WATA.LIS.INDICATOR.LED/Libary/NeUsbController.dll install/fta_actuators/lib/fta_actuators/
```

---

## ?? 테스트 시나리오

`test_action_publisher` 노드는 5초마다 다음 시나리오를 순환합니다:

1. **초록색 연속 점등** - 정상 상태
2. **노란색 점멸** - 경고 상태
3. **빨간색 + 부저** - 에러 상태
4. **부저만** - 알림
5. **모두 끄기** - 대기

---

## ?? ROS2 토픽 구조

```
┌─────────────┐
│     FTE     │  (미구현, 테스트용 대체: test_action_publisher)
│   (Engine)  │
└──────┬──────┘
       │ /actions/event (QoS1)
       │ ActionEvent
       │   - actuator_type: "led_buzzer"
       │   - action: "set_light" | "set_buzzer" | "set_all" | "clear"
       │   - parameters: JSON string
       ▼
┌─────────────────────┐
│  LED/Buzzer Node    │
│  (fta_actuators)    │
│                     │
│  ┌───────────────┐  │
│  │ Patlite       │  │
│  │ Driver        │──┼──→ NeUsbController.dll
│  │               │  │         │
│  │ - set_light() │  │         │
│  │ - set_buzzer()│  │         ▼
│  └───────────────┘  │    USB Device
└──────┬──────────────┘    (Patlite)
       │
       │ /actuators/status (QoS1)
       │ ActuatorStatus
       │   - actuator_type: "led_buzzer"
       │   - status: "idle" | "active" | "error"
       │   - message: string
       │   - error_code: int32
       ▼
┌─────────────┐
│  Monitoring │
│  Tools      │
│  (rqt, etc) │
└─────────────┘
```

---

## ??? 개발 가이드

### 새로운 액추에이터 추가 시

1. **드라이버 작성**
   - `include/fta_actuators/your_driver.hpp`
   - `src/your_driver.cpp`

2. **노드 작성**
   - `include/fta_actuators/your_node.hpp`
   - `src/your_node.cpp`

3. **CMakeLists.txt 수정**
   - 새 라이브러리 및 실행 파일 추가

4. **Launch 파일 추가**
   - `launch/your_actuator.launch.py`

5. **테스트**
   - `src/test_your_actuator.cpp`

### 코딩 규칙 (from .github/copilot-instructions.md)

- **언어**: C++17 이상
- **네이밍**:
  - 패키지: `fta_*`
  - 노드: `snake_case`
  - 클래스: `PascalCase`
  - 함수/변수: `snake_case`
- **QoS**: 액추에이터는 QoS1 (Reliable) 사용
- **에러 처리**: `/actuators/status`에 에러 코드 포함

---

## ?? 주의사항

### Windows에서 개발 시
- NeUsbController.dll이 필수
- Visual Studio 또는 WSL2 사용 권장

### Linux (Ubuntu 22.04)에서 배포 시
- Patlite의 Linux 드라이버가 있는지 확인
- 없으면 `libNeUsbController.so` 제작 필요

### 디바이스가 없을 때
- `test_led_buzzer.launch.py`로 토픽 통신 테스트 가능
- 실제 하드웨어 제어는 불가 (드라이버 연결 실패)

---

## ?? 다음 단계

### 단기 (현재 완료)
- ? Patlite LED/Buzzer ROS2 노드 구현
- ? 테스트 노드 구현
- ? 메시지 인터페이스 정의

### 중기 (향후 개발)
- ? Speaker 액추에이터 추가
- ? Battery 제어 액추에이터 추가
- ? 통합 Launch 파일 (`fta_bringup`)

### 장기
- ? FTE (Engine) 개발 - 중앙 관제
- ? FTL (Logger) 연동
- ? FTC/FTV UI 연동

---

## ?? 문제 해결

### Q1: 빌드 에러 - "nlohmann/json.hpp not found"

```bash
sudo apt-get install nlohmann-json3-dev
```

### Q2: 디바이스 연결 실패

```
1. Patlite USB 연결 확인
2. NeUsbController.dll 경로 확인
3. 드라이버 설치 확인
4. 다른 프로그램에서 사용 중인지 확인
```

### Q3: 토픽이 보이지 않음

```bash
# ROS2 환경 소싱 확인
source install/setup.bash

# 노드 실행 확인
ros2 node list

# 토픽 확인
ros2 topic list
```

---

## ?? 완료!

이제 ROS2 기반의 FTA 프로젝트가 완성되었습니다!

**테스트 명령:**
```bash
ros2 launch fta_actuators test_led_buzzer.launch.py
```

**실제 사용:**
```bash
ros2 launch fta_actuators led_buzzer.launch.py
```

질문이나 문제가 있으면 `.github/copilot-instructions.md`를 참고하세요! ??
