# FTA 시스템 아키텍처

## 전체 시스템 구성

```
┌─────────────────────────────────────────────────────────────┐
│                    Forklift Twin System                     │
├─────────────┬─────────────┬─────────────┬──────────────────┤
│     FTE     │     FTA     │     FTL     │   FTC / FTV      │
│   (Engine)  │ (Actuators) │  (Logger)   │ (Console/Viewer) │
└─────────────┴─────────────┴─────────────┴──────────────────┘
       │             │             │              │
       └─────────────┴─────────────┴──────────────┘
                     ROS2 DDS (내부망)
                     
       ┌─────────────────────────────────────┐
       │  WebSocket / REST API (외부망)      │
       └─────────────────────────────────────┘
```

## FTA 내부 아키텍처

### 컴포넌트 구조
```
FTA (Forklift Twin Actuators)
│
├── fta_actuators/ (액추에이터 노드)
│   ├── LEDBuzzerNode
│   │   ├── interface/          # 공통 인터페이스
│   │   ├── drivers/            # 제조사별 드라이버
│   │   │   ├── gpio_driver/    # GPIO 기반
│   │   │   └── serial_driver/  # 시리얼 통신
│   │   └── control/            # 제어 로직
│   │
│   ├── SpeakerNode
│   │   ├── interface/
│   │   ├── drivers/
│   │   │   ├── alsa_driver/    # ALSA 오디오
│   │   │   └── usb_driver/     # USB 오디오
│   │   └── control/
│   │
│   ├── BatteryNode (11.23~)
│   │   └── mcu_interface/      # 전력계 MCU 통신
│   │
│   └── PowerManagementNode (11.23~)
│       └── power_control/      # 절전 모드 제어
│
├── fta_msgs/ (메시지 정의)
│   ├── ActuatorCommand.msg
│   ├── ActuatorStatus.msg
│   ├── LEDBuzzerControl.msg
│   └── SpeakerControl.msg
│
└── fta_bringup/ (실행 설정)
    ├── launch/
    │   ├── fta.launch.py
    │   └── actuators.launch.py
    └── config/
        └── actuators.yaml
```

### 데이터 흐름

```
[FTE] --/actions/event--> [FTA] --/actuators/status--> [FTE]
                            │
                            ├── LED Buzzer 제어
                            ├── Speaker 제어
                            ├── 배터리 제어
                            └── 저전력 모드
```

#### 1. 명령 수신 (Subscribe)
```
/actions/event (QoS1)
└── ActuatorCommand
    ├── target: "led" | "buzzer" | "speaker"
    ├── action: "on" | "off" | "play"
    ├── parameters: {...}
    └── timestamp
```

#### 2. 제어 실행
```
FTA Node
├── 명령 파싱
├── 드라이버 선택
├── 하드웨어 제어
└── 결과 기록
```

#### 3. 상태 발행 (Publish)
```
/actuators/status (QoS1)
└── ActuatorStatus
    ├── actuator_id
    ├── success: true | false
    ├── error_code
    ├── error_message
    └── timestamp
```

## 통신 프로토콜

### ROS2 DDS (내부망)
- **용도**: FTA ↔ FTE, 센서 노드, 융합/판단 모듈
- **QoS**: QoS1 (신뢰성 보장)
- **전송 방식**: Pub/Sub 패턴

### QoS 정책 비교
| 컴포넌트 | QoS | 이유 |
|---------|-----|------|
| 센서 데이터 | QoS0 | 최신 데이터가 중요, 일부 손실 허용 |
| 액추에이터 제어 | QoS1 | 명령 손실 불가, 신뢰성 최우선 |
| 상태 보고 | QoS1 | 모든 상태 변화 추적 필요 |

## 하드웨어 독립성

### 드라이버 플러그인 패턴
```cpp
// 1. 공통 인터페이스
class ActuatorInterface {
public:
    virtual bool initialize() = 0;
    virtual bool execute(const Command& cmd) = 0;
    virtual Status getStatus() = 0;
    virtual ~ActuatorInterface() = default;
};

// 2. 제조사별 구현
class LEDDriver_GPIO : public ActuatorInterface {
    bool initialize() override {
        // GPIO 초기화
    }
    bool execute(const Command& cmd) override {
        // GPIO 제어
    }
};

class LEDDriver_Serial : public ActuatorInterface {
    bool initialize() override {
        // 시리얼 포트 초기화
    }
    bool execute(const Command& cmd) override {
        // 시리얼 명령 전송
    }
};

// 3. 런타임 선택
std::unique_ptr<ActuatorInterface> driver;
if (config.driver_type == "gpio") {
    driver = std::make_unique<LEDDriver_GPIO>();
} else if (config.driver_type == "serial") {
    driver = std::make_unique<LEDDriver_Serial>();
}
```

## 확장성 설계

### 새 액추에이터 추가 절차
1. `fta_msgs`에 메시지 타입 정의
2. `fta_actuators`에 노드 클래스 생성
3. 공통 인터페이스 상속
4. 드라이버 구현
5. 런치 파일 업데이트
6. 테스트 작성

### 센서 융합 확장
FTA는 직접 센서 데이터를 처리하지 않습니다.
센서 → 융합/판단 모듈 → `/actions/event` → FTA 경로를 따릅니다.

## 성능 및 리소스 관리

### 시스템 경량화
- ROS2 런타임에서만 실행 (FTA, FTE, FTL)
- FTC, FTV는 필요시에만 실행
- 사용하지 않는 드라이버는 동적 로드/언로드

### 과부하 방지
- 토픽 발행 빈도 제한 (최소 100ms 간격)
- 명령 큐 사이즈 제한
- 하드웨어 응답 타임아웃 설정

## 에러 복구 전략

### 1. 하드웨어 통신 실패
```
재시도 로직 (최대 3회)
└── 실패 시 /actuators/status에 에러 발행
    └── FTL에 로그 기록
        └── FTE가 상위 시스템에 알림
```

### 2. 노드 크래시
```
ROS2 lifecycle 관리
└── WatchDog (FTE)가 감지
    └── 자동 재시작 시도
        └── 재시작 실패 시 시스템 알람
```

## 보안 고려사항

### 접근 제어
- `/actions/event` 토픽은 FTE만 발행 가능
- 외부 클라이언트 접속 이력은 FTL에 기록

### 명령 검증
```cpp
bool validateCommand(const Command& cmd) {
    // 명령 타입 확인
    // 파라미터 범위 검증
    // 권한 확인
}
```
