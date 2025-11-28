# FTA Actuators 디렉토리 구조 개선안

## 현재 구조의 문제점
- Patlite 관련 파일들이 여러 위치에 분산됨
- 추후 다른 액추에이터(Speaker, Battery 등) 추가 시 구조가 복잡해질 수 있음
- 액추에이터별 독립성이 부족함

## 개선된 디렉토리 구조

```
fta_actuators/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── config/                          # 전역 설정 파일
│   └── actuators_config.yaml
│
├── launch/                          # 전역 런치 파일
│   ├── all_actuators.launch.py
│   └── patlite_only.launch.py
│
├── include/fta_actuators/
│   ├── common/                      # 공통 인터페이스 및 유틸리티
│   │   ├── actuator_interface.hpp  # 액추에이터 기본 인터페이스
│   │   └── actuator_types.hpp      # 공통 타입 정의
│   │
│   ├── patlite/                     # Patlite LED/Buzzer 모듈
│   │   ├── patlite_types.hpp       # Patlite 전용 타입
│   │   ├── patlite_controller.hpp  # 제어 로직 (C# 포팅)
│   │   ├── patlite_driver.hpp      # 하드웨어 드라이버 인터페이스
│   │   ├── patlite_usb_driver.hpp  # USB 통신 구현
│   │   ├── patlite_scenarios.hpp   # 시나리오 매핑
│   │   └── patlite_node.hpp        # ROS2 노드
│   │
│   ├── speaker/                     # (추후) Speaker 모듈
│   │   ├── speaker_types.hpp
│   │   ├── speaker_controller.hpp
│   │   ├── speaker_driver.hpp
│   │   └── speaker_node.hpp
│   │
│   ├── battery/                     # (추후) 배터리 제어 모듈
│   │   ├── battery_types.hpp
│   │   ├── battery_controller.hpp
│   │   ├── battery_driver.hpp
│   │   └── battery_node.hpp
│   │
│   └── power/                       # (추후) 저전력 모드 모듈
│       ├── power_types.hpp
│       ├── power_controller.hpp
│       └── power_node.hpp
│
├── src/
│   ├── common/                      # 공통 구현
│   │   └── actuator_interface.cpp
│   │
│   ├── patlite/                     # Patlite 구현
│   │   ├── patlite_controller.cpp
│   │   ├── patlite_driver.cpp
│   │   ├── patlite_usb_driver.cpp
│   │   ├── patlite_scenarios.cpp
│   │   ├── patlite_node.cpp
│   │   └── patlite_node_main.cpp   # 실행 파일 진입점
│   │
│   ├── speaker/                     # (추후) Speaker 구현
│   │   ├── speaker_controller.cpp
│   │   ├── speaker_driver.cpp
│   │   ├── speaker_node.cpp
│   │   └── speaker_node_main.cpp
│   │
│   ├── battery/                     # (추후) Battery 구현
│   │   └── ...
│   │
│   └── tests/                       # 테스트 파일들
│       ├── test_patlite_action_publisher.cpp
│       ├── test_patlite_buzzer_sound.cpp
│       └── test_speaker_tts.cpp
│
├── docs/                            # 액추에이터별 문서
│   ├── PATLITE.md
│   ├── SPEAKER.md
│   ├── BATTERY.md
│   └── ARCHITECTURE.md
│
└── reference/                       # 참조용 원본 코드 (이미 상위에 있음)
```

## 주요 개선 사항

### 1. 액추에이터별 모듈화
- 각 액추에이터가 독립적인 서브디렉토리를 가짐
- `patlite/`, `speaker/`, `battery/`, `power/` 등으로 명확히 분리
- 각 모듈은 자체적인 types, controller, driver, node를 포함

### 2. 공통 인터페이스 분리
- `common/` 디렉토리에 모든 액추에이터가 공유하는 인터페이스 정의
- 일관된 API를 통해 새 액추에이터 추가 용이

### 3. 명확한 파일 명명 규칙
- `led_buzzer_node` → `patlite_node` (제조사명 명확화)
- `test_action_publisher` → `test_patlite_action_publisher` (대상 명확화)

### 4. 문서화 개선
- 각 액추에이터별 독립 문서 (`docs/PATLITE.md`)
- 전체 아키텍처 문서 (`docs/ARCHITECTURE.md`)

## 마이그레이션 단계

### Phase 1: Patlite 모듈 재구성 (지금)
1. ✅ `patlite_types.hpp` 생성 완료
2. ✅ `patlite_controller.hpp/cpp` 생성 완료
3. ⏳ 파일 이동 및 이름 변경
4. ⏳ CMakeLists.txt 업데이트
5. ⏳ 빌드 및 테스트

### Phase 2: 공통 인터페이스 정의 (다음)
1. `common/actuator_interface.hpp` 설계
2. Patlite 모듈이 공통 인터페이스 구현하도록 리팩토링

### Phase 3: 새 액추에이터 추가 (추후)
1. Speaker 모듈 추가
2. Battery 모듈 추가
3. Power 모듈 추가

## 이점

### 확장성
- 새 액추에이터 추가 시 기존 코드 수정 최소화
- 독립적인 개발 및 테스트 가능

### 유지보수성
- 각 모듈의 책임이 명확함
- 파일 찾기 용이

### 재사용성
- 공통 인터페이스를 통한 코드 재사용
- 드라이버 교체 용이 (USB → Serial → Network)

## 네이밍 컨벤션

### 파일명
- 헤더: `{module}_{component}.hpp` (예: `patlite_controller.hpp`)
- 구현: `{module}_{component}.cpp` (예: `patlite_controller.cpp`)
- 노드: `{module}_node.hpp/cpp` (예: `patlite_node.hpp`)

### 클래스명
- `{Module}{Component}` (예: `PatliteController`, `SpeakerNode`)

### 네임스페이스
- `fta_actuators::{module}` (예: `fta_actuators::patlite`)
