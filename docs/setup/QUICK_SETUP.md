# FTA 개발 환경 빠른 체크리스트

## ?? 현재 상황

- ? Visual Studio에서 C# 솔루션(.sln)으로 열려있음
- ? ROS2 파일들이 "기타 파일"로 표시됨
- ? CMake가 활성화되지 않음

---

## ? 빠른 해결 방법

### 방법 1: WSL2 + VS Code (강력 추천!)

```bash
# 1. WSL2 설치 (PowerShell 관리자)
wsl --install -d Ubuntu-22.04

# 2. Ubuntu 재부팅 후
# 3. ROS2 설치
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions nlohmann-json3-dev -y

# 4. VS Code에서
# Ctrl+Shift+P → "WSL: Connect to WSL"
# → 프로젝트 폴더 열기
```

### 방법 2: Visual Studio 2022 (제한적)

```
1. 파일 → 솔루션 닫기
2. 파일 → 열기 → CMake...
3. fta_interfaces/CMakeLists.txt 선택
```

---

## ?? Python 테스트 (pytest)

### 결론: **지금은 필요 없음**

- C++ 노드는 `colcon test`로 충분
- Python 노드 개발 시에만 필요
- 통합 테스트는 `test_led_buzzer.launch.py`로 가능

---

## ?? 다음 단계

### WSL2 설치 후

```bash
# 빌드
cd /mnt/c/Users/wmszz/source/repos/Forklift-Twin-Actuator
source /opt/ros/humble/setup.bash
colcon build --packages-select fta_interfaces fta_actuators

# 실행
source install/setup.bash
ros2 launch fta_actuators test_led_buzzer.launch.py
```

---

## ?? 상세 가이드

→ **DEVELOPMENT_SETUP.md** 참고
