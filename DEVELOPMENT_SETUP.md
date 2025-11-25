# FTA 개발 환경 설정 가이드

## ?? 현재 문제 및 해결 방법

### ? **문제 1: Visual Studio에서 "기타 파일"로 표시**

**원인**: C# 솔루션(.sln) 파일로 열어서 ROS2 C++ 프로젝트가 인식되지 않음

**해결**:
- ROS2 프로젝트는 CMake 기반이므로 CMake 프로젝트로 열어야 함
- 또는 **VS Code + WSL2**를 사용 (ROS2 개발 표준)

### ? **문제 2: CMake가 활성화되지 않음**

**원인**: C# 솔루션 컨텍스트에서는 CMake가 작동하지 않음

**해결**: 아래 방법 중 하나 선택

---

## ?? **방법 1: Visual Studio 2022 (Windows 네이티브)**

### 장점
- Windows에서 직접 개발 가능
- GUI 디버거 사용

### 단점
- ROS2는 Linux 기반이므로 제약 있음
- NeUsbController.dll만 Windows에서 작동

### 설정 방법

1. **현재 솔루션 닫기**
   ```
   파일 → 솔루션 닫기
   ```

2. **CMake 프로젝트 열기**
   ```
   파일 → 열기 → CMake...
   → fta_interfaces/CMakeLists.txt 선택
   ```

3. **CMake 설정 생성**
   - Visual Studio가 자동으로 캐시 생성
   - `CMakeSettings.json` 파일이 생성됨

4. **빌드**
   ```
   빌드 → 모두 빌드
   ```

### CMakeSettings.json 예제

```json
{
  "configurations": [
    {
      "name": "x64-Debug",
      "generator": "Ninja",
      "configurationType": "Debug",
      "buildRoot": "${projectDir}\\out\\build\\${name}",
      "installRoot": "${projectDir}\\out\\install\\${name}",
      "cmakeCommandArgs": "",
      "buildCommandArgs": "",
      "ctestCommandArgs": "",
      "inheritEnvironments": [ "msvc_x64_x64" ],
      "variables": []
    }
  ]
}
```

---

## ?? **방법 2: VS Code + WSL2 (강력 추천!)**

### 장점 ?
- **ROS2 네이티브 환경** (Ubuntu 22.04)
- colcon 빌드 완벽 지원
- ROS2 확장 플러그인 사용 가능
- Linux 환경에서 개발 → 실제 배포 환경과 동일

### 단점
- WSL2 설치 필요
- Windows DLL (NeUsbController.dll)은 WSL에서 작동 안 함
  - 해결: Linux용 드라이버 사용 또는 Windows에서만 테스트

### 설정 방법

#### 1. WSL2 설치 (PowerShell 관리자 권한)

```powershell
wsl --install -d Ubuntu-22.04
```

재부팅 후:

```bash
# Ubuntu 22.04 실행
# 사용자 계정 생성
```

#### 2. ROS2 Humble 설치

```bash
# Ubuntu 업데이트
sudo apt update && sudo apt upgrade -y

# ROS2 저장소 추가
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install ros-humble-desktop -y

# 개발 도구 설치
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-vcstool -y

# nlohmann-json 설치
sudo apt install nlohmann-json3-dev -y

# rosdep 초기화
sudo rosdep init
rosdep update
```

#### 3. VS Code 설정

**Windows에서 VS Code 실행 후:**

1. **확장 설치**
   - `Remote - WSL`
   - `C/C++`
   - `CMake Tools`
   - `ROS` (ms-iot.vscode-ros)

2. **WSL에서 폴더 열기**
   ```
   Ctrl+Shift+P → "WSL: Connect to WSL"
   → Ubuntu-22.04 선택
   → 폴더 열기: /mnt/c/Users/wmszz/source/repos/Forklift-Twin-Actuator
   ```

3. **VS Code 설정 확인**
   - `.vscode/` 폴더가 자동 생성됨
   - `c_cpp_properties.json`, `tasks.json`, `launch.json` 확인

#### 4. 빌드 및 실행

```bash
# 터미널에서 (VS Code 내장 터미널)
cd /mnt/c/Users/wmszz/source/repos/Forklift-Twin-Actuator

# ROS2 환경 소싱
source /opt/ros/humble/setup.bash

# 빌드
colcon build --packages-select fta_interfaces fta_actuators

# 환경 소싱
source install/setup.bash

# 실행
ros2 launch fta_actuators test_led_buzzer.launch.py
```

---

## ?? **Python 테스트 필요성**

### 결론: **지금은 필요 없음**

#### Python 테스트가 필요한 경우
- ROS2 Python 노드 개발 시
- Python 라이브러리 개발 시
- Integration 테스트를 Python으로 작성 시

#### 현재 프로젝트 (C++ 노드)
- **C++ 테스트**: `colcon test` 또는 직접 노드 실행
- **통합 테스트**: `test_led_buzzer.launch.py`로 충분

### 만약 Python 테스트를 추가한다면

```bash
# pytest 설치
sudo apt install python3-pytest -y

# 테스트 실행
colcon test --packages-select fta_actuators
colcon test-result --verbose
```

---

## ??? **추천 개발 흐름**

### 개발 단계별 도구 선택

| 단계 | 도구 | 이유 |
|------|------|------|
| **초기 개발** | VS Code + WSL2 | ROS2 네이티브 환경 |
| **Windows DLL 테스트** | Visual Studio 2022 | NeUsbController.dll 디버깅 |
| **최종 배포** | Linux (Ubuntu 22.04) | 실제 운영 환경 |

### 추천 워크플로우

1. **VS Code + WSL2**에서 개발
   - C++ 코드 작성
   - ROS2 메시지 정의
   - Launch 파일 작성

2. **WSL2에서 빌드 및 테스트**
   ```bash
   colcon build
   ros2 launch fta_actuators test_led_buzzer.launch.py
   ```

3. **Windows에서 DLL 테스트 필요 시**
   - Visual Studio 2022로 전환
   - CMake 프로젝트로 열기
   - DLL 함수 호출 디버깅

---

## ?? **지금 당장 해야 할 일**

### 옵션 A: WSL2 설치 (추천)

```powershell
# PowerShell 관리자 권한
wsl --install -d Ubuntu-22.04
```

재부팅 후 위의 "방법 2" 따라하기

### 옵션 B: Visual Studio에서 계속 (제한적)

1. Visual Studio에서 솔루션 닫기
2. `파일` → `열기` → `CMake...`
3. `fta_interfaces/CMakeLists.txt` 선택
4. CMake 캐시 생성 대기

---

## ?? **CMake 오류 해결**

### 오류 1: "find_package(ament_cmake) not found"

**원인**: ROS2가 설치되지 않음

**해결 (WSL2)**:
```bash
sudo apt install ros-humble-desktop -y
source /opt/ros/humble/setup.bash
```

**해결 (Windows)**: 불가능, WSL2 사용 필요

### 오류 2: "nlohmann/json.hpp not found"

```bash
sudo apt install nlohmann-json3-dev -y
```

### 오류 3: "colcon: command not found"

```bash
sudo apt install python3-colcon-common-extensions -y
```

---

## ?? **다음 단계**

1. **개발 환경 선택** (WSL2 추천)
2. **빌드 테스트**
   ```bash
   colcon build --packages-select fta_interfaces fta_actuators
   ```
3. **노드 실행 테스트**
   ```bash
   ros2 launch fta_actuators test_led_buzzer.launch.py
   ```
4. **실제 Patlite 디바이스 연결 테스트**

---

## ?? **FAQ**

### Q1: Visual Studio와 VS Code 중 뭘 써야 하나요?

**A**: ROS2 개발은 **VS Code + WSL2** 강력 추천
- Visual Studio는 C# 개발에 최적화
- ROS2는 Linux 기반이므로 WSL2가 필수

### Q2: Windows에서 ROS2 개발이 불가능한가요?

**A**: 가능하지만 제약이 많음
- ROS2 Windows 빌드는 실험적 단계
- 대부분의 패키지가 Linux 전용
- **WSL2 사용이 표준**

### Q3: pytest가 꼭 필요한가요?

**A**: 아니요
- C++ 노드는 `colcon test` 사용
- Python 노드 개발 시에만 필요

### Q4: 왜 "기타 파일"로 표시되나요?

**A**: C# 솔루션(.sln)으로 열어서
- ROS2 프로젝트는 CMake 기반
- VS Code + WSL2로 해결

---

## ?? **정리**

### ? 해야 할 것
1. **WSL2 설치** (가장 중요!)
2. **VS Code + WSL 확장 설치**
3. **ROS2 Humble 설치** (WSL2 내부)

### ? 하지 않아도 되는 것
1. pytest 설치 (지금은 불필요)
2. Visual Studio에서 C# 솔루션 유지
3. Windows에서 ROS2 네이티브 빌드

---

**다음 단계**: WSL2 설치 후 알려주시면 ROS2 설정을 도와드리겠습니다! ??
