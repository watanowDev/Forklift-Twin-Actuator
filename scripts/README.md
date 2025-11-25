# FTA 프로젝트 스크립트 모음

이 폴더에는 개발 편의를 위한 자동화 스크립트가 포함되어 있습니다.

## 📝 스크립트 목록

### 1. `deploy_to_remote.ps1` (Windows)
**용도**: 파일을 원격 Ubuntu 서버로 전송

**사용법**:
```powershell
.\scripts\deploy_to_remote.ps1
```

**전송되는 파일**:
- `fta_beginner/` - 초보자용 테스트 패키지
- `fta_interfaces/` - 메시지 정의
- `BEGINNER_GUIDE.md` - 초보자 가이드
- `CURRENT_STATUS.md` - 현재 작업 상태

---

### 2. `build_on_remote.sh` (Ubuntu/Linux)
**용도**: 원격 서버에서 프로젝트 빌드

**사용법**:
```bash
# SSH 접속 후
cd ~/Forklift-Twin-Actuator
./scripts/build_on_remote.sh
```

**수행 작업**:
1. ROS2 환경 설정
2. `fta_interfaces` 빌드
3. `fta_beginner` 빌드
4. 테스트 방법 안내

---

### 3. `deploy_and_build.ps1` (Windows) ⭐ 권장
**용도**: 파일 전송 + 원격 빌드를 한 번에 수행

**사용법**:
```powershell
# 일반 배포
.\scripts\deploy_and_build.ps1

# 이전 빌드 삭제 후 클린 빌드
.\scripts\deploy_and_build.ps1 -Clean

# 배포 후 바로 SSH 접속하여 테스트
.\scripts\deploy_and_build.ps1 -Test
```

**옵션**:
- `-Clean`: 이전 빌드 캐시 삭제 후 빌드
- `-Test`: 빌드 완료 후 자동으로 SSH 접속

---

## 🚀 추천 워크플로우

### 처음 시작할 때
```powershell
# Windows PowerShell
cd C:\Users\wmszz\source\repos\Forklift-Twin-Actuator
.\scripts\deploy_and_build.ps1 -Clean -Test
```

### 코드 수정 후 테스트
```powershell
# 1. 코드 수정
# 2. 배포 및 빌드
.\scripts\deploy_and_build.ps1

# 3. SSH 접속 (자동)
# 또는 수동 접속:
ssh wmszz@172.30.1.102
cd ~/Forklift-Twin-Actuator
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 빌드만 다시 하고 싶을 때
```bash
# Ubuntu에서
cd ~/Forklift-Twin-Actuator
./scripts/build_on_remote.sh
```

---

## 🔧 스크립트 커스터마이징

### 원격 서버 주소 변경
`deploy_and_build.ps1` 파일에서:
```powershell
$RemoteHost = "172.30.1.102"  # 여기를 수정
$RemoteUser = "wmszz"          # 여기를 수정
```

### 전송할 파일 추가
`deploy_and_build.ps1`의 `scp` 명령어에 파일/폴더 추가:
```powershell
scp -r fta_beginner fta_interfaces YOUR_NEW_FOLDER "$RemoteUser@$RemoteHost:$RemotePath/"
```

---

## ❓ 트러블슈팅

### SSH 비밀번호를 매번 입력해야 해요
SSH Key를 설정하세요:
```powershell
# Windows에서
ssh-keygen -t rsa
type $env:USERPROFILE\.ssh\id_rsa.pub | ssh wmszz@172.30.1.102 "cat >> ~/.ssh/authorized_keys"
```

### 스크립트 실행 권한 오류
```bash
# Ubuntu에서
chmod +x ~/Forklift-Twin-Actuator/scripts/*.sh
```

### PowerShell 실행 정책 오류
```powershell
# Windows PowerShell (관리자 권한)
Set-ExecutionPolicy RemoteSigned -Scope CurrentUser
```

---

## 📚 추가 정보

- 스크립트 사용 중 문제가 발생하면 [BEGINNER_GUIDE.md](../BEGINNER_GUIDE.md)를 참고하세요.
- 빌드 에러는 [CURRENT_STATUS.md](../CURRENT_STATUS.md)의 "트러블슈팅" 섹션을 확인하세요.
