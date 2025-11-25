# ========================================
# 올인원 스크립트: 전송 + 빌드를 한 번에!
# Windows PowerShell에서 실행
# ========================================

param(
    [switch]$Clean,  # 이전 빌드 삭제
    [switch]$Test    # 빌드 후 바로 테스트
)

$RemoteHost = "172.30.1.102"
$RemoteUser = "wmszz"
$RemotePath = "~/Forklift-Twin-Actuator"

Write-Host ""
Write-Host "================================" -ForegroundColor Cyan
Write-Host "FTA 프로젝트 배포 스크립트" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host "원격 서버: $RemoteHost" -ForegroundColor Green
Write-Host "사용자: $RemoteUser" -ForegroundColor Green
Write-Host ""

# Step 1: 현재 위치 확인
Write-Host "[1/5] 프로젝트 디렉토리 확인..." -ForegroundColor Yellow
Set-Location C:\Users\wmszz\source\repos\Forklift-Twin-Actuator
Write-Host "  ✓ 현재 위치: $(Get-Location)" -ForegroundColor Green
Write-Host ""

# Step 2: Git 상태 확인 (선택사항)
Write-Host "[2/5] Git 상태 확인..." -ForegroundColor Yellow
git status --short
Write-Host ""

# Step 3: 파일 전송
Write-Host "[3/5] 파일 전송 중..." -ForegroundColor Yellow
Write-Host "  전송할 파일:" -ForegroundColor Cyan
Write-Host "    - fta_beginner/" -ForegroundColor White
Write-Host "    - fta_interfaces/" -ForegroundColor White
Write-Host "    - 문서 파일들" -ForegroundColor White
Write-Host ""

# 원격 디렉토리 생성
ssh "$RemoteUser@$RemoteHost" "mkdir -p $RemotePath"

# 파일 전송
scp -r fta_beginner fta_interfaces BEGINNER_GUIDE.md CURRENT_STATUS.md scripts "$RemoteUser@$RemoteHost:$RemotePath/"

if ($LASTEXITCODE -eq 0) {
    Write-Host "  ✓ 파일 전송 완료!" -ForegroundColor Green
} else {
    Write-Host "  ✗ 파일 전송 실패!" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Step 4: 원격 빌드
Write-Host "[4/5] 원격 빌드 시작..." -ForegroundColor Yellow

# 빌드 스크립트에 실행 권한 부여
ssh "$RemoteUser@$RemoteHost" "chmod +x $RemotePath/scripts/build_on_remote.sh"

# Clean 옵션 처리
$CleanCommand = ""
if ($Clean) {
    Write-Host "  [Clean 모드] 이전 빌드 삭제" -ForegroundColor Yellow
    ssh "$RemoteUser@$RemoteHost" "cd $RemotePath && rm -rf build/ install/ log/"
}

# 빌드 실행
$BuildCommand = @"
cd $RemotePath
source /opt/ros/jazzy/setup.bash
echo '=== fta_interfaces 빌드 ==='
colcon build --packages-select fta_interfaces
source install/setup.bash
echo '=== fta_beginner 빌드 ==='
colcon build --packages-select fta_beginner
source install/setup.bash
echo '빌드 완료!'
"@

ssh "$RemoteUser@$RemoteHost" "bash -c '$BuildCommand'"

if ($LASTEXITCODE -eq 0) {
    Write-Host "  ✓ 빌드 성공!" -ForegroundColor Green
} else {
    Write-Host "  ✗ 빌드 실패!" -ForegroundColor Red
    exit 1
}
Write-Host ""

# Step 5: 완료 메시지
Write-Host "================================" -ForegroundColor Green
Write-Host "배포 완료!" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green
Write-Host ""

# 테스트 옵션
if ($Test) {
    Write-Host "테스트 모드가 활성화되었습니다." -ForegroundColor Yellow
    Write-Host "SSH 접속 후 테스트를 시작합니다..." -ForegroundColor Yellow
    Write-Host ""
    
    ssh -t "$RemoteUser@$RemoteHost" "cd $RemotePath && source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash"
} else {
    # 다음 단계 안내
    Write-Host "다음 단계: SSH 접속 후 테스트" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "  ssh $RemoteUser@$RemoteHost" -ForegroundColor White
    Write-Host "  cd $RemotePath" -ForegroundColor White
    Write-Host "  source /opt/ros/jazzy/setup.bash" -ForegroundColor White
    Write-Host "  source install/setup.bash" -ForegroundColor White
    Write-Host ""
    Write-Host "테스트 명령어:" -ForegroundColor Cyan
    Write-Host "  ros2 run fta_beginner simple_publisher" -ForegroundColor White
    Write-Host "  ros2 run fta_beginner simple_subscriber" -ForegroundColor White
    Write-Host "  ros2 run fta_beginner patlite_test" -ForegroundColor White
    Write-Host ""
}

Write-Host "사용 예시:" -ForegroundColor Yellow
Write-Host "  .\scripts\deploy_and_build.ps1           # 일반 배포" -ForegroundColor White
Write-Host "  .\scripts\deploy_and_build.ps1 -Clean    # 클린 빌드" -ForegroundColor White
Write-Host "  .\scripts\deploy_and_build.ps1 -Test     # 배포 후 바로 SSH 접속" -ForegroundColor White
Write-Host ""
