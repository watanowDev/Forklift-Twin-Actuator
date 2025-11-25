# ========================================
# Windows에서 실행: 파일을 Ubuntu로 전송
# ========================================

# 1. 프로젝트 디렉토리로 이동
Write-Host "================================" -ForegroundColor Cyan
Write-Host "1. 프로젝트 디렉토리 확인" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Set-Location C:\Users\wmszz\source\repos\Forklift-Twin-Actuator
Write-Host "현재 위치: $(Get-Location)" -ForegroundColor Green
Write-Host ""

# 2. 전송할 파일 확인
Write-Host "================================" -ForegroundColor Cyan
Write-Host "2. 전송할 파일 목록" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host "- fta_beginner/ (초보자용 테스트 패키지)" -ForegroundColor Yellow
Write-Host "- fta_interfaces/ (메시지 정의)" -ForegroundColor Yellow
Write-Host "- BEGINNER_GUIDE.md" -ForegroundColor Yellow
Write-Host "- CURRENT_STATUS.md" -ForegroundColor Yellow
Write-Host ""

# 3. SCP로 파일 전송
Write-Host "================================" -ForegroundColor Cyan
Write-Host "3. SSH로 파일 전송 시작" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host "원격 서버: 172.30.1.102" -ForegroundColor Green
Write-Host "사용자: wmszz" -ForegroundColor Green
Write-Host ""
Write-Host "비밀번호를 입력하세요..." -ForegroundColor Yellow

# 디렉토리가 없으면 생성
ssh wmszz@172.30.1.102 "mkdir -p ~/Forklift-Twin-Actuator"

# 파일 전송
Write-Host "파일 전송 중..." -ForegroundColor Yellow
scp -r fta_beginner fta_interfaces BEGINNER_GUIDE.md CURRENT_STATUS.md wmszz@172.30.1.102:~/Forklift-Twin-Actuator/

Write-Host ""
Write-Host "================================" -ForegroundColor Green
Write-Host "✅ 파일 전송 완료!" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green
Write-Host ""

# 4. 다음 단계 안내
Write-Host "================================" -ForegroundColor Cyan
Write-Host "다음 단계: SSH 접속 및 빌드" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host "다음 명령어를 실행하세요:" -ForegroundColor Yellow
Write-Host ""
Write-Host "  ssh wmszz@172.30.1.102" -ForegroundColor White
Write-Host "  cd ~/Forklift-Twin-Actuator" -ForegroundColor White
Write-Host "  source /opt/ros/jazzy/setup.bash" -ForegroundColor White
Write-Host "  colcon build --packages-select fta_interfaces fta_beginner" -ForegroundColor White
Write-Host "  source install/setup.bash" -ForegroundColor White
Write-Host ""
