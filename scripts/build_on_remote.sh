#!/bin/bash
# ========================================
# Ubuntu에서 실행: 빌드 및 테스트
# ========================================

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}========================================"
echo -e "FTA 프로젝트 빌드 스크립트"
echo -e "========================================${NC}"
echo ""

# 1. 프로젝트 디렉토리로 이동
echo -e "${CYAN}1. 프로젝트 디렉토리로 이동${NC}"
cd ~/Forklift-Twin-Actuator || { echo -e "${RED}❌ 디렉토리를 찾을 수 없습니다!${NC}"; exit 1; }
echo -e "${GREEN}✅ 현재 위치: $(pwd)${NC}"
echo ""

# 2. ROS2 환경 설정
echo -e "${CYAN}2. ROS2 환경 설정${NC}"
source /opt/ros/jazzy/setup.bash
echo -e "${GREEN}✅ ROS2 Jazzy 환경 로드 완료${NC}"
echo ""

# 3. 이전 빌드 삭제 (선택사항)
read -p "이전 빌드를 삭제하시겠습니까? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}빌드 캐시 삭제 중...${NC}"
    rm -rf build/ install/ log/
    echo -e "${GREEN}✅ 삭제 완료${NC}"
fi
echo ""

# 4. fta_interfaces 빌드
echo -e "${CYAN}3. fta_interfaces 패키지 빌드${NC}"
colcon build --packages-select fta_interfaces --event-handlers console_direct+
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ fta_interfaces 빌드 성공!${NC}"
else
    echo -e "${RED}❌ fta_interfaces 빌드 실패!${NC}"
    exit 1
fi
echo ""

# 5. 환경 적용
echo -e "${CYAN}4. 빌드 환경 적용${NC}"
source install/setup.bash
echo -e "${GREEN}✅ 환경 적용 완료${NC}"
echo ""

# 6. fta_beginner 빌드
echo -e "${CYAN}5. fta_beginner 패키지 빌드${NC}"
colcon build --packages-select fta_beginner --event-handlers console_direct+
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ fta_beginner 빌드 성공!${NC}"
else
    echo -e "${RED}❌ fta_beginner 빌드 실패!${NC}"
    exit 1
fi
echo ""

# 7. 다시 환경 적용
source install/setup.bash
echo -e "${GREEN}✅ 빌드 완료! 모든 패키지 사용 가능${NC}"
echo ""

# 8. 테스트 안내
echo -e "${CYAN}========================================"
echo -e "테스트 방법"
echo -e "========================================${NC}"
echo ""
echo -e "${YELLOW}[테스트 1] 간단한 통신 테스트${NC}"
echo -e "  터미널 1: ${WHITE}ros2 run fta_beginner simple_publisher${NC}"
echo -e "  터미널 2: ${WHITE}ros2 run fta_beginner simple_subscriber${NC}"
echo ""
echo -e "${YELLOW}[테스트 2] Patlite USB 장치 확인${NC}"
echo -e "  1. Patlite USB 연결"
echo -e "  2. ${WHITE}lsusb${NC} 명령어로 장치 확인"
echo -e "  3. ${WHITE}ros2 run fta_beginner patlite_test${NC}"
echo ""
echo -e "${YELLOW}[테스트 3] 명령 전송 테스트${NC}"
echo -e "  터미널 1: ${WHITE}ros2 run fta_beginner patlite_test${NC}"
echo -e "  터미널 2: ${WHITE}ros2 topic pub --once /patlite/command std_msgs/msg/String \"{data: 'LED_RED_ON'}\"${NC}"
echo ""
echo -e "${GREEN}준비 완료! 테스트를 시작하세요!${NC}"
