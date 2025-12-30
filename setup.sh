#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

echo "============================================"
echo "Last 100m Delivery Robot - Setup"
echo "============================================"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 외부 의존성 가져오기
echo -e "\n${YELLOW}[1/4] Fetching external dependencies...${NC}"
cd "$PROJECT_ROOT"

if ! command -v vcs &> /dev/null; then
    echo -e "${RED}Error: vcs not found. Install with: pip install vcstool${NC}"
    exit 1
fi

vcs import < dependencies.repos

if [ -d "$PROJECT_ROOT/external/tiago_isaac" ]; then
    echo -e "${GREEN}✓ tiago_isaac cloned successfully${NC}"
else
    echo -e "${RED}✗ Failed to clone tiago_isaac${NC}"
    exit 1
fi

# 2. ros2_ws/src 디렉토리 생성
echo -e "\n${YELLOW}[2/4] Setting up ROS2 workspace...${NC}"
mkdir -p "$PROJECT_ROOT/ros2_ws/src"

# 3. 심볼릭 링크 생성
echo -e "\n${YELLOW}[3/4] Creating symbolic links...${NC}"

# tiago_rviz2 패키지 링크
TIAGO_RVIZ2_SRC="$PROJECT_ROOT/external/tiago_isaac/ros2_ws/src/tiago_rviz2"
TIAGO_RVIZ2_LINK="$PROJECT_ROOT/ros2_ws/src/tiago_rviz2"

if [ -d "$TIAGO_RVIZ2_SRC" ]; then
    if [ -L "$TIAGO_RVIZ2_LINK" ]; then
        rm "$TIAGO_RVIZ2_LINK"
    fi
    ln -sf "$TIAGO_RVIZ2_SRC" "$TIAGO_RVIZ2_LINK"
    echo -e "${GREEN}✓ tiago_rviz2 linked${NC}"
else
    echo -e "${YELLOW}⚠ tiago_rviz2 not found in tiago_isaac (may be optional)${NC}"
fi

# 4. 안내 메시지
echo -e "\n${YELLOW}[4/4] Setup complete!${NC}"
echo ""
echo "============================================"
echo "Next steps:"
echo "============================================"
echo ""
echo "1. Isaac Sim extension 경로 설정:"
echo "   $PROJECT_ROOT/external/tiago_isaac/exts/"
echo ""
echo "2. ROS2 워크스페이스 빌드:"
echo "   cd $PROJECT_ROOT/ros2_ws"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "3. TIAGo USD 파일 위치:"
echo "   $PROJECT_ROOT/external/tiago_isaac/tiago_dual_functional.usd"
echo "   $PROJECT_ROOT/external/tiago_isaac/tiago_dual_functional_light.usd"
echo ""
echo -e "${GREEN}Setup finished successfully!${NC}"
