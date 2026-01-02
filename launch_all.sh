#!/bin/bash
# Tiago Delivery System Launcher

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}🚀 Starting Tiago Delivery System${NC}"
echo "=================================="

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS 2 not sourced! Please run: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

# Check if workspace is built
if [ ! -d "$HOME/tiago-delivery/ros2_ws/install" ]; then
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    cd ~/tiago-delivery/ros2_ws
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Build failed!${NC}"
        exit 1
    fi
fi

# Source workspace
source ~/tiago-delivery/ros2_ws/install/setup.bash

echo -e "${GREEN}✅ Workspace sourced${NC}"
echo ""
echo -e "${YELLOW}📦 Launching Perception nodes...${NC}"
ros2 launch receiver_detection perception.launch.py &
PERCEPTION_PID=$!
sleep 2

echo -e "${YELLOW}🤖 Launching Manipulation node...${NC}"
ros2 launch manipulation manipulation.launch.py &
MANIPULATION_PID=$!
sleep 2

echo ""
echo -e "${GREEN}✅ All systems running!${NC}"
echo "=================================="
echo -e "${YELLOW}To test the system, run:${NC}"
echo "  ros2 action send_goal --feedback /approach_box interfaces/action/ApproachBox \\"
echo "  '{stop_distance: 0.5, timeout_sec: 20.0, min_confidence: 0.0, align_first: true}'"
echo ""
echo -e "${RED}Press Ctrl+C to stop all nodes${NC}"

# Wait for Ctrl+C
trap "echo -e '\n${RED}Stopping all nodes...${NC}'; kill $PERCEPTION_PID $MANIPULATION_PID 2>/dev/null; exit" INT
wait
