# ===========================================
# TIAGo Delivery Robot - Bash Aliases
# ===========================================
# Add this to your ~/.bashrc:
#   source /path/to/tiago-delivery/docker/aliases.sh
# ===========================================

# Isaac Sim 실행 (EC2 스트리밍용)
alias isaac-sim='
  export ROS_DISTRO=humble
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID=110
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/IsaacSim/exts/isaacsim.ros2.bridge/humble/lib
  export TIAGO_ISAAC_EXTS=/home/ubuntu/tiago-delivery/external/tiago_isaac/exts
  cd /opt/IsaacSim && ./isaac-sim.streaming.sh \
    --ext-folder $TIAGO_ISAAC_EXTS \
    --/app/livestream/publicEndpointAddress=$(curl -s ifconfig.me) \
    --/app/livestream/port=49100'

# ROS2 워크스페이스 컨테이너 실행
alias tiago-ws='docker run -it --rm \
  --network=host \
  --ipc=host \
  --pid=host \
  -v /home/ubuntu/tiago-delivery:/home/ros/tiago-delivery \
  tiago-delivery-ros2 \
  bash'

# 빌드 후 워크스페이스 컨테이너 실행 (처음 또는 Dockerfile 변경 시)
alias tiago-ws-build='
  cd /home/ubuntu/tiago-delivery && \
  docker build -t tiago-delivery-ros2 -f docker/Dockerfile.ros2-humble . && \
  docker run -it --rm \
    --network=host \
    --ipc=host \
    --pid=host \
    -v /home/ubuntu/tiago-delivery:/home/ros/tiago-delivery \
    tiago-delivery-ros2 \
    bash'
