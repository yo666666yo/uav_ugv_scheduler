#!/bin/bash
# 安全启动：feed + 安全版桥 + watchdog（桥死自动重启）
# 用法: bash /home/pi/Group4SwarmScheduler/work_bridge_patch/safe_launch.sh
set -u
LOG_DIR=/tmp
BRIDGE_BIN=/tmp/fcu_bridge_001_safe
FEED_PY=/tmp/odom_feed_v2.py
DRONE_IP=192.168.1.126
ROS_SETUP="source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10"

# 1. 清理全部残留
docker exec fanciswarm_ros2 bash -lc "pkill -9 -f fcu_bridge; pkill -9 -f odom_feed; sleep 1"

# 2. 起 feed
docker exec -d fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=10 && python3 $FEED_PY > $LOG_DIR/feed_safe.log 2>&1"
echo "[$(date +%T)] feed started"

# 3. watchdog：桥死自动重启
BRIDGE_ARGS="--ros-args -p DRONE_IP:=$DRONE_IP -p channel:=1 -p offboard:=false -p use_uwb:=true -p set_goal:=false -p simple_target:=true"
RESTART_COUNT=0
while true; do
    ALIVE=$(docker exec fanciswarm_ros2 sh -c "ps aux | grep fcu_bridge_001_safe | grep -vE \"grep|defunct\" | wc -l" 2>/dev/null)
    if [ "${ALIVE:-0}" -lt 1 ]; then
        RESTART_COUNT=$((RESTART_COUNT+1))
        echo "[$(date +%T)] bridge DOWN -> restart #$RESTART_COUNT"
        docker exec -d fanciswarm_ros2 bash -lc "$ROS_SETUP && $BRIDGE_BIN $BRIDGE_ARGS > $LOG_DIR/fcu_bridge_safe.log 2>&1"
        sleep 5
    fi
    sleep 3
done
