#!/bin/bash
# ZRDDS 环境 + 测试
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/shared/group4_ws/install/setup.bash
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
echo "=== LD_LIBRARY_PATH has humble: $(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -c /opt/ros/humble/lib) ==="
MODE=$1
case $MODE in
  agent)
    timeout 25 python3 /home/ubuntu/zrdds_type_test.py agent 2>&1 | grep -E "T0|TEST|RESULT|Segmentation|Traceback|fault"
    echo "AGENT_TYPE_EXIT=$?"
    ;;
  car)
    timeout 20 /home/ubuntu/shared/group4_ws/install/group4_car_agent/lib/group4_car_agent/car_agent --ros-args --params-file /home/ubuntu/shared/group4_ws/install/group4_car_agent/share/group4_car_agent/config/car_001.yaml 2>&1 | tail -15
    echo "CAR_EXIT=$?"
    ;;
  carlog)
    timeout 25 /home/ubuntu/shared/group4_ws/install/group4_car_agent/lib/group4_car_agent/car_agent --ros-args --params-file /home/ubuntu/shared/group4_ws/install/group4_car_agent/share/group4_car_agent/config/car_001.yaml 2>&1 | grep -vE "LOGINFO|ZRSpdp|\*\*\*|^local|^$" | tail -20
    echo "CAR_EXIT=$?"
    ;;
esac
