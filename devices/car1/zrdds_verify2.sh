#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/shared/group4_ws/install/setup.bash
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
export RMW_ZRDDS_XML_PATH=/home/ubuntu/snowman/ZRDDS/ZRDDS-2.4.4/ZRDDS_QOS_PROFILES.xml

echo "=== node list (zrdds, same xml) ==="
ros2 node list 2>/dev/null | grep -E "group4|agent"
echo "=== node info /group4_car_agent ==="
ros2 node info /group4_car_agent 2>&1 | sed -n '/Subscribers/,/Service/p' | head -24
echo "=== topic echo /group4/agent/state ==="
timeout 10 ros2 topic echo /group4/agent/state --once 2>&1 | grep -v -E "WARNING|Could not" | head -16
echo "=== done ==="
