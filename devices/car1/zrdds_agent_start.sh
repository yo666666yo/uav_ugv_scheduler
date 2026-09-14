#!/bin/bash
# ZRDDS car_agent 启动脚本（由 group4_car_agent.service 调用）
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/shared/group4_ws/install/setup.bash
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
export RMW_ZRDDS_XML_PATH=/home/ubuntu/snowman/ZRDDS/ZRDDS-2.4.4/ZRDDS_QOS_PROFILES.xml
exec /home/ubuntu/shared/group4_ws/install/group4_car_agent/lib/group4_car_agent/car_agent \
  --ros-args \
  --params-file /home/ubuntu/shared/group4_ws/install/group4_car_agent/share/group4_car_agent/config/car_001.yaml
