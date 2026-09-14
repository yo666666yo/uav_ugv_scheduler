#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/shared/group4_ws/install/setup.bash
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
export RMW_ZRDDS_XML_PATH=/home/ubuntu/snowman/ZRDDS/ZRDDS-2.4.4/ZRDDS_QOS_PROFILES.xml
python3 /home/ubuntu/zrdds_sub_diag.py $1
