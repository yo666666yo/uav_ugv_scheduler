#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/shared/group4_ws/install/setup.bash
export ZRDDS_HOME=/usr/ZRDDS/ZRDDS-2.4.4
export LD_LIBRARY_PATH=/usr/ZRDDS/ZRDDS-2.4.4/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_zrdds_dynamic_cpp
export RMW_ZRDDS_XML_PATH=/home/ubuntu/snowman/ZRDDS/ZRDDS-2.4.4/ZRDDS_QOS_PROFILES.xml

echo "=== arm/gripper/kinematics nodes ==="
ros2 node list 2>/dev/null | grep -iE "arm|gripper|kinem|servo|pick|grasp"
echo "=== arm-related services ==="
ros2 service list 2>/dev/null | grep -iE "arm|gripper|ik|kinem|pick|servo" | head -20
echo "=== camera depth/image topics ==="
ros2 topic list 2>/dev/null | grep -iE "depth|image|point|ascamera|aurora|scan" | head -20
echo "=== grasp script exists ==="
ls -l /home/ubuntu/approach_grasp_c1.py 2>/dev/null || echo "NO_SCRIPT"
echo "=== done ==="
