"""Project-owned launch description; does not modify vendor launch files."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package="fcu_core", executable="fcu_bridge_001", name="fcu_bridge_001",
        parameters=[{"DRONE_IP":"192.168.1.126", "channel":1, "offboard":False, "use_uwb":True,
                     "set_goal":False, "simple_target":True}],
        remappings=[("command", "/fcu_command/command"), ("mission_001", "/fcu_mission/mission_001")])])
