import rclpy, sys, traceback
from rclpy.node import Node

print('T0 import rclpy OK', flush=True)
import rclpy
from group4_interfaces.msg import Battery, TaskAssignment, AgentState

which = sys.argv[1] if len(sys.argv) > 1 else 'battery'

def test_battery():
    rclpy.init()
    n = Node('zrdds_t_batt')
    p = n.create_publisher(Battery, '/zrdds_t_batt', 10)
    msg = Battery()
    msg.raw_value = 7000
    msg.voltage_v = 7.0
    msg.remaining_percent = 80.0
    msg.battery_ok = True
    for i in range(5):
        p.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.2)
    print('TEST battery OK', flush=True)
    n.destroy_node(); rclpy.shutdown()

def test_task():
    rclpy.init()
    n = Node('zrdds_t_task')
    p = n.create_publisher(TaskAssignment, '/zrdds_t_task', 10)
    msg = TaskAssignment()
    msg.schema_version = '1.0'
    msg.task_id = 't1'
    msg.target_pose.frame_id = 'map'
    msg.target_pose.x_m = 1.0
    for i in range(5):
        p.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.2)
    print('TEST task(嵌套Pose) OK', flush=True)
    n.destroy_node(); rclpy.shutdown()

def test_agent():
    rclpy.init()
    n = Node('zrdds_t_agent')
    p = n.create_publisher(AgentState, '/zrdds_t_agent', 10)
    msg = AgentState()
    msg.schema_version = '1.0'
    msg.agent_id = 'car_001'
    msg.capabilities = ['nav', 'grasp']
    msg.pose.frame_id = 'map'
    msg.battery.voltage_v = 7.0
    for i in range(5):
        p.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.2)
    print('TEST agent(嵌套+string[]) OK', flush=True)
    n.destroy_node(); rclpy.shutdown()

try:
    {'battery': test_battery, 'task': test_task, 'agent': test_agent}[which]()
    print('RESULT OK', flush=True)
except Exception as e:
    print('RESULT EXC:', repr(e), flush=True)
    traceback.print_exc()
