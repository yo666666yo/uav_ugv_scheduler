import rclpy, time, sys
from rclpy.node import Node

which = sys.argv[1] if len(sys.argv) > 1 else 'battery'
rclpy.init()
n = Node('zrdds_diag2')
got = []
if which == 'battery':
    from std_msgs.msg import UInt16
    n.create_subscription(UInt16, '/ros_robot_controller/battery', lambda m: got.append(m.data), 10)
    topic = '/ros_robot_controller/battery'
elif which == 'odom':
    from nav_msgs.msg import Odometry
    n.create_subscription(Odometry, '/odom', lambda m: got.append(m.header.stamp.sec), 10)
    topic = '/odom'
else:
    from sensor_msgs.msg import LaserScan
    n.create_subscription(LaserScan, '/scan', lambda m: got.append(len(m.ranges)), 10)
    topic = '/scan'
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.3)
    if got:
        break
print('GOT %d msgs from %s: %s' % (len(got), topic, got[:3]), flush=True)
n.destroy_node()
rclpy.shutdown()
print('DONE', flush=True)
