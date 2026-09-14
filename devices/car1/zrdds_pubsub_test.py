import rclpy, time
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
n = Node('zrdds_pubsub_test')
got = []
def cb(m):
    got.append(m.data)
    print('SUB got:', m.data, flush=True)
n.create_subscription(String, '/zrdds_test', cb, 10)
pub = n.create_publisher(String, '/zrdds_test', 10)

# 等 rmw 建立连接
time.sleep(2)
ok = False
for i in range(6):
    msg = String()
    msg.data = 'hello-zrdds-%d' % i
    pub.publish(msg)
    t0 = time.time()
    while time.time() - t0 < 2 and not got:
        rclpy.spin_once(n, timeout_sec=0.2)
    if got:
        print('PUB-SUB OK, received %d msgs' % len(got), flush=True)
        ok = True
        break
if not ok:
    print('PUB-SUB FAILED, got 0 msgs', flush=True)
n.destroy_node()
rclpy.shutdown()
print('TEST_DONE', flush=True)
