import rclpy, time
from rclpy.node import Node
from kinematics_msgs.srv import SetRobotPose
from kinematics.kinematics_control import set_pose_target

rclpy.init()
n = Node('ik_test')
cli = n.create_client(SetRobotPose, '/kinematics/set_pose_target')
if not cli.wait_for_service(timeout_sec=3.0):
    print('IK_SERVICE_MISSING')
    rclpy.shutdown()
    exit(1)
# 测试几个典型抓取位姿（脚本实际用的位置范围）
for pos, yaw in [([0.19, 0.0, 0.03], 80), ([0.22, 0.0, 0.02], 80), ([0.25, -0.01, 0.02], 30)]:
    msg = set_pose_target(list(pos), yaw, [-180.0, 180.0], 1.0)
    fut = cli.call_async(msg)
    res = None
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 8.0:
        rclpy.spin_once(n, timeout_sec=0.1)
        if fut.done():
            res = fut.result()
            break
    if res is None:
        print('IK pos=%s yaw=%d -> NO_RESPONSE' % (pos, yaw))
    else:
        pulse = list(res.pulse) if res.pulse else None
        print('IK pos=%s yaw=%d -> OK pulse=%s' % (pos, yaw, pulse))
cli.destroy()
n.destroy_node()
rclpy.shutdown()
print('DONE', flush=True)
