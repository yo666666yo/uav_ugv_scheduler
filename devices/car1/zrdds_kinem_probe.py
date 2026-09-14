import rclpy, time
from rclpy.node import Node

rclpy.init()
n = Node('kinem_probe')
out = []
from kinematics_msgs.srv import SetRobotPose, SetJointValue
for name, st in [('/kinematics/set_pose_target', SetRobotPose),
                 ('/kinematics/set_joint_value_target', SetJointValue)]:
    try:
        cli = n.create_client(st, name)
        ok = cli.wait_for_service(timeout_sec=2.0)
        out.append('%s -> %s' % (name, 'AVAILABLE' if ok else 'MISSING'))
        cli.destroy()
    except Exception as ex:
        out.append('%s -> ERR %s' % (name, ex))
print('\n'.join(out), flush=True)
n.destroy_node()
rclpy.shutdown()
print('DONE', flush=True)
