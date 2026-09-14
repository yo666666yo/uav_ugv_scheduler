import rclpy, time
from rclpy.node import Node

rclpy.init()
n = Node('grasp_env_diag')
log = []

# 1) 相机话题是否有数据
from sensor_msgs.msg import Image, CameraInfo
got_rgb, got_depth, got_info = [], [], []
n.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', lambda m: got_rgb.append(m.height), 1)
n.create_subscription(Image, '/ascamera/camera_publisher/depth0/image_raw', lambda m: got_depth.append(m.height), 1)
n.create_subscription(CameraInfo, '/ascamera/camera_publisher/depth0/camera_info', lambda m: got_info.append(m.width), 1)

# 2) IK/FK 服务探测（kinematics）
from kinematics_msgs.srv import SetJointValue  # 尝试常见接口
srv_names = ['/kinematics/set_pose_target', '/set_pose_target', '/kinematics_node/set_pose_target',
             '/kinematics/fk', '/fk', '/kinematics_node/fk']
for s in srv_names:
    try:
        if n.wait_for_service(s, timeout=1.0):
            log.append('SERVICE_OK ' + s)
        else:
            log.append('SERVICE_NO ' + s)
    except Exception as ex:
        log.append('SERVICE_ERR %s %s' % (s, ex))

t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.3)
    if got_rgb and got_depth and got_info:
        break
log.append('RGB %d DEPTH %d INFO %d' % (len(got_rgb), len(got_depth), len(got_info)))
print('\n'.join(log), flush=True)
n.destroy_node()
rclpy.shutdown()
print('DONE', flush=True)
