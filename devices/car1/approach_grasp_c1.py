#!/usr/bin/env python3
# encoding: utf-8
# 1号车(LanderPi 192.168.1.138) 一体化"行进-发现-抓取"：官方cmd_vel驱动前进 → 视觉识别黄色 → 深度到位停车 → 释放串口 → Board SDK 机械臂抓取
# 两阶段：
#   Phase1 DRIVE : 订阅相机 / 发布 /controller/cmd_vel(linear.x>0前进, angular.z转向微调) / 深度<0.28 停车
#   Phase2 GRASP : kill ros_robot_controller 释放串口 → Board('/dev/rrc') 直驱 → 云台PID跟踪 → IK逆解 → 闭爪540
# 前提：bringup 在跑（含 ros_robot_controller / odom_publisher / aurora 相机）；kinematics 节点已单独拉起
import cv2
import math
import time
import queue
import sys
import os
import threading

sys.stdout = open(os.devnull, 'w')

import numpy as np
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from kinematics.kinematics_control import set_pose_target

sys.path.insert(0, '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/')
from ros_robot_controller_sdk import Board

hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.037],
    [0.0, 0.0, 0.0, 1.0],
]

YELLOW_MIN = np.array([90, 100, 132], dtype=np.uint8)
YELLOW_MAX = np.array([255, 255, 255], dtype=np.uint8)
MIN_AREA = 150  # 1m 外方块仅约150-300px²；须够低才能远处检测到（墙/路标靠后续过滤挡）

GRAB_DIST_MIN = 0.10
GRAB_DIST_MAX = 0.30
STABLE_SECS = 2.0
TRACK_STEP = 3

GRIP_CLOSE = 540   # 1号车完全夹紧
INIT_SERVOS = [[1, 500], [2, 700], [3, 151], [4, 70], [5, 500], [10, 150]]
HOLD_SERVOS = [[1, 500], [2, 650], [3, 130], [4, 120], [5, 500]]

# ---- 驱动阶段参数 ----
FORWARD_SPEED = 0.15      # m/s
STEER_KP = 0.8            # angular.z = -STEER_KP * (cx/w - 0.5)
APPROACH_STOP = 0.25   # 1号车：0.21m(cy=354)/0.215m(cy~350)时抓取检测失灵锁远处；0.25m(cy~290)可靠且IK可达(x~0.19, 0.245m参考成功)   # 1号车方块在~0.21m(cy=354)即开始丢失跳到墙纹；0.23m(cy~315)仍可靠检出，且IK可达(x~0.17)      # 深度小于此值且居中才停车：须把方块带到机械臂可达区（0.19-0.21m 时 IK≈x0.20 可达；0.28m 时 x0.30 不可达）
MAX_DRIVE_TIME = 25.0     # 秒，超时停车
DIST_NOT_SEEN_STOP = 2.5  # 接近区已见目标后若连续丢失超过此秒数→停车


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


def xyz_quat_to_mat(xyz, quat):
    w, x, y, z = quat
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def translate_mat(xyz):
    T = np.eye(4)
    T[:3, 3] = xyz
    return T


def kill_ros_robot_controller():
    """按 cmdline 找到并 kill ros_robot_controller 节点，释放 /dev/rrc"""
    killed = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                cmd = f.read().decode(errors='replace')
            if '/lib/ros_robot_controller/ros_robot_controller' in cmd:
                os.kill(int(pid), 9)
                killed.append(pid)
        except Exception:
            pass
    return killed


class PID:
    def __init__(self, kp=20.5, ki=1.0, kd=1.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.SetPoint = 0.0
        self.output = 0.0
        self._integral = 0.0
        self._last_error = 0.0

    def update(self, pv):
        error = self.SetPoint - pv
        self._integral += error
        self.output = self.kp * error + self.ki * self._integral + self.kd * (error - self._last_error)
        self._last_error = error

    def clear(self):
        self._integral = 0.0
        self._last_error = 0.0
        self.output = 0.0


class ApproachGraspNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.image_queue = queue.Queue(maxsize=2)
        rgb_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image')
        depth_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/ascamera/camera_publisher/depth0/camera_info')
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 5, 0.05)
        sync.registerCallback(self.multi_callback)

        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.target_mem = None  # (cx, cy, depth, timestamp) 帧级抗抖
        self.get_logger().info('approach-grasp node ready')

    def multi_callback(self, rgb, depth, info):
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put((rgb, depth, info))

    def get_frame(self, timeout=1.0):
        try:
            return self.image_queue.get(block=True, timeout=timeout)
        except queue.Empty:
            return None

    def depth_raised_mask(self, bgr, depth_image):
        """深度凸起掩码：黄色 且 比局部平滑背景近>=RAISE_THR 的像素。
        物理依据：方块是凸起的3D物体，正面比其背景(墙/地面)更近；平面路标/墙贴纸深度与背景相同被剔除。"""
        depth16 = depth_image.astype(np.float32)
        depth = depth16.copy()
        depth[depth <= 0] = np.nan
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        ymask = cv2.inRange(lab, YELLOW_MIN, YELLOW_MAX)
        ymask = cv2.morphologyEx(ymask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        depth_f = depth.copy()
        depth_f[np.isnan(depth_f)] = 0.0
        bg = cv2.blur(depth_f, (9, 9))            # 平滑背景（9x9，方块更完整）
        diff = bg - depth                          # 正值=比背景近
        raised = np.zeros_like(ymask)
        raised[(ymask > 0) & (np.nan_to_num(diff, nan=0.0) > 0.008) & (depth > 0)] = 255
        return raised

    def detect_yellow(self, bgr, depth_image=None):
        h, w = bgr.shape[:2]
        # 方法1：深度凸起检测（方块）。平面黄路标/墙面贴纸深度与背景一致→被剔除
        if depth_image is not None:
            raised = self.depth_raised_mask(bgr, depth_image)
            raised = cv2.morphologyEx(raised, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
            cnts_r, _ = cv2.findContours(raised, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cands = []
            for cc in cnts_r:
                area = cv2.contourArea(cc)
                if area < 80:
                    continue
                x, y, ww, hh = cv2.boundingRect(cc)
                if ww <= 0 or hh <= 0:
                    continue
                if max(ww, hh) / min(ww, hh) > 3.0:
                    continue
                cx0, cy0 = int(x + ww / 2), int(y + hh / 2)
                d = self.depth_probe(depth_image, cx0, cy0)
                if d is None or not (0.08 < d < 1.5):
                    continue
                phys_w = ww * d / 422.686
                phys_h = hh * d / 417.6
                if not (0.015 < min(phys_w, phys_h) and max(phys_w, phys_h) < 0.11):
                    continue
                cands.append(cc)
            if cands:
                # 优先最近深度（方块比远处墙/背景近），深度相近(同0.05m桶)时选离中轴最近（盲道砖恒偏右）
                def _pick(cc):
                    x, y, ww, hh = cv2.boundingRect(cc)
                    cx0 = int(x + ww / 2)
                    cy0 = int(y + hh / 2)
                    d = self.depth_probe(depth_image, cx0, cy0)
                    if d is None:
                        d = 1.5
                    return (int(d / 0.05), abs(cx0 / w - 0.5))
                best = min(cands, key=_pick)
                (cx, cy), r = cv2.minEnclosingCircle(best)
                return (int(cx), int(cy)), r
        # 方法2（兜底）：颜色掩码 + 长宽比 + 物理尺寸过滤，选离画面中轴最近
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab, YELLOW_MIN, YELLOW_MAX)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for cc in cnts:
            area = cv2.contourArea(cc)
            if area < MIN_AREA:
                continue
            x, y, ww, hh = cv2.boundingRect(cc)
            if ww <= 0 or hh <= 0:
                continue
            if max(ww, hh) / min(ww, hh) > 3.0:
                continue
            if depth_image is not None:
                cx0, cy0 = int(x + ww / 2), int(y + hh / 2)
                d = self.depth_probe(depth_image, cx0, cy0)
                if d is None or not (0.08 < d < 1.5):
                    continue
                phys_w = ww * d / 422.686
                phys_h = hh * d / 417.6
                if not (0.015 < min(phys_w, phys_h) and max(phys_w, phys_h) < 0.11):
                    continue
            _d = d if d is not None else 1.5
            _key = (int(_d / 0.05), abs((x + ww / 2) / w - 0.5))
            if best is None or _key < best[0]:
                best = (_key, area, cc)
        if best is None:
            return None, None
        _, area, cc = best
        (cx, cy), r = cv2.minEnclosingCircle(cc)
        return (int(cx), int(cy)), r

    def depth_at(self, depth_image, cx, cy):
        h, w = depth_image.shape[:2]
        r = 8
        x0, x1 = max(0, cx - r), min(w, cx + r)
        y0, y1 = max(0, cy - r), min(h, cy + r)
        win = depth_image[y0:y1, x0:x1]
        valid = win[np.logical_and(win > 0, win < 10000)]
        if valid.size < 3:
            return None
        return float(np.median(valid) / 1000.0)

    def depth_probe(self, depth_image, cx, cy):
        """中心+四角多点探测，返回有效中值（抗深度空洞）"""
        vals = []
        for dx, dy in [(0, 0), (-8, 0), (8, 0), (0, -8), (0, 8)]:
            d = self.depth_at(depth_image, cx + dx, cy + dy)
            if d is not None:
                vals.append(d)
        if not vals:
            return None
        return float(np.median(vals))

    def publish_cmdvel(self, lx, az):
        m = Twist()
        m.linear.x = lx
        m.angular.z = az
        self.cmd_vel_pub.publish(m)

    def drive_phase(self):
        """前进 + 识别黄色 + 分级减速转向对准；深度到位且居中后停车。返回 True=已就位"""
        self.get_logger().info('[DRIVE] start, cruise %.2f m/s' % FORWARD_SPEED)
        start = time.time()
        seen_in_range = False
        lost_since = None
        dist = None
        last_track = None        # (cx, cy, dist, t) 跳变拒绝：目标位置突变过大则保留上一帧
        stuck_hist = []          # (time, dist) 深度停滞检测
        min_dist_seen = 1e9
        while rclpy.ok() and time.time() - start < MAX_DRIVE_TIME:
            fr = self.get_frame(timeout=0.4)
            if fr is None:
                continue
            ros_rgb, ros_depth, cam_info = fr
            rgb_image = np.ndarray(shape=(ros_rgb.height, ros_rgb.width, 3), dtype=np.uint8, buffer=ros_rgb.data)
            depth_image = np.ndarray(shape=(ros_depth.height, ros_depth.width), dtype=np.uint16, buffer=ros_depth.data)
            h, w = rgb_image.shape[:2]

            center, r = self.detect_yellow(rgb_image, depth_image)
            got_depth = None
            if center is None:
                # 用目标记忆防抖（帧级深度空洞/检测抖动）
                if self.target_mem is not None and time.time() - self.target_mem[3] < 0.5:
                    cx, cy, dist, _ = self.target_mem
                    center = (cx, cy)
                    got_depth = dist
                else:
                    center = None
                    got_depth = None
            if center is None:
                # 未看到目标
                if seen_in_range:
                    # 接近区丢失：方块可能已贴近/移出视野，绝不能继续巡航直行（会撞上）
                    if lost_since is None:
                        lost_since = time.time()
                        self.publish_cmdvel(0.0, 0.0)
                    if time.time() - lost_since > 2.0:
                        self.get_logger().warn('[DRIVE] target lost in range, stop (avoid collision)')
                        break
                    continue
                # 远距离未看到：巡航直行寻找
                self.publish_cmdvel(FORWARD_SPEED, 0.0)
                continue
            cx, cy = center
            lost_since = None
            dist = got_depth if got_depth is not None else self.depth_probe(depth_image, cx, cy)
            # 跳变拒绝：新目标与上一帧位置/深度差过大 -> 误锁，保留上一帧目标
            if dist is not None and last_track is not None:
                lcx, lcy, ld, _ = last_track
                jump_px = abs(cx - lcx) + abs(cy - lcy)
                if jump_px > 200 or (ld is not None and abs(dist - ld) > 0.25):
                    self.get_logger().info('[DRIVE] jump rejected (dx=%d px, d=%.3f->%.3f), keep last' % (jump_px, ld if ld is not None else 0, dist))
                    cx, cy = lcx, lcy
                    dist = ld
                else:
                    last_track = (cx, cy, dist, time.time())
            elif dist is not None:
                last_track = (cx, cy, dist, time.time())
            if dist is not None:
                self.target_mem = (cx, cy, dist, time.time())
            offset = cx / w - 0.5
            az = max(-0.5, min(0.5, -STEER_KP * offset))  # 转向量限幅
            # 已到位：要求偏移足够小才停车；否则原地转向对准
            if dist is not None and dist < APPROACH_STOP:
                if abs(offset) > 0.12:
                    self.publish_cmdvel(0.0, az)
                    self.get_logger().info('[DRIVE] close, aligning off=%.3f' % offset)
                    continue
                self.get_logger().info('[DRIVE] in range & centered, STOP depth=%.3f cx=%d' % (dist, cx))
                break
            # 分级减速：>0.6 巡航；0.4~0.6 中速；<0.4 慢速并要求居中
            if dist is None or dist > 0.6:
                lx = FORWARD_SPEED
            elif dist > 0.40:
                lx = 0.10
            else:
                lx = 0.04 if abs(offset) <= 0.25 else 0.02
            if dist is not None and dist < 0.6:
                seen_in_range = True
            if dist is not None:
                self.get_logger().info('[DRIVE] cx=%d cy=%d depth=%.3f steer=%.3f lx=%.2f' % (cx, cy, dist, az, lx))
                # 深度停滞检测：行驶中目标深度长时间不下降 -> 误锁远处目标，停车避免开走
                if lx > 0 and dist > APPROACH_STOP and dist > 0.35:
                    # 仅远距离(>0.35m)启用停滞检测；近距离慢速逼近属正常，不误触发
                    now = time.time()
                    stuck_hist.append((now, dist))
                    if dist < min_dist_seen:
                        min_dist_seen = dist
                    while stuck_hist and now - stuck_hist[0][0] > 8.0:
                        stuck_hist.pop(0)
                    if len(stuck_hist) >= 6:
                        t0, d0 = stuck_hist[0]
                        # 8 秒内深度未下降至少 2cm -> 判定停滞
                        if dist >= d0 - 0.02 and dist > min_dist_seen + 0.03 and now - t0 > 7.0:
                            self.get_logger().warn('[DRIVE] depth stuck %.3f>%.3f, false-lock stop' % (d0, dist))
                            break
            self.publish_cmdvel(lx, az)
            time.sleep(0.03)
        self.publish_cmdvel(0.0, 0.0)
        time.sleep(0.3)
        reached = dist is not None and dist < APPROACH_STOP
        self.get_logger().info('[DRIVE] ended reached=%s' % reached)
        if not reached:
            try:
                fr = self.get_frame(timeout=0.5)
                if fr is not None:
                    rr, rd, _ = fr
                    cv2.imwrite('/tmp/drive_fail_rgb.jpg', np.ndarray(shape=(rr.height, rr.width, 3), dtype=np.uint8, buffer=rr.data))
                    cv2.imwrite('/tmp/drive_fail_depth.png', np.ndarray(shape=(rd.height, rd.width), dtype=np.uint16, buffer=rd.data))
            except Exception:
                pass
        return reached
    def grasp_phase(self):
        """释放串口 + Board SDK 直驱机械臂抓取（含云台跟踪/深度/IK/闭爪540）"""
        self.get_logger().info('[GRASP] killing ros_robot_controller to free serial')
        killed = kill_ros_robot_controller()
        self.get_logger().info('[GRASP] killed pids=%s' % killed)
        time.sleep(1.5)

        self.board = Board('/dev/rrc')
        self.board.enable_reception()

        self.set_pose_target_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.set_pose_target_client.wait_for_service()
        self.fk_client = self.create_client(SetJointValue, '/kinematics/set_joint_value_target')
        self.fk_client.wait_for_service()

        self.pid_yaw = PID(20.5, 1.0, 1.2)
        self.pid_pitch = PID(20.5, 1.0, 1.2)
        self.yaw = 500
        self.pitch = 70
        self.last_pitch_yaw = (0, 0)
        self.stamp = time.time()
        self.moving = False
        self.done = False

        self.set_servos(1.0, INIT_SERVOS)
        time.sleep(1.5)
        self.get_logger().info('[GRASP] arm init done, start tracking')
        self.grasp_main_loop()

    def set_servos(self, duration, positions):
        self.board.bus_servo_set_position(duration, [[i, int(p)] for i, p in positions])

    def read_servos(self, ids):
        vals = {}
        for sid in ids:
            v = None
            for _ in range(4):
                try:
                    v = self.board.bus_servo_read_position(sid)[0]
                    if v is not None:
                        break
                except Exception:
                    v = None
                time.sleep(0.03)
            vals[sid] = v
        return vals

    def send_request(self, client, msg, timeout=8.0):
        future = client.call_async(msg)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            if future.done():
                return future.result()
            time.sleep(0.01)
        return None

    def get_endpoint(self):
        servos = self.read_servos([1, 2, 3, 4, 5])
        if any(servos.get(i) is None for i in [1, 2, 3, 4, 5]):
            self.get_logger().error('servo read failed, skip endpoint')
            return None
        joint_value = [float(servos.get(i, 500)) for i in [1, 2, 3, 4, 5]]
        req = SetJointValue.Request()
        req.joint_value = joint_value
        res = self.send_request(self.fk_client, req)
        if res is None:
            self.get_logger().error('FK no response')
            return None
        p = res.pose.position
        o = res.pose.orientation
        return xyz_quat_to_mat([p.x, p.y, p.z], [o.w, o.x, o.y, o.z])

    def pick(self, position):
        yaw = 80 if position[2] < 0.2 else 30
        position[2] -= 0.013   # 降低抓取高度：2车成功z=0.017，1车0.030偏高->夹上端一抬就掉
        self.get_logger().info('[GRASP] IK position=(%.3f, %.3f, %.3f) yaw=%d' % (position[0], position[1], position[2], yaw))
        msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res is None or not res.pulse:
            self.get_logger().error('[GRASP] IK no solution, skip grab')
            self.moving = False
            return
        servo_data = res.pulse
        self.set_servos(1.0, [[1, servo_data[0]]])
        time.sleep(1.0)
        self.set_servos(1.5, [[1, servo_data[0]], [2, servo_data[1]], [3, servo_data[2]], [4, servo_data[3]], [5, servo_data[4]]])
        time.sleep(1.5)
        self.set_servos(0.5, [[10, GRIP_CLOSE]])
        time.sleep(1.0)
        self.get_logger().info('[GRASP] gripper closed (firm %d)' % GRIP_CLOSE)
        time.sleep(0.5)   # 闭合后停稳，让方块吃进夹爪
        self.set_servos(1.5, HOLD_SERVOS)
        time.sleep(1.5)
        self.get_logger().info('[GRASP] GRASP DONE, holding object')
        self.moving = False
        self.done = True

    def grasp_main_loop(self):
        _grasp_t0 = time.time()
        while rclpy.ok() and not self.done:
            if time.time() - _grasp_t0 > 45.0:
                self.get_logger().warn('[GRASP] grasp timeout 45s, abort')
                self.set_servos(1.5, HOLD_SERVOS)
                break
            fr = self.get_frame(timeout=1.0)
            if fr is None:
                continue
            ros_rgb, ros_depth, cam_info = fr
            rgb_image = np.ndarray(shape=(ros_rgb.height, ros_rgb.width, 3), dtype=np.uint8, buffer=ros_rgb.data)
            depth_image = np.ndarray(shape=(ros_depth.height, ros_depth.width), dtype=np.uint16, buffer=ros_depth.data)
            h, w = rgb_image.shape[:2]
            if self.moving:
                continue
            center, r = self.detect_yellow(rgb_image, depth_image)
            if center is None:
                if self.target_mem is not None and time.time() - self.target_mem[3] < 0.5:
                    center = self.target_mem[:2]
                else:
                    continue
            cx, cy = center
            cx_1 = cx / w
            if abs(cx_1 - 0.5) > 0.02:
                self.pid_yaw.SetPoint = 0.5
                self.pid_yaw.update(cx_1)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 0), 1000)
            else:
                self.pid_yaw.clear()
            cy_1 = cy / h
            if abs(cy_1 - 0.5) > 0.02:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(cy_1)
                self.pitch = min(max(self.pitch + self.pid_pitch.output, 50), 720)
            else:
                self.pid_pitch.clear()
            self.set_servos(0.02, [[1, int(self.yaw)], [4, int(self.pitch)]])
            if abs(self.last_pitch_yaw[0] - self.pitch) < TRACK_STEP and abs(self.last_pitch_yaw[1] - self.yaw) < TRACK_STEP:
                if time.time() - self.stamp > STABLE_SECS:
                    self.stamp = time.time()
                    self.moving = True
                    threading.Thread(target=self.do_grab, args=(depth_image, cam_info, (cx, cy))).start()
            else:
                self.stamp = time.time()
            self.last_pitch_yaw = (self.pitch, self.yaw)

    def do_grab(self, depth_image, cam_info, center):
        cx, cy = center
        dist = self.depth_probe(depth_image, cx, cy)
        if dist is not None:
            self.target_mem = (cx, cy, dist, time.time())
        if dist is None:
            self.get_logger().error('[GRASP] no valid depth')
            self.moving = False
            return
        self.get_logger().info('[GRASP] depth=%.3f m' % dist)
        if not (GRAB_DIST_MIN < dist < GRAB_DIST_MAX):
            self.get_logger().warn('[GRASP] out of range (%.3f), keep tracking' % dist)
            self.moving = False
            return
        dist += 0.015 + 0.015
        endpoint = self.get_endpoint()
        if endpoint is None:
            self.moving = False
            return
        K = cam_info.k
        position = depth_pixel_to_camera((cx, cy), dist, (K[0], K[4], K[2], K[5]))
        position[0] -= 0.01
        position[1] -= 0.02
        position[2] += 0.02   # 前向补偿 0.03->0.02（抓取偏前，收 1cm）
        pose_end = np.matmul(np.array(hand2cam_tf_matrix), translate_mat(position))
        world_pose = np.matmul(endpoint, pose_end)
        pose_t = world_pose[:3, 3]
        self.pick(pose_t)


def main():
    rclpy.init()
    node = ApproachGraspNode('approach_grasp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_t = threading.Thread(target=executor.spin, daemon=True)
    spin_t.start()
    time.sleep(1.0)
    reached = node.drive_phase()
    if reached:
        node.grasp_phase()
        while rclpy.ok() and not node.done:
            time.sleep(0.2)
    node.get_logger().info('APPROACH-GRASP FINISHED')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
