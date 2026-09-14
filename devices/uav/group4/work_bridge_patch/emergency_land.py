#!/usr/bin/env python3
"""一键应急降落（树莓派宿主机运行）
用法: python3 /home/pi/Group4SwarmScheduler/work_bridge_patch/emergency_land.py
多路径自动切换，任何一步不生效自动走下一步：
  A. 桥活着 -> LAND(4) x3 -> 观察 6s
  B. 位置目标降高 z->0.03 持续 -> 降到 0.25m
  C. DISARM x5 -> 确认落地
  D. 桥死了 -> pymavlink 直连(心跳+位置流) + DISARM
"""
import subprocess, time, sys, os

CONTAINER = "fanciswarm_ros2"

def sh(cmd, timeout=30):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception as e:
        return f"ERR {e}"

def docker_exec(code):
    return sh(f"docker exec {CONTAINER} bash -lc \"{code}\"")

def bridge_alive():
    n = docker_exec("ps aux | grep fcu_bridge_001 | grep -vE \"grep|defunct\" | wc -l")
    return n.isdigit() and int(n) > 0

def last_z():
    """从桥日志取最近 z（桥打印 x:..y:..z:..），返回浮点（取负=高度）"""
    out = docker_exec("grep \\\"x:\\\" /tmp/fcu_bridge_safe.log 2>/dev/null | tail -1")
    try:
        import re
        m = re.search(r"z:(-?[0-9.]+)", out)
        if m:
            return -float(m.group(1))  # 桥取负，转回物理高度
    except Exception:
        pass
    return None

def send_cmd(data, wait=1.0):
    code = f"""source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=10 && python3 -c \"
import rclpy, time
from std_msgs.msg import Int16
rclpy.init()
n = rclpy.create_node(el)
pub = n.create_publisher(Int16, command, 10)
time.sleep(0.5)
pub.publish(Int16(data={data}))
time.sleep({wait})
\" """
    return docker_exec(code)

def send_land_target(x, y, z, duration=15):
    code = f"""source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=10 && python3 -c \"
import rclpy, time
from std_msgs.msg import Float32MultiArray
rclpy.init()
n = rclpy.create_node(el_desc)
pub = n.create_publisher(Float32MultiArray, mission_001, 10)
time.sleep(0.3)
for i in range(int({duration}/0.4)):
    m = Float32MultiArray()
    m.data = [0.0,0.0,{x},{y},{z},0.0,0.0,0.0,0.0,0.0,0.0]
    pub.publish(m)
    time.sleep(0.4)
\" """
    return docker_exec(code)

def watch_z(seconds, expect_down=True):
    """观察 z seconds 秒，返回 (最终z, 是否明显下降/到地)"""
    z0 = last_z()
    end = time.time() + seconds
    zmin = z0
    while time.time() < end:
        z = last_z()
        if z is not None:
            zmin = min(zmin, z)
        time.sleep(1.0)
    zf = last_z()
    print(f"    高度: {z0:.2f}m -> {zf:.2f}m (最低 {zmin:.2f}m)")
    landed = (zf is not None and zf < 0.15) or (zmin is not None and zmin < 0.15)
    falling = zf is not None and z0 is not None and zf < z0 - 0.1
    return zf, landed, falling

print("=" * 46)
print(" 一键应急降落  (无人机 001)")
print("=" * 46)

# ---- A. 桥活着 -> LAND ----
if bridge_alive():
    print("[A] 桥在线 -> 发 LAND(4) x3")
    send_cmd(4, wait=1.5)
    time.sleep(1)
    send_cmd(4, wait=1.5)
    time.sleep(1)
    send_cmd(4, wait=1.5)
    zf, landed, falling = watch_z(6)
    if landed:
        print("[OK] 已落地 (z<0.15m)")
        sys.exit(0)
    if falling:
        print("[..] 高度在下降，等待落地...")
        zf, landed, falling = watch_z(8)
        if landed:
            print("[OK] 已落地")
            sys.exit(0)
    print("[A] LAND 未生效，切换到降高")
else:
    print("[A] 桥不在线，直接走直连路径")

# ---- B. 位置目标降高 ----
print("[B] 位置目标降高 z->0.03 (15s)")
# 用当前位置作为 x,y
cur = sh(f"docker exec {CONTAINER} bash -lc \"grep \\\"x:\\\" /tmp/fcu_bridge_safe.log 2>/dev/null | tail -1\"")
import re
m = re.search(r"x:([0-9.]+),y:([0-9.]+)", cur) if cur else None
cx, cy = (float(m.group(1)), float(m.group(2))) if m else (0.77, 0.88)
send_land_target(cx, cy, 0.03, duration=15)
zf, landed, falling = watch_z(10)
if landed:
    print("[OK] 已落地")
    sys.exit(0)
if falling:
    print("[..] 持续下降，等待...")
    send_land_target(cx, cy, 0.03, duration=10)
    zf, landed, falling = watch_z(8)
    if landed:
        print("[OK] 已落地")
        sys.exit(0)

# ---- C. DISARM（低空兜底）----
print("[C] DISARM(2) x5 (低空兜底)")
if bridge_alive():
    for i in range(5):
        send_cmd(2, wait=0.6)
    zf, landed, falling = watch_z(5)
    if landed or (zf is not None and zf < 0.3):
        print("[OK] 已落地 (DISARM 生效)")
        sys.exit(0)

# ---- D. pymavlink 直连 ----
print("[D] 桥失效 -> pymavlink 直连迫降")
code = """python3 -c \"
import time
from pymavlink import mavutil
c = mavutil.mavlink_connection(tcp:192.168.1.126:333)
if c.wait_heartbeat(timeout=5):
    for i in range(3):
        c.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0,0,0)
        c.mav.local_position_ned_send(0, 0.77, -0.88, -0.2, 0,0,0, 0)
        time.sleep(1.0)
    c.mav.set_mode_send(1, 82)
    time.sleep(0.5)
    c.mav.command_long_send(1,1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0,0)
    print(直连 DISARM 已发送)
    time.sleep(2)
c.close()
\" """
print(sh(code, timeout=20))

print("[!] 若仍未落地：飞控失控保护会自动降落 (GCS失联)，请就近观察/人工接管")
