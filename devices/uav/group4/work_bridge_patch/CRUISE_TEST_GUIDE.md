# 无人机001 巡航测试操作手册 v2（无 feed 配方 · 2026-09-07 实测通过）
# 前提：无人机在沙盘内、桨叶装好、App 已断开、电池充满（关键！）、激光已初始化（拿起>1m 放下）
#
# ⚠️ 重大变更：不再需要 feed！
#   原因：feed（假VINS转圈：飞控位置→树莓派→重发布→灌回飞控）是 9/5-9/6 贴地猛冲/拒解锁的根源。
#   飞控板载 UWB+IMU 融合定位自洽：平滑、无漂移、坐标系=沙盘全局坐标（航点可直接用）。
#   详见文末「实测验证记录」。

# ---------- 步骤 0：起飞前检查 ----------
# 1. 电池充满：起飞电压应 ≥7.8V（实测满电 8.2V 巡航一圈 → 6.7V，接近 6.4V 告警线；电压不足会中途自动降落）
# 2. 激光初始化：启动提示音结束后，把无人机垂直拿起 >1m，停 2 秒，放下（不做则固件拒解锁，滴滴报警）
# 3. 无人机放沙盘内（放西南角 (0.6,1.0) 附近 = 巡航起点最省电；其他位置也能起，飞控会先飞向起点）
# 4. 树莓派/容器在线（docker ps 看到 fanciswarm_ros2 Up）

# ---------- 步骤 1：清理残留 ----------
docker exec fanciswarm_ros2 bash -lc "pkill -9 -f fcu_bridge; pkill -9 -f odom_feed; pkill -9 -f fcu_mission; sleep 1"

# ---------- 步骤 2：启动安全版桥（后台）【注意：不起 feed！】 ----------
docker exec -d fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && /tmp/fcu_bridge_001_safe --ros-args -p DRONE_IP:=192.168.1.126 -p channel:=1 -p offboard:=false -p use_uwb:=true -p set_goal:=false -p simple_target:=true > /tmp/fcu_bridge_user.log 2>&1"
sleep 6

# ---------- 步骤 3：确认桥状态（关键检查点） ----------
docker exec fanciswarm_ros2 sh -c "grep -E 'connect succeed' /tmp/fcu_bridge_user.log | tail -1; grep heartbeat /tmp/fcu_bridge_user.log | tail -1"
# 必须看到：connect succeed + heartbeat voltage≈8V（满电）
# 注意：无 feed 时桥日志只有心跳，不打印 x:（位置数据从 mission 的 [PATROL] 或订阅 odom_global_001 看）

# ---------- 步骤 4：启动 fcu_mission（航点节点）【注意：必须加 LD_LIBRARY_PATH！】 ----------
docker exec -d fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && export LD_LIBRARY_PATH=/home/snowman/fcu_core_onboard/install/quadrotor_msgs/lib:\$LD_LIBRARY_PATH && ros2 run fcu_core fcu_mission > /tmp/fcu_mission_user.log 2>&1"
sleep 4
docker exec fanciswarm_ros2 sh -c "tail -2 /tmp/fcu_mission_user.log"
# 必须看到：[PATROL] odom: x=... y=...（沙盘坐标，y 为正；位置≈无人机实际位置）
# 若报 "cannot open shared object file: libquadrotor_msgs" → LD_LIBRARY_PATH 没生效

# ---------- 步骤 5：写命令发送脚本（一次即可，之后复用） ----------
docker exec fanciswarm_ros2 sh -c "cat > /tmp/send_cmd.py << 'EOF'
import rclpy, sys, time
from std_msgs.msg import Int16
rclpy.init()
n = rclpy.create_node(\"cmd_sender\")
pub = n.create_publisher(Int16, \"command\", 10)
time.sleep(1.0)
pub.publish(Int16(data=int(sys.argv[1])))
print(\"sent cmd=\", sys.argv[1])
n.destroy_node()
rclpy.shutdown()
EOF"

# ---------- 步骤 6：解锁（ARM=1，SET_MODE 官方同款） ----------
docker exec fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && python3 /tmp/send_cmd.py 1"
# 检查点：飞控发出"解锁声音"（硬件解锁，绿灯）；若 3 秒内无反应，再发一次 ARM
# 注意：SET_MODE 解锁是"硬件解锁"，电机不一定转，电流仍 ~0.8A 属正常

# ---------- 步骤 7：起飞（TAKEOFF=3） ----------
docker exec fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && python3 /tmp/send_cmd.py 3"
sleep 6
# 检查点：电流升至 7-9A（电机全速）、mission 日志 z≈1.0（巡航高度）
# 确认悬停稳定（位置小幅摆动 <0.2m）后再巡航

# ---------- 步骤 8：启动巡航（CRUISE=0） ----------
docker exec fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && python3 /tmp/send_cmd.py 0"
# 预期：先飞向起点 (0.6,1.0)（若不在起点附近），然后逆时针巡航：西侧北上→东北角→东侧南下→回起点
# 关键检查点：前 5 秒方向应朝西南（x,y 减小）；若方向异常立即 LAND

# ---------- 步骤 9：监控巡航（位置+电压，每 5 秒一次） ----------
for i in $(seq 1 30); do docker exec fanciswarm_ros2 sh -c "tail -1 /tmp/fcu_mission_user.log"; docker exec fanciswarm_ros2 sh -c "grep heartbeat /tmp/fcu_bridge_user.log | tail -1"; sleep 5; done
# 电压监控：<6.8V 准备降落；<6.6V 立即 LAND（官方 6.4V 会自动降落）
# 完整一圈约 2.5 分钟；mission 完成时输出："巡航完成，回到起点，自动停止悬停"

# ---------- 步骤 10：降落（LAND=4） ----------
docker exec fanciswarm_ros2 bash -lc "source /opt/ros/humble/setup.bash && source /tmp/fcu_project_ws/install/setup.bash && export ROS_DOMAIN_ID=10 && python3 /tmp/send_cmd.py 4"
# 检查点：垂直下降、触地停桨（armed False、电流 <1A、电压回升）

# ================================================================
# ⚠️ 应急（优先级从高到低）：
# 1. 地面端喊"迫降" → 位置降高至 0.2m + 断电（应急脚本，自动逐级兜底）：
#    python3 /home/pi/Group4SwarmScheduler/work_bridge_patch/emergency_land.py
# 2. LAND 异常（贴地猛冲/失控）→ 直接杀桥断连，触发飞控官方"失联自动降落"：
#    docker exec fanciswarm_ros2 bash -lc "pkill -9 -f fcu_bridge"
# 3. 任何时刻：用户可用手机 App 接管（App 有最高优先级）
# ================================================================

# ================================================================
# 📌 2026-09-07 实测验证记录（方案 A · 无 feed · 全链路一次成功）
#
# 流程：解锁✅ → 起飞✅（z≈1.0m）→ 22航点巡航完整一圈✅ → 平稳降落✅
# 轨迹：东北 (3.3,1.1) 起飞 → 直飞西南起点 (0.6,1.0) → 西侧北上 y≈4.0 → 东北角
#       → 东侧南下 → 回起点 (0.6,1.0)，mission 输出"巡航完成，回到起点，自动停止悬停"
# 电压：7.87V 起 → 一圈后 6.71V（余量小，务必满电起步！）
# 落地：z 0.52→0.0 垂直下降，x/y 漂移 <0.2m，零猛冲
#       （对比 feed 版：贴地 y 0.82→1.95 猛冲 1.1m 撞树 → 撞击保护停桨）
#
# 关键结论：
# 1. feed（假VINS转圈）是贴地猛冲/拒解锁/方向错误的根源，已彻底移除
# 2. 飞控板载 UWB+IMU 融合 = 平滑 + 绝对坐标（沙盘全局）+ 无漂移，航点直接可用
# 3. mission 启动坑：缺 libquadrotor_msgs → 需 LD_LIBRARY_PATH（见步骤4）
# 4. ARM 用 SET_MODE(MAV_MODE_AUTO_ARMED) = 官方 App 解锁同款；听到解锁声即成功
# 5. ros2 CLI daemon 损坏 → 一律用 rclpy 脚本发命令（/tmp/send_cmd.py）
# ================================================================
