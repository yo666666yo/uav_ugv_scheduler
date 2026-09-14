# 4组无人机真实适配器（默认禁用）

## 当前结论

真实适配器已经按厂商 `fcu_core_ros2` 接口预留，但默认配置为：

```json
"real_control_enabled": false
```

在此状态下可以读取状态和调用相机，但 `takeoff`、`goto_pose`、`return_to_pose`、`land` 均会在发布任何控制消息之前抛出 `RealControlDisabled`。

## 厂商控制链路

```text
无人机 Agent
  -> FanciInnovRealUAVAdapter（本项目安全门）
  -> ROS 2 Topic
  -> 厂商 fcu_bridge_001
  -> MAVLink over TCP
  -> Mcontroller 192.168.1.126:333
```

不由本项目直接猜测或拼装飞控 MAVLink 控制帧。厂商 bridge 负责 ROS 2 与 Mcontroller 之间的转换。

## 已确认的 ROS 2 接口

### 动作 Topic

- Topic：`/fcu_command/command`
- 类型：`std_msgs/msg/Int16`
- `1`：解锁
- `2`：上锁
- `3`：起飞
- `4`：降落

厂商还定义了任务演示与跟踪编号，但本项目真实适配器没有开放这些操作。

### 位置目标 Topic

- Topic：`/fcu_mission/mission_001`
- 类型：`std_msgs/msg/Float32MultiArray`
- 必须正好包含 11 项：

```text
[yaw, yaw_rate, px, py, pz, vx, vy, vz, ax, ay, az]
```

当前项目使用 `simple_target=true`，只填绝对位置和 yaw，速度、加速度及 yaw_rate 填 0。目标位置采用 UWB 全局坐标，单位分别为米和弧度。

## 为什么现在不能把开关改成 true

启用前至少应完成：老师授权、UWB 定位与坐标方向复核、飞行边界与禁飞区实测、低电量阈值验证、手动接管准备、无桨台架测试、限高低速首飞。配置文件中的检查项目前全部为 `false`，这是有意保留的安全状态。

## 离线验证

```bash
cd ~/Group4SwarmScheduler
python3 -m unittest discover -s tests -v
```

测试使用假命令后端，不连接无人机，不会发送任何飞控指令。
