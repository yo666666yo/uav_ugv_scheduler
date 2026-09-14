# 第 4 组终端 JSON 无人机 Agent

该服务使用标准输入/标准输出上的 JSON Lines 模拟总 Agent 与无人机 Agent 通信。每一行只能包含一个完整 JSON 对象。

当前只支持两种安全模式：

- `dry-run`：完整模拟检查、起飞、移动、拍照、返航和降落，不连接真实飞控；
- `readonly`：接收和记录任务，但以 `REAL_CONTROL_DISABLED` 拒绝执行。

当前版本不存在 `real` 启动选项，不能通过误填参数开启真实控制。

## ROS2 真实状态模式

在 1 号机的 `fanciswarm_ros2` 容器中，可使用真实 ROS2 Topic 作为任务前检查和状态发布来源：

```bash
source /opt/ros/humble/setup.bash
source /home/snowman/fcu_core_onboard/install/setup.bash
export ROS_DOMAIN_ID=10

python3 -m tools.run_terminal_agent \
  --config config/drone_001.real.disabled.json \
  --mode dry-run \
  --state-source ros2
```

该模式订阅：

- `/odom_global_001`：UWB 全局位置，bridge 已转换为米；
- `/imu_global_001`：姿态和朝向；
- `/batt_now_001`：电压；
- `/battery_remaining_001`：剩余电量，厂商的 `0` 按未知值处理。

该模式不创建控制发布器，不连接 MAVLink TCP 333。真实状态合格时执行模拟行为，状态不合格时拒绝任务。

## 启动

在项目根目录执行：

```bash
python3 -m tools.run_terminal_agent \
  --config config/drone_001.sim.json \
  --mode dry-run
```

服务启动后，将一条单行任务 JSON 粘贴到终端并按回车。

## 任务示例

```json
{"schema_version":"1.0","message_type":"TASK_ASSIGNMENT","timestamp":"2026-09-03T10:30:00+08:00","mission_id":"mission_001","task_id":"task_inspect_001","target_agent_id":"drone_001","task_type":"INSPECT_TARGET","parameters":{"target_position":{"frame_id":"uwb_map","x_m":2.5,"y_m":3.0,"z_m":0.0},"inspection_height_m":0.8,"target_label":"garbage"}}
```

协议也兼容项目原有的 `timestamp_ms` 和顶层 `target_pose` 格式。坐标单位统一为米，坐标系必须为 `uwb_map`。

## 输出

服务依次输出：

1. `AGENT_STARTED`；
2. 若干条 `TASK_FEEDBACK`；
3. 一条 `TASK_RESULT`；
4. 标准输入关闭时输出 `AGENT_STOPPED`。

标准输出只包含一行一条的 JSON，可被网页后端或未来的 ZRDDS Transport 直接解析。

## 任务去重

默认任务记录文件为：

```text
logs/terminal_agent/tasks.json
```

相同 `task_id` 再次提交时，服务返回 `DUPLICATE`，不会重新执行。需要重复测试时应使用新的 `task_id`。不要在真实控制场景中通过删除任务记录来强制重跑。

## 当前边界

- 使用 `--state-source simulated` 时，启动事件中的 `readonly_ready` 为 `null`；
- 使用 `--state-source ros2` 时，`dry-run` 使用真实起始位置和电池检查，但移动、相机和目标识别仍为模拟；
- 当前不连接 ZRDDS；
- 当前不会向 `/fcu_command/command` 或 `/fcu_mission/mission_001` 发布消息。
