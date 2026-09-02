# Group4 总 Agent（master_agent）

对应《总Agent设备Agent与适配器架构说明》第 4 节的总 Agent 四大职责，数据契约严格遵循《公共接口规范 V0.1》。

## 模块结构

```
master_agent/
├── main.py                  # 总 Agent 入口（uvicorn）
├── run_devices.py           # 设备集群启动器（3 UAV + 3 UGV 独立进程）
├── config/master_agent.yaml # 本地配置（LLM key 走环境变量）
├── master_agent/
│   ├── models.py            # Pose / AgentState / TaskAssignment / TaskFeedback（规范第 3~8 节）
│   ├── bus.py               # DDS 总线抽象：InMemoryBus（现在）+ ZRDDSBus（接入点预留）
│   ├── registry.py          # 设备状态目录：1s 心跳 / 3s 警告 / 5s 离线（架构说明 4.2）
│   ├── scheduler.py         # 最近可用设备调度：能力过滤 + 欧氏距离（架构说明 4.3）
│   ├── mission_fsm.py       # 任务状态机：核查→判定→清理→完成 + 失败重分配（架构说明 4.4）
│   ├── parser.py            # 对话任务解析：DeepSeek LLM + 正则降级（架构说明 4.1）
│   ├── device_agent.py      # 设备 Agent：WS 接入总 Agent、SIM 执行层（可替换为真适配器）
│   └── server.py            # 网页后端 + /ws/agent 设备网关
└── tests/                   # 8 个 pytest（单元 + 多进程 e2e 闭环）
```

## 快速开始

```bash
pip install fastapi uvicorn httpx pydantic websockets pytest

# 终端 1：起总 Agent
python main.py --port 8100

# 终端 2：起设备集群（3 UAV + 3 UGV，独立进程，WS 连总 Agent）
python run_devices.py --master ws://localhost:8100

# 发布一个任务（等价于网页对话框输入）
curl -X POST http://localhost:8100/api/chat \
  -H 'Content-Type: application/json' \
  -d '{"message": "检查坐标（3.2，5.1）的障碍物并完成清理"}'

# 查看设备目录 / 任务列表 / 人工判定
curl http://localhost:8100/api/agents
curl http://localhost:8100/api/missions
curl -X POST http://localhost:8100/api/manual/inspection-result \
  -H 'Content-Type: application/json' \
  -d '{"mission_id": "mission_xxx", "garbage_detected": true}'

# 跑测试（8 个：单元 + 多进程 e2e）
pytest tests/ -v
```

## 设计要点

1. **LLM 只做解析，不做决策**：设备选择、安全检查全部在 `scheduler.py` / `mission_fsm.py` 的确定性代码里（规范第 2 节、架构说明 4.1）。
2. **设备 Agent 独立进程**：通过 `/ws/agent` 网关连接（hello/agent_state/task_feedback 上行，task_assignment 下行）；执行层是 `SimExecutor`（匀速移动 + 作业脚本），接真实设备时只替换这一层。
3. **失败重分配**：设备反馈 FAILED/MANUAL_CONTROL_REQUIRED 时拉黑该设备并重选（每任务最多 3 次），无可用设备才置 FAILED。
4. **总 Agent 不发底层控制命令**：只通过 `TOPIC_TASK_ASSIGNMENT` 发结构化任务（规范第 2 节）。
5. **总线可替换**：业务代码只依赖 `DDSBus` 接口，ZRDDS bridge 就绪后实现 `ZRDDSBus` 即可，其余模块零改动。
6. **坐标系**：统一 `uwb_map`（规范 4.2），调度器会拒绝跨坐标系比较。

## 待办（对接期）

- [ ] ZRDDS bridge（C++ 进程：ZRDDS <-> TCP/JSON <-> 本模块）替代 InMemoryBus 与 /ws/agent
- [ ] 把 uav_ugv_scheduler 仓库的 `Mission.*` Topic 迁移到 `/group4/*` 契约
- [ ] 真实设备适配器：替换 SimExecutor，接入飞控/底盘（架构说明第 5 节）
- [ ] 图片自动判定模型（当前人工按钮 /api/manual/inspection-result）
- [ ] 前端接 `/ws/events`（替换 telemetry.json 轮询）
