# Group4 总 Agent（master_agent）

对应《总Agent设备Agent与适配器架构说明》第 4 节的总 Agent 四大职责，数据契约严格遵循《公共接口规范 V0.1》。

## 模块结构

```
master_agent/
├── main.py                  # 入口（uvicorn；--demo 附加 3+3 模拟设备闭环）
├── config/master_agent.yaml # 本地配置（LLM key 走环境变量）
├── master_agent/
│   ├── models.py            # Pose / AgentState / TaskAssignment / TaskFeedback（规范第 3~8 节）
│   ├── bus.py               # DDS 总线抽象：InMemoryBus（现在）+ ZRDDSBus（接入点预留）
│   ├── registry.py          # 设备状态目录：1s 心跳 / 3s 警告 / 5s 离线（架构说明 4.2）
│   ├── scheduler.py         # 最近可用设备调度：能力过滤 + 欧氏距离（架构说明 4.3）
│   ├── mission_fsm.py       # 任务状态机：核查→判定→清理→完成（架构说明 4.4）
│   ├── parser.py            # 对话任务解析：DeepSeek LLM + 正则降级（架构说明 4.1）
│   └── server.py            # 网页后端：/api/chat、/ws/events、/api/manual/inspection-result
└── tests/                   # pytest 单元测试
```

## 快速开始

```bash
pip install fastapi uvicorn httpx pydantic pytest

# 单机闭环演示（3 无人机 + 3 无人车模拟设备）
python main.py --demo --auto-confirm --port 8100

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

# 跑测试
pytest tests/ -v
```

## 设计要点

1. **LLM 只做解析，不做决策**：设备选择、安全检查全部在 `scheduler.py` / `mission_fsm.py` 的确定性代码里（规范第 2 节、架构说明 4.1）。
2. **总 Agent 不发底层控制命令**：只通过 `TOPIC_TASK_ASSIGNMENT` 发结构化任务（规范第 2 节）。
3. **总线可替换**：业务代码只依赖 `DDSBus` 接口，ZRDDS bridge 就绪后实现 `ZRDDSBus` 即可，其余模块零改动。
4. **坐标系**：统一 `uwb_map`（规范 4.2），调度器会拒绝跨坐标系比较。

## 待办（对接期）

- [ ] ZRDDS bridge（C++ 进程：ZRDDS <-> TCP/JSON <-> 本模块）替代 InMemoryBus
- [ ] 把 uav_ugv_scheduler 仓库的 `Mission.*` Topic 迁移到 `/group4/*` 契约
- [ ] 6 台设备 Agent 进程化（当前 demo 在单进程内模拟）
- [ ] 失败重分配：`_on_failure` 目前直接置 FAILED，后期实现换设备重试（架构说明 4.4）
- [ ] 前端接 `/ws/events`（替换 telemetry.json 轮询）
