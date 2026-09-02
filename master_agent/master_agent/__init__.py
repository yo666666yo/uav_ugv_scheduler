"""Group4 总 Agent 包。

模块结构（对应架构说明第 4 节的四大职责）：
  models.py      数据契约：Pose / AgentState / TaskAssignment / TaskFeedback
  bus.py         DDS 总线抽象（内存实现可跑，ZRDDS 接入点预留）
  registry.py    设备状态目录（1s 心跳 / 3s 警告 / 5s 离线）
  scheduler.py   调度算法（最近可用设备）
  mission_fsm.py 任务状态机（核查 -> 判定 -> 清理 -> 完成）
  parser.py      对话任务解析（LLM + 正则降级）
  server.py      网页后端（/api/chat, /ws/events, 人工判定接口）
"""
