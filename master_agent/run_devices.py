# 设备集群启动器：一条命令拉起 3 UAV + 3 UGV（独立进程版）
# 用法:
#   python run_devices.py                          # 6 台全部连 localhost:8100
#   python run_devices.py --master ws://192.168.1.146:8100
#   python run_devices.py --only drone_001,car_002 # 只起指定设备

from __future__ import annotations

import argparse
import asyncio
import logging

from master_agent.device_agent import AgentConfig, DeviceAgent
from master_agent.models import AgentType, Pose

# 与总 Agent demo 相同的初始布局（可按实际沙盘调整）
DEVICES = [
    ("drone_001", AgentType.UAV, 1.25, 2.10, ["INSPECT", "CAPTURE_IMAGE"]),
    ("drone_002", AgentType.UAV, 4.80, 1.60, ["INSPECT", "CAPTURE_IMAGE"]),
    ("drone_003", AgentType.UAV, 2.00, 5.50, ["INSPECT", "CAPTURE_IMAGE"]),
    ("car_001",   AgentType.UGV, 4.00, 2.00, ["PICKUP", "DELIVER"]),
    ("car_002",   AgentType.UGV, 5.00, 5.00, ["PICKUP", "DELIVER"]),
    ("car_003",   AgentType.UGV, 0.80, 3.40, ["PICKUP", "DELIVER"]),
]


async def main() -> None:
    p = argparse.ArgumentParser(description="3 UAV + 3 UGV 模拟设备集群")
    p.add_argument("--master", default="ws://localhost:8100", help="总 Agent 地址")
    p.add_argument("--only", default="", help="逗号分隔的 agent_id 列表，只启动这些设备")
    args = p.parse_args()

    only = {s.strip() for s in args.only.split(",") if s.strip()}
    agents = []
    for agent_id, atype, x, y, caps in DEVICES:
        if only and agent_id not in only:
            continue
        cfg = AgentConfig(
            agent_id=agent_id, agent_type=atype,
            start=Pose(x_m=x, y_m=y), capabilities=caps,
            master_url=args.master,
        )
        agents.append(DeviceAgent(cfg))

    logging.info("启动 %d 台设备 -> %s", len(agents), args.master)
    await asyncio.gather(*(a.run() for a in agents))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
