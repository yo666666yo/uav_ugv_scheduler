# DDS 总线抽象层
# Topic 定义对应《公共接口规范》第 5 节；ZRDDS 接入前先用内存实现跑通业务闭环。

from __future__ import annotations

import asyncio
import logging
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Awaitable, Callable

logger = logging.getLogger(__name__)

# 规范第 5 节的四个 Topic
TOPIC_AGENT_STATE = "/group4/agent/state"
TOPIC_TASK_ASSIGNMENT = "/group4/task/assignment"
TOPIC_TASK_FEEDBACK = "/group4/task/feedback"
TOPIC_MISSION_RESULT = "/group4/mission/result"

Handler = Callable[[str, dict], Awaitable[None]]


class DDSBus(ABC):
    """ZRDDS 总线抽象。接入真实 ZRDDS 时实现 ZRDDSBus 即可，业务代码零改动。"""

    @abstractmethod
    async def publish(self, topic: str, payload: dict) -> None: ...

    @abstractmethod
    def subscribe(self, topic: str, handler: Handler) -> None: ...


class InMemoryBus(DDSBus):
    """同进程内存总线：开发/中期演示用。payload 走 dict（JSON 语义），
    与 ZRDDS 类型化消息一一对应。"""

    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._log: list[tuple[str, dict]] = []  # 调试用消息留痕

    async def publish(self, topic: str, payload: dict) -> None:
        self._log.append((topic, dict(payload)))
        logger.debug("PUB %s %s", topic, payload.get("task_id") or payload.get("agent_id") or "")
        for h in list(self._subs[topic]):
            try:
                await h(topic, payload)
            except Exception:
                logger.exception("handler error on %s", topic)

    def subscribe(self, topic: str, handler: Handler) -> None:
        self._subs[topic].append(handler)

    @property
    def log(self) -> list[tuple[str, dict]]:
        return self._log


class ZRDDSBus(DDSBus):
    """ZRDDS 真实总线占位。

    ZRDDS 2.5.0 目前只提供 C++ SDK（x86_64）。可行接入路径：
      1. 用 C++ 写一个 bridge 进程：ZRDDS 订阅/发布 <-> 本进程 TCP/JSON（架构说明 11.2 节同款思路）；
      2. 或等官方 Python/ARM64 绑定。
    实现本类时：
      - publish: dict -> IDL 类型化消息后写入 ZRDDS writer；
      - subscribe: 注册 ZRDDS reader listener，把消息转回 dict 再调 handler。
    """

    def __init__(self, endpoint: str = "localhost:7400", domain_id: int = 42):
        raise NotImplementedError("ZRDDS 接入待 bridge 进程就绪后实现（见类注释）")

    async def publish(self, topic: str, payload: dict) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, handler: Handler) -> None:
        raise NotImplementedError


def build_bus(backend: str = "memory") -> DDSBus:
    if backend == "memory":
        return InMemoryBus()
    if backend == "zrdds":
        return ZRDDSBus()
    raise ValueError(f"未知总线后端: {backend}")
