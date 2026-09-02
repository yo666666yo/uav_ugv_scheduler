# 对话任务解析（架构说明 4.1 节）
# 自然语言 -> 结构化任务。LLM 只负责解析；
# 设备选择、状态判断和安全约束全部由确定性代码完成（调度器/状态机）。

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass

from .models import Pose

logger = logging.getLogger(__name__)

# 输出契约：LLM 必须只输出一个 JSON 对象
_SYSTEM_PROMPT = """你是无人机/无人车任务调度系统的任务解析器。
把用户的自然语言指令转换成结构化任务 JSON。只输出 JSON，不要任何解释。

输出格式：
{
  "mission_type": "INSPECT_AND_CLEAR",   // 固定值，当前只支持这一种任务
  "target": {"frame_id": "uwb_map", "x": <float>, "y": <float>, "z": 0.0}
}

用户没有给出坐标时输出：
{"error": "NO_COORDINATE", "message": "<向用户询问坐标的话术>"}
"""

# 降级用：匹配 "（3.2，5.1）" / "(3.2, 5.1)" / "坐标 3.2 5.1" / "x=3.2 y=5.1"
_COORD_RE = re.compile(
    r"([+-]?\d+(?:\.\d+)?)\s*[,，]\s*([+-]?\d+(?:\.\d+)?)"
)


@dataclass
class ParsedMission:
    mission_type: str = "INSPECT_AND_CLEAR"
    target: Pose | None = None
    error: str | None = None
    message: str | None = None


class IntentParser:
    """LLM 解析 + 正则降级。DEEPSEEK_API_KEY 未配置时自动走降级路径。"""

    def __init__(self, api_key: str | None = None, model: str = "deepseek-chat"):
        self.api_key = api_key or os.environ.get("DEEPSEEK_API_KEY", "")
        self.model = model
        self.base_url = os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com")

    @property
    def llm_enabled(self) -> bool:
        return bool(self.api_key)

    async def parse(self, text: str) -> ParsedMission:
        if self.llm_enabled:
            try:
                return await self._parse_with_llm(text)
            except Exception:
                logger.exception("LLM 解析失败，降级到正则")
        return self._parse_fallback(text)

    # ---------------------------------------------------------------- LLM 路径
    async def _parse_with_llm(self, text: str) -> ParsedMission:
        import httpx

        async with httpx.AsyncClient(timeout=20.0) as client:
            resp = await client.post(
                f"{self.base_url}/chat/completions",
                headers={"Authorization": f"Bearer {self.api_key}"},
                json={
                    "model": self.model,
                    "messages": [
                        {"role": "system", "content": _SYSTEM_PROMPT},
                        {"role": "user", "content": text},
                    ],
                    "temperature": 0.0,
                },
            )
            resp.raise_for_status()
            content = resp.json()["choices"][0]["message"]["content"].strip()
        return self._load_json(content)

    @staticmethod
    def _load_json(content: str) -> ParsedMission:
        # 容错：剥掉可能的 ```json 包裹
        content = re.sub(r"^```(?:json)?|```$", "", content, flags=re.MULTILINE).strip()
        data = json.loads(content)
        if "error" in data:
            return ParsedMission(error=data["error"], message=data.get("message"))
        t = data.get("target") or {}
        return ParsedMission(
            mission_type=data.get("mission_type", "INSPECT_AND_CLEAR"),
            target=Pose(
                frame_id=t.get("frame_id", "uwb_map"),
                x_m=float(t["x"]),
                y_m=float(t["y"]),
                z_m=float(t.get("z", 0.0)),
            ),
        )

    # ---------------------------------------------------------------- 正则降级
    @staticmethod
    def _parse_fallback(text: str) -> ParsedMission:
        m = _COORD_RE.search(text.replace("，", "，"))
        if not m:
            return ParsedMission(
                error="NO_COORDINATE",
                message="请提供目标坐标，例如：检查坐标（3.2，5.1）的障碍物并完成清理",
            )
        return ParsedMission(
            target=Pose(frame_id="uwb_map", x_m=float(m.group(1)), y_m=float(m.group(2)), z_m=0.0)
        )
