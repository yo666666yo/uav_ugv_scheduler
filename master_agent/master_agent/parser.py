# 对话任务解析（架构说明 4.1 节）
# 自然语言 -> 结构化任务。LLM 只负责解析；
# 设备选择、状态判断和安全约束全部由确定性代码完成（调度器/状态机）。

from __future__ import annotations

import json
import logging
import math
import os
import re
from dataclasses import dataclass

from .models import Pose

logger = logging.getLogger(__name__)

# 输出契约：LLM 必须只输出一个 JSON 对象
_SYSTEM_PROMPT = """你是无人机/无人车任务调度系统的任务解析器。
把用户的自然语言指令转换成结构化任务 JSON。只输出 JSON，不要任何解释。

有坐标时输出：
{
  "mission": {
    "mission_type": "INSPECT_AND_CLEAR",
    "target": {"frame_id": "uwb_map", "x": <float>, "y": <float>, "z": 0.0}
  },
  "error": null
}

用户没有给出坐标时输出：
{"mission": null, "error": {"code": "NO_COORDINATE", "message": "<向用户询问坐标的话术>"}}
"""

# 降级用：匹配 "（3.2，5.1）" / "(3.2, 5.1)" / "坐标 3.2 5.1" / "x=3.2 y=5.1"
_COORD_RE = re.compile(
    r"([+-]?\d+(?:\.\d+)?)\s*[,，]\s*([+-]?\d+(?:\.\d+)?)"
)

_PROVIDERS = {
    "deepseek": {
        "api_key_env": "DEEPSEEK_API_KEY",
        "base_url_env": "DEEPSEEK_BASE_URL",
        "base_url": "https://api.deepseek.com",
        "model_env": "DEEPSEEK_MODEL",
        "model": "deepseek-chat",
    },
    "openai": {
        "api_key_env": "OPENAI_API_KEY",
        "base_url_env": "OPENAI_BASE_URL",
        "base_url": "https://api.openai.com/v1",
        "model_env": "OPENAI_MODEL",
        "model": "gpt-4.1-mini",
    },
}

_MISSION_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "mission_type": {"type": "string", "const": "INSPECT_AND_CLEAR"},
        "target": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "frame_id": {"type": "string", "const": "uwb_map"},
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
            },
            "required": ["frame_id", "x", "y", "z"],
        },
    },
    "required": ["mission_type", "target"],
}

_ERROR_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "code": {"type": "string", "const": "NO_COORDINATE"},
        "message": {"type": "string"},
    },
    "required": ["code", "message"],
}

_INTENT_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "mission": {"anyOf": [_MISSION_SCHEMA, {"type": "null"}]},
        "error": {"anyOf": [_ERROR_SCHEMA, {"type": "null"}]},
    },
    "required": ["mission", "error"],
}


@dataclass
class ParsedMission:
    mission_type: str = "INSPECT_AND_CLEAR"
    target: Pose | None = None
    error: str | None = None
    message: str | None = None


class IntentParser:
    """OpenAI/DeepSeek 任务解析，未配置或请求失败时自动降级为正则解析。"""

    def __init__(
        self,
        api_key: str | None = None,
        model: str | None = None,
        provider: str | None = None,
    ):
        self.provider = (provider or os.environ.get("LLM_PROVIDER", "deepseek")).strip().lower()
        if self.provider not in _PROVIDERS:
            raise ValueError(f"Unsupported LLM provider: {self.provider}")
        config = _PROVIDERS[self.provider]
        self.api_key = api_key or os.environ.get(config["api_key_env"], "")
        self.model = model or os.environ.get(config["model_env"], config["model"])
        self.base_url = os.environ.get(config["base_url_env"], config["base_url"]).rstrip("/")

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

        if self.provider == "deepseek":
            endpoint = f"{self.base_url}/chat/completions"
            payload = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": _SYSTEM_PROMPT},
                    {"role": "user", "content": text},
                ],
                "response_format": {"type": "json_object"},
                "temperature": 0.0,
            }
        else:
            endpoint = f"{self.base_url}/responses"
            payload = {
                "model": self.model,
                "instructions": _SYSTEM_PROMPT,
                "input": text,
                "text": {
                    "format": {
                        "type": "json_schema",
                        "name": "master_agent_intent",
                        "strict": True,
                        "schema": _INTENT_SCHEMA,
                    }
                },
                "store": False,
            }

        async with httpx.AsyncClient(timeout=20.0) as client:
            resp = await client.post(
                endpoint,
                headers={"Authorization": f"Bearer {self.api_key}"},
                json=payload,
            )
            resp.raise_for_status()
            data = resp.json()
            if self.provider == "deepseek":
                content = data["choices"][0]["message"]["content"].strip()
            else:
                content = self._openai_output_text(data)
        return self._load_json(content)

    @staticmethod
    def _openai_output_text(data: dict) -> str:
        for item in data.get("output", []):
            for content in item.get("content", []):
                if content.get("type") == "output_text" and content.get("text"):
                    return content["text"].strip()
        raise ValueError("OpenAI response contains no output_text")

    @staticmethod
    def _load_json(content: str) -> ParsedMission:
        # 容错：剥掉可能的 ```json 包裹
        content = re.sub(r"^```(?:json)?|```$", "", content, flags=re.MULTILINE).strip()
        data = json.loads(content)
        if not isinstance(data, dict):
            raise ValueError("LLM output must be a JSON object")
        if set(data) != {"mission", "error"}:
            raise ValueError("LLM mission output has missing or unsupported fields")
        if data["mission"] is None:
            error = data["error"]
            if not isinstance(error, dict) or set(error) != {"code", "message"}:
                raise ValueError("LLM error output has missing or unsupported fields")
            if error["code"] != "NO_COORDINATE":
                raise ValueError("unsupported LLM error code")
            return ParsedMission(error=error["code"], message=str(error["message"]))
        if data["error"] is not None:
            raise ValueError("LLM output cannot contain a mission and an error")
        data = data["mission"]
        if not isinstance(data, dict) or set(data) != {"mission_type", "target"}:
            raise ValueError("LLM mission has missing or unsupported fields")
        if data["mission_type"] != "INSPECT_AND_CLEAR":
            raise ValueError("unsupported mission_type")
        t = data.get("target") or {}
        if not isinstance(t, dict) or set(t) != {"frame_id", "x", "y", "z"}:
            raise ValueError("LLM target has missing or unsupported fields")
        if t["frame_id"] != "uwb_map":
            raise ValueError("LLM target must use frame_id=uwb_map")
        coordinates = [float(t[name]) for name in ("x", "y", "z")]
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError("LLM target coordinates must be finite")
        return ParsedMission(
            mission_type="INSPECT_AND_CLEAR",
            target=Pose(
                frame_id="uwb_map",
                x_m=coordinates[0],
                y_m=coordinates[1],
                z_m=coordinates[2],
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
