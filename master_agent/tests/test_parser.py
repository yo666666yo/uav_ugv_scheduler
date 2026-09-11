import asyncio
import json

import httpx
import pytest

from master_agent.parser import IntentParser


VALID_MISSION = {
    "mission_type": "INSPECT_AND_CLEAR",
    "target": {"frame_id": "uwb_map", "x": 3.2, "y": 5.1, "z": 0.0},
}
VALID_INTENT = {"mission": VALID_MISSION, "error": None}


def test_fallback_parses_coordinates_without_an_api_key(monkeypatch):
    monkeypatch.delenv("DEEPSEEK_API_KEY", raising=False)
    parser = IntentParser(provider="deepseek")
    result = asyncio.run(parser.parse("请检查（3.2，5.1）的垃圾"))
    assert result.target is not None
    assert (result.target.x_m, result.target.y_m) == (3.2, 5.1)


def test_rejects_noncanonical_frame():
    invalid = json.loads(json.dumps(VALID_INTENT))
    invalid["mission"]["target"]["frame_id"] = "map"
    with pytest.raises(ValueError, match="frame_id=uwb_map"):
        IntentParser._load_json(json.dumps(invalid))


def test_no_coordinate_result_is_preserved():
    result = IntentParser._load_json(
        json.dumps(
            {
                "mission": None,
                "error": {"code": "NO_COORDINATE", "message": "请提供坐标"},
            }
        )
    )
    assert result.error == "NO_COORDINATE"
    assert result.target is None


def test_deepseek_uses_chat_completions(monkeypatch):
    captured = {}

    async def handle(request: httpx.Request) -> httpx.Response:
        captured["path"] = request.url.path
        captured["body"] = json.loads(request.content)
        return httpx.Response(
            200,
            request=request,
            json={"choices": [{"message": {"content": json.dumps(VALID_INTENT)}}]},
        )

    transport = httpx.MockTransport(handle)
    real_client = httpx.AsyncClient

    def client_factory(*args, **kwargs):
        kwargs["transport"] = transport
        return real_client(*args, **kwargs)

    monkeypatch.setattr(httpx, "AsyncClient", client_factory)
    result = asyncio.run(IntentParser(api_key="test", provider="deepseek").parse("test"))
    assert result.target is not None
    assert captured["path"] == "/chat/completions"
    assert captured["body"]["response_format"] == {"type": "json_object"}


def test_openai_uses_responses_with_strict_schema(monkeypatch):
    captured = {}

    async def handle(request: httpx.Request) -> httpx.Response:
        captured["path"] = request.url.path
        captured["body"] = json.loads(request.content)
        return httpx.Response(
            200,
            request=request,
            json={
                "output": [
                    {"content": [{"type": "output_text", "text": json.dumps(VALID_INTENT)}]}
                ]
            },
        )

    transport = httpx.MockTransport(handle)
    real_client = httpx.AsyncClient

    def client_factory(*args, **kwargs):
        kwargs["transport"] = transport
        return real_client(*args, **kwargs)

    monkeypatch.setattr(httpx, "AsyncClient", client_factory)
    result = asyncio.run(IntentParser(api_key="test", provider="openai").parse("test"))
    assert result.target is not None
    assert captured["path"] == "/v1/responses"
    response_format = captured["body"]["text"]["format"]
    assert response_format["type"] == "json_schema"
    assert response_format["strict"] is True


def test_api_failure_falls_back_to_regex(monkeypatch):
    async def fail(*args, **kwargs):
        raise httpx.ConnectError("offline")

    monkeypatch.setattr(IntentParser, "_parse_with_llm", fail)
    result = asyncio.run(IntentParser(api_key="test", provider="deepseek").parse("(1.0, 2.0)"))
    assert result.target is not None
    assert (result.target.x_m, result.target.y_m) == (1.0, 2.0)
