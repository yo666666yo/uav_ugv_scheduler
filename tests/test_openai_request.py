"""Exercise OpenAI and DeepSeek SDK requests without network access."""

import json
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import httpx
from openai import OpenAI


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from llm_planner import generate_plan  # noqa: E402


class ProviderRequestTests(unittest.TestCase):
    def exercise_provider(
        self,
        provider: str,
        model: str,
        api_key_env: str,
        base_url: str,
    ) -> tuple[dict, dict, object]:
        expected_plan = json.loads((ROOT / "task_plan.example.json").read_text(encoding="utf-8"))
        captured = {}

        def handle(request: httpx.Request) -> httpx.Response:
            captured["body"] = json.loads(request.content)
            return httpx.Response(
                200,
                request=request,
                json={
                    "id": "resp_test",
                    "object": "response",
                    "created_at": 0,
                    "status": "completed",
                    "model": model,
                    "output": [
                        {
                            "id": "msg_test",
                            "type": "message",
                            "status": "completed",
                            "role": "assistant",
                            "content": [
                                {
                                    "type": "output_text",
                                    "annotations": [],
                                    "text": json.dumps(expected_plan),
                                }
                            ],
                        }
                    ],
                },
            )

        http_client = httpx.Client(transport=httpx.MockTransport(handle))
        client = OpenAI(api_key="test-only", base_url=base_url, http_client=http_client)
        try:
            with patch.dict(os.environ, {api_key_env: "test-only"}, clear=True), patch(
                "openai.OpenAI", return_value=client
            ) as constructor:
                actual_plan = generate_plan("test incident", provider, model)
        finally:
            client.close()

        self.assertEqual(actual_plan, expected_plan)
        return captured["body"], expected_plan, constructor

    def test_openai_request_uses_strict_json_schema(self) -> None:
        body, _, constructor = self.exercise_provider(
            provider="openai",
            model="gpt-5.6-luna",
            api_key_env="OPENAI_API_KEY",
            base_url="https://api.openai.com/v1",
        )
        self.assertEqual(body["model"], "gpt-5.6-luna")
        self.assertFalse(body["store"])
        self.assertEqual(body["text"]["format"]["type"], "json_schema")
        self.assertTrue(body["text"]["format"]["strict"])
        self.assertFalse(body["text"]["format"]["schema"]["additionalProperties"])
        constructor.assert_called_once_with(api_key="test-only")

    def test_deepseek_request_uses_compatible_endpoint_and_schema(self) -> None:
        body, _, constructor = self.exercise_provider(
            provider="deepseek",
            model="deepseek-v4-flash",
            api_key_env="DEEPSEEK_API_KEY",
            base_url="https://api.deepseek.com",
        )
        self.assertEqual(body["model"], "deepseek-v4-flash")
        self.assertEqual(body["text"]["format"]["type"], "json_schema")
        self.assertNotIn("strict", body["text"]["format"])
        constructor.assert_called_once_with(
            api_key="test-only", base_url="https://api.deepseek.com"
        )


if __name__ == "__main__":
    unittest.main()
