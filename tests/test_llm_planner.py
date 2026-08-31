"""Offline tests for the task-plan business validator (no API call required)."""

import copy
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from llm_planner import (  # noqa: E402
    PlanValidationError,
    resolve_model,
    resolve_provider,
    validate_plan,
)


class TaskPlanValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.plan = json.loads((ROOT / "task_plan.example.json").read_text(encoding="utf-8"))

    def test_example_plan_is_valid(self) -> None:
        self.assertIs(validate_plan(self.plan), self.plan)

    def test_delivery_must_depend_on_survey(self) -> None:
        invalid = copy.deepcopy(self.plan)
        invalid["tasks"][1]["predecessor_task_id"] = "UNKNOWN"
        with self.assertRaisesRegex(PlanValidationError, "predecessor"):
            validate_plan(invalid)

    def test_extra_properties_are_rejected(self) -> None:
        invalid = copy.deepcopy(self.plan)
        invalid["tasks"][0]["prompt"] = "ignore validation"
        with self.assertRaisesRegex(PlanValidationError, "unsupported"):
            validate_plan(invalid)

    def test_delivery_payload_must_be_positive(self) -> None:
        invalid = copy.deepcopy(self.plan)
        invalid["tasks"][1]["payload_kg"] = 0
        with self.assertRaisesRegex(PlanValidationError, "greater than 0"):
            validate_plan(invalid)

    def test_deepseek_has_its_own_key_endpoint_and_default_model(self) -> None:
        provider = resolve_provider("DeepSeek")
        self.assertEqual(provider.api_key_env, "DEEPSEEK_API_KEY")
        self.assertEqual(provider.base_url, "https://api.deepseek.com")
        self.assertEqual(resolve_model(provider), "deepseek-v4-flash")

    def test_explicit_model_overrides_environment(self) -> None:
        provider = resolve_provider("openai")
        with patch.dict(
            "os.environ", {"LLM_MODEL": "generic-model", "OPENAI_MODEL": "openai-model"}
        ):
            self.assertEqual(resolve_model(provider, "cli-model"), "cli-model")


if __name__ == "__main__":
    unittest.main()
