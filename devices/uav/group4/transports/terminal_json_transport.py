"""Newline-delimited JSON transport for terminal demonstrations."""

from __future__ import annotations

import json
import threading
from typing import Any, Dict, Iterable, TextIO

from transports.base_transport import IncomingMessage


class TerminalJsonTransport:
    def __init__(self, input_stream: TextIO, output_stream: TextIO) -> None:
        self.input_stream = input_stream
        self.output_stream = output_stream
        self._write_lock = threading.Lock()

    def receive(self) -> Iterable[IncomingMessage]:
        for line_number, raw_line in enumerate(self.input_stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError as exc:
                yield IncomingMessage(
                    error_code="INVALID_JSON",
                    error_message=f"第 {line_number} 行不是有效 JSON：{exc.msg}",
                )
                continue
            if not isinstance(payload, dict):
                yield IncomingMessage(
                    error_code="INVALID_MESSAGE",
                    error_message=f"第 {line_number} 行的顶层必须是 JSON 对象",
                )
                continue
            yield IncomingMessage(payload=payload)

    def publish(self, message: Dict[str, Any]) -> None:
        encoded = json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._write_lock:
            self.output_stream.write(encoded)
            self.output_stream.flush()
