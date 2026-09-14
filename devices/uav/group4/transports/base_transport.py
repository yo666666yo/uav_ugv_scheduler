"""Transport-neutral interface for agent messages."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Protocol


@dataclass(frozen=True)
class IncomingMessage:
    payload: Dict[str, Any] | None = None
    error_code: str | None = None
    error_message: str | None = None


class AgentTransport(Protocol):
    def receive(self) -> Iterable[IncomingMessage]: ...

    def publish(self, message: Dict[str, Any]) -> None: ...

