"""Thread-safe polling and caching for a real read-only state provider."""

from __future__ import annotations

import copy
import threading
import time
from typing import Any, Callable, Dict


class RealStateMonitor:
    def __init__(
        self,
        state_provider: Callable[[], Dict[str, Any]],
        *,
        poll_interval_s: float = 0.1,
        publish_interval_s: float = 1.0,
        stale_after_s: float = 1.5,
    ) -> None:
        self.state_provider = state_provider
        self.poll_interval_s = float(poll_interval_s)
        self.publish_interval_s = float(publish_interval_s)
        self.stale_after_s = float(stale_after_s)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._initialized = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: Dict[str, Any] | None = None
        self._latest_time = 0.0
        self._callback: Callable[[Dict[str, Any]], None] | None = None

    def set_publish_callback(
        self, callback: Callable[[Dict[str, Any]], None] | None
    ) -> None:
        with self._lock:
            self._callback = callback

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="group4-real-state-monitor",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        next_publish = 0.0
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                state = dict(self.state_provider() or {})
                state.pop("monitor_error", None)
            except Exception as exc:
                state = {
                    "online": False,
                    "device_online": False,
                    "readonly_ready": False,
                    "control_ready": False,
                    "telemetry_mode": "OFFLINE",
                    "position_valid": False,
                    "battery_valid": False,
                    "monitor_error": f"{type(exc).__name__}: {exc}",
                }
            now = time.monotonic()
            with self._lock:
                self._latest = copy.deepcopy(state)
                self._latest_time = now
                callback = self._callback
            self._initialized.set()
            if state.get("readonly_ready"):
                self._ready.set()
            if callback is not None and now >= next_publish:
                try:
                    callback(self.snapshot())
                except Exception:
                    # State publishing must not stop telemetry polling.
                    pass
                next_publish = now + self.publish_interval_s
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, self.poll_interval_s - elapsed))

    def wait_initialized(self, timeout_s: float) -> bool:
        return self._initialized.wait(float(timeout_s))

    def wait_readonly_ready(self, timeout_s: float) -> bool:
        return self._ready.wait(float(timeout_s))

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            state = copy.deepcopy(self._latest) if self._latest is not None else {}
            age = None if not self._latest_time else max(0.0, now - self._latest_time)
        state["state_age_s"] = age
        state["state_stale"] = age is None or age > self.stale_after_s
        if state["state_stale"]:
            state["readonly_ready"] = False
            state["control_ready"] = False
        return state

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.poll_interval_s * 5.0))
        close = getattr(self.state_provider, "close", None)
        if callable(close):
            close()
