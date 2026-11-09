"""Small helpers shared by the ROS 2 Body drivers (import rclpy lazily)."""

from __future__ import annotations

import threading
import time
from typing import Any


class RclpyContext:
    """An independently owned ROS context, node and executor for each transport."""

    def __init__(self, node_name: str) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        self.context = Context()
        try:
            from rclpy.signals import SignalHandlerOptions

            rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):  # pragma: no cover - older rclpy
            if not self.context.ok():
                rclpy.init(context=self.context)
        self.rclpy = rclpy
        self.node: Any = Node(node_name, context=self.context)
        # Callbacks are short and share the default mutually exclusive group.
        # A worker pool adds GIL contention and can starve CAN RX/service replies.
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, name=f"{node_name}-spin", daemon=True)
        self._thread.start()

    def close(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=2.0)
            self._thread.join(timeout=2.0)
            self.node.destroy_node()
            if self.context.ok():
                self.rclpy.shutdown(context=self.context)
        except Exception:  # pragma: no cover
            pass


class Ticker:
    """Sleep-based fixed-rate loop helper."""

    def __init__(self, rate_hz: float) -> None:
        self.period = 1.0 / max(1.0, float(rate_hz))
        self._next = time.monotonic()

    def wait(self) -> None:
        self._next += self.period
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()


__all__ = ["RclpyContext", "Ticker"]
