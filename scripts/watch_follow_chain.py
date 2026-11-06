#!/usr/bin/env python3
"""Live view of the whole follow command chain, for acceptance runs.

Run INSIDE the follow container:

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    python3 scripts/watch_follow_chain.py 2>/dev/null

One rclpy node subscribes to every hop and prints a refreshed table once a
second.  (Shelling out to `ros2 topic hz` per topic takes ~15 s per topic, which
is useless for watching a moving robot.)  The zenoh router logs
"Received Data for unknown expr_id" noise to stderr, hence the `2>/dev/null`.

The table is enough to tell apart the failure modes that look identical from the
outside:

  * a hop with 0 publishers  -> the stage above it is not running
  * a rate far below 50 Hz   -> that stage is stalling
  * vx pinned at 0 while wz moves
                             -> the chassis guard is holding forward motion,
                                which on the Ranger means the turn-radius clamp
                                is refusing the command

Ctrl-C to exit.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


CHAIN: tuple[tuple[str, str], ...] = (
    ("/ranger/robottrack/cmd_vel_raw", "模型原始输出（未融合距离）"),
    ("/cmd_vel_nav", "source mux 唯一输出"),
    ("/cmd_vel_guard_input", "velocity smoother 输出"),
    ("/cmd_vel_guarded", "velocity guard 输出"),
    ("/cmd_vel", "chassis guard 输出 -> 底盘"),
)


class _Hop:
    __slots__ = ("topic", "label", "count", "vx", "wz", "last", "first")

    def __init__(self, topic: str, label: str) -> None:
        self.topic = topic
        self.label = label
        self.count = 0
        self.vx = 0.0
        self.wz = 0.0
        self.last: float | None = None
        self.first: float | None = None


class ChainWatcher(Node):
    def __init__(self) -> None:
        super().__init__("follow_chain_watcher")
        self._hops = [_Hop(topic, label) for topic, label in CHAIN]
        self._last_counts: dict[int, int] = {id(hop): 0 for hop in self._hops}
        for hop in self._hops:
            self.create_subscription(
                Twist, hop.topic, self._make_callback(hop), 10
            )
        self.create_timer(1.0, self._render)

    def _make_callback(self, hop: _Hop):
        def _on(message: Twist) -> None:
            now = time.monotonic()
            hop.count += 1
            hop.vx = float(message.linear.x)
            hop.wz = float(message.angular.z)
            hop.last = now
            if hop.first is None:
                hop.first = now

        return _on

    def _render(self) -> None:
        now = time.monotonic()
        lines = [f"follow command chain   {time.strftime('%H:%M:%S')}   (Ctrl-C 退出)", ""]
        lines.append(
            f"{'topic':<26} {'pub':>4} {'sub':>4} {'Δmsg':>6} {'vx':>9} {'wz':>9}  备注"
        )
        lines.append("-" * 92)
        for hop in self._hops:
            pubs = len(self.get_publishers_info_by_topic(hop.topic))
            subs = len(self.get_subscriptions_info_by_topic(hop.topic))
            stale = hop.last is None or (now - hop.last) > 2.0
            delta = hop.count - self._last_counts[id(hop)]
            self._last_counts[id(hop)] = hop.count

            note = ""
            if pubs == 0:
                note = "无发布者 <- 上一级没在跑"
            elif stale:
                note = "无数据"
            elif hop.topic == "/cmd_vel" and pubs > 1:
                note = "!! 底盘有多个控制者"
            elif hop.topic == "/cmd_vel_nav" and pubs > 1:
                note = "!! /cmd_vel_nav 有多个发布者"
            elif abs(hop.vx) < 1e-6 and abs(hop.wz) > 1e-6 and delta > 0:
                note = "vx=0 只有转向"

            vx = "-" if stale else f"{hop.vx:+.3f}"
            wz = "-" if stale else f"{hop.wz:+.3f}"
            lines.append(
                f"{hop.topic:<26} {pubs:>4} {subs:>4} {delta:>6} {vx:>9} {wz:>9}  {note}"
            )
            lines.append(f"  {hop.label}")
        lines.append("")
        lines.append("检查点: /cmd_vel 的 pub 必须恰好为 1（只有 chassis guard）")
        lines.append("检查点: /cmd_vel_nav 的 pub 必须恰好为 1（只有 source mux）")
        lines.append("注意: sub 计数包含本观测器自己，所以比实际多 1")
        print("\033[2J\033[H" + "\n".join(lines), flush=True)


def main() -> int:
    rclpy.init(args=sys.argv[1:])
    node = ChainWatcher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what a SIGTERM (e.g. `timeout`) looks
        # like; exit quietly instead of dumping a traceback over the table.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
