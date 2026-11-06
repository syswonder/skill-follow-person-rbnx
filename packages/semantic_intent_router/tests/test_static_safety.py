from __future__ import annotations

import re
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parent.parent
MODULES = ROOT / "semantic_intent_router"
START_SH = ROOT / "scripts" / "start.sh"


class StaticSafetyTests(unittest.TestCase):
    def test_router_has_no_ros_or_robot_control_surface(self) -> None:
        """The router may only name the follow-distance capability.

        It must not be able to publish a velocity, reach the chassis, or import
        anything that could.  Anything on this list would mean the NL layer had
        grown a path around the guard.
        """

        sources = "\n".join(
            path.read_text(encoding="utf-8") for path in MODULES.glob("*.py")
        )
        for forbidden in (
            "rclpy",
            "geometry_msgs",
            "unitree_sdk",
            "/cmd_vel",
            "create_publisher",
            "/api/sport/request",
            "/lowcmd",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, sources)

    def test_router_uses_only_the_standard_library(self) -> None:
        """No third-party import: it must start before the rest of the stack."""

        imports: set[str] = set()
        for path in MODULES.glob("*.py"):
            for line in path.read_text(encoding="utf-8").splitlines():
                match = re.match(r"^(?:from|import)\s+([a-zA-Z_][\w.]*)", line)
                if match:
                    imports.add(match.group(1).split(".")[0])
        third_party = sorted(
            name
            for name in imports
            if name not in {"__future__", "semantic_intent_router"}
            and name
            not in {
                "argparse",
                "dataclasses",
                "http",
                "ipaddress",
                "json",
                "math",
                "pathlib",
                "re",
                "time",
                "typing",
                "unicodedata",
            }
        )
        self.assertEqual(third_party, [])

    def test_start_script_does_not_elevate_or_touch_ros(self) -> None:
        script = START_SH.read_text(encoding="utf-8")
        for forbidden in (
            r"\bsudo\b",
            r"\bapt(?:-get)?\b",
            r"\bsystemctl\b",
            r"\bdocker\b",
            r"\bros2\b",
            r"\brbnx\b",
            r"\bip\s+(?:addr|route|link)\b",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertIsNone(re.search(forbidden, script))

    def test_start_script_defaults_to_preview(self) -> None:
        script = START_SH.read_text(encoding="utf-8")
        self.assertIn("SEMANTIC_INTENT_EXECUTION_MODE:-preview", script)
        self.assertIn("--execution-mode", script)

    def test_execution_mode_is_validated(self) -> None:
        source = (MODULES / "server.py").read_text(encoding="utf-8")
        self.assertIn('_EXECUTION_MODES = frozenset({"preview", "live"})', source)
        self.assertIn("must be preview or live", source)

    def test_bind_is_loopback_only(self) -> None:
        source = (MODULES / "server.py").read_text(encoding="utf-8")
        self.assertIn("address.is_loopback", source)
        self.assertIn("must bind to a literal loopback address", source)

    def test_router_never_logs_utterances(self) -> None:
        """Request bodies and headers must not reach the log."""

        source = (MODULES / "server.py").read_text(encoding="utf-8")
        self.assertIn("def log_message(self, _format: str, *_args: Any)", source)


if __name__ == "__main__":
    unittest.main()
