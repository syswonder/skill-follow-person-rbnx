from __future__ import annotations

import json
import threading
import unittest
import urllib.request

from semantic_intent_router.server import (
    FEEDBACK_PREFIX,
    FOLLOW_DISTANCE_CAPABILITY,
    build_server,
    current_turn,
    decide,
)


CAP_SYSTEM_MESSAGE = {
    "role": "system",
    "content": f"- capability_name: {FOLLOW_DISTANCE_CAPABILITY}\n",
}


def _user(text: str) -> dict[str, str]:
    return {"role": "user", "content": text}


def _feedback(payload: dict) -> dict[str, str]:
    return {"role": "user", "content": FEEDBACK_PREFIX + json.dumps(payload)}


def _leaf(
    *,
    success: bool = True,
    accepted: bool | None = True,
    target_distance_m: float | None = 5.0,
) -> dict:
    output: dict = {"accepted": accepted} if accepted is not None else {}
    if target_distance_m is not None:
        output["target_distance_m"] = target_distance_m
    return {
        "leaf_result": {
            "contract_id": "robonix/primitive/follow/distance",
            "success": success,
            "output": output,
        }
    }


class TurnSplittingTests(unittest.TestCase):
    def test_last_real_command_wins_over_feedback(self) -> None:
        turn = current_turn([_user("靠近一点"), _feedback(_leaf())])
        self.assertEqual(turn.command, "靠近一点")
        self.assertEqual(len(turn.feedback_messages), 1)

    def test_feedback_only_history_has_no_command(self) -> None:
        turn = current_turn([_feedback(_leaf())])
        self.assertEqual(turn.command, "")
        self.assertEqual(turn.feedback_messages, [])


class DecideTests(unittest.TestCase):
    def test_set_command_emits_exactly_one_capability_call(self) -> None:
        decision = decide([CAP_SYSTEM_MESSAGE, _user("跟随距离设为3米")])
        children = decision.envelope["rtdl"]["children"]
        self.assertEqual(len(children), 1)
        self.assertEqual(children[0]["cap"], FOLLOW_DISTANCE_CAPABILITY)
        self.assertEqual(
            children[0]["args"], {"operation": "set", "meters": 3.0}
        )

    def test_relative_command_emits_a_signed_adjust(self) -> None:
        decision = decide([CAP_SYSTEM_MESSAGE, _user("离我远一点")])
        args = decision.envelope["rtdl"]["children"][0]["args"]
        self.assertEqual(args, {"operation": "adjust", "meters": 1.0})

    def test_missing_capability_fails_closed_without_a_call(self) -> None:
        decision = decide([_user("靠近一点")])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertIn("未公布", decision.envelope["content"])

    def test_feedback_terminates_the_turn_and_never_replays(self) -> None:
        # A replayed "adjust +1 m" would add another metre.
        decision = decide(
            [CAP_SYSTEM_MESSAGE, _user("离我远一点"), _feedback(_leaf())]
        )
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertEqual(decision.envelope["task_update"]["status"], "done")

    def test_successful_feedback_reports_the_new_target(self) -> None:
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("保持6米"),
                _feedback(_leaf(target_distance_m=6.0)),
            ]
        )
        self.assertIn("6 米", decision.envelope["content"])
        self.assertEqual(decision.envelope["task_update"]["status"], "done")

    def test_rejected_feedback_is_reported_as_failure(self) -> None:
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("保持9米"),
                _feedback(_leaf(accepted=False)),
            ]
        )
        self.assertIn("失败", decision.envelope["content"])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])

    def test_unparseable_feedback_is_terminal_not_retried(self) -> None:
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("靠近一点"),
                {"role": "user", "content": FEEDBACK_PREFIX + "not json"},
            ]
        )
        self.assertIn("未重复下发", decision.envelope["content"])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])

    def test_unrelated_utterance_calls_nothing(self) -> None:
        decision = decide([CAP_SYSTEM_MESSAGE, _user("去厨房")])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])

    def test_router_never_emits_a_velocity_or_ros_capability(self) -> None:
        # The router may only move the follow setpoint; it must never be able to
        # publish a Twist or reach the chassis directly.
        for utterance in ("靠近一点", "保持3米", "离我远一点", "随便说点什么"):
            decision = decide([CAP_SYSTEM_MESSAGE, _user(utterance)])
            rendered = json.dumps(decision.envelope, ensure_ascii=False)
            for forbidden in ("cmd_vel", "Twist", "chassis", "nav2"):
                self.assertNotIn(forbidden, rendered, utterance)


class PreviewModeTests(unittest.TestCase):
    """Preview is the dry-run boundary: parse and report, call nothing."""

    def test_recognized_utterance_emits_no_capability_call(self) -> None:
        decision = decide(
            [CAP_SYSTEM_MESSAGE, _user("保持3米")], execution_mode="preview"
        )
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertIn("预览模式", decision.envelope["content"])
        self.assertIn("3 米", decision.envelope["content"])

    def test_relative_utterance_is_recognized_in_preview(self) -> None:
        decision = decide([_user("靠近一点")], execution_mode="preview")
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertIn("缩短 1 米", decision.envelope["content"])

    def test_unrelated_utterance_still_calls_nothing(self) -> None:
        decision = decide([_user("你好")], execution_mode="preview")
        self.assertEqual(decision.envelope["rtdl"]["children"], [])

    def test_unknown_execution_mode_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            decide([_user("保持5米")], execution_mode="whatever")


class HttpSurfaceTests(unittest.TestCase):
    def setUp(self) -> None:
        self._server = build_server("127.0.0.1", 0, execution_mode="live")
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._thread.start()
        self._base = f"http://127.0.0.1:{self._server.server_address[1]}/v1"

    def tearDown(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)

    def _post(self, payload: dict) -> dict:
        request = urllib.request.Request(
            f"{self._base}/chat/completions",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=5.0) as response:
            return json.loads(response.read())

    def test_non_streaming_completion_returns_the_envelope(self) -> None:
        body = self._post(
            {
                "model": "go2-semantic-router",
                "messages": [CAP_SYSTEM_MESSAGE, _user("保持5米")],
            }
        )
        content = json.loads(body["choices"][0]["message"]["content"])
        self.assertEqual(
            content["rtdl"]["children"][0]["args"],
            {"operation": "set", "meters": 5.0},
        )

    def test_streaming_completion_terminates_with_done(self) -> None:
        request = urllib.request.Request(
            f"{self._base}/chat/completions",
            data=json.dumps(
                {
                    "stream": True,
                    "messages": [CAP_SYSTEM_MESSAGE, _user("靠近一点")],
                }
            ).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=5.0) as response:
            body = response.read().decode("utf-8")
        self.assertIn("chat.completion.chunk", body)
        self.assertTrue(body.rstrip().endswith("data: [DONE]"))

    def test_models_endpoint_is_available_for_pilot_discovery(self) -> None:
        with urllib.request.urlopen(f"{self._base}/models", timeout=5.0) as response:
            body = json.loads(response.read())
        self.assertEqual(body["object"], "list")

    def test_unknown_path_is_not_found(self) -> None:
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(f"{self._base}/other", timeout=5.0)
        self.assertEqual(caught.exception.code, 404)

    def test_non_loopback_bind_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            build_server("0.0.0.0", 0)


# Pilot injects these as user-role turns too.  Every one of them was measured in
# system/pilot/src/planner.rs; the scope line is the one that fired on *every*
# completed plan, because Pilot pushes it before a tree's results and after the
# user's utterance, so it was the last user turn the router saw.
PILOT_SCOPE_MESSAGE = (
    'Executor feedback scope: plan_id=1, independent RTDL tree="update RobotTrack '
    "specified-person lock\". Attribute the following results only to this tree. A "
    "failure here blocks dependent steps in this tree, but does not cancel or "
    "invalidate other in-flight trees."
)
PILOT_HARNESS_MESSAGE = (
    "Pilot harness feedback: plan-control target 1 is not active or is already "
    "stopping. Re-read In-flight trees and choose a currently listed plan_id. Do "
    "not retry a completed control operation."
)
PILOT_IMAGE_MESSAGE = (
    "Executor feedback: the previous capability call returned this image. Use it "
    "as observation data for the current task; this is not a new user request."
)


class PilotHarnessTurnTests(unittest.TestCase):
    """The harness's own user turns must never be mistaken for the utterance."""

    def test_scope_line_is_not_the_command(self) -> None:
        turn = current_turn(
            [_user("靠近一点"), _user(PILOT_SCOPE_MESSAGE), _feedback(_leaf())]
        )
        self.assertEqual(turn.command, "靠近一点")
        # Only the leaf feedback is a result; the scope line must not join it.
        self.assertEqual(len(turn.feedback_messages), 1)
        self.assertTrue(
            turn.feedback_messages[0]["content"].startswith(FEEDBACK_PREFIX)
        )

    def test_harness_and_image_lines_are_not_the_command(self) -> None:
        for injected in (PILOT_HARNESS_MESSAGE, PILOT_IMAGE_MESSAGE):
            turn = current_turn([_user("保持3米"), _user(injected)])
            self.assertEqual(turn.command, "保持3米", injected)
            self.assertEqual(turn.feedback_messages, [], injected)

    def test_harness_only_history_has_no_command(self) -> None:
        turn = current_turn([_user(PILOT_SCOPE_MESSAGE)])
        self.assertEqual(turn.command, "")
        self.assertEqual(turn.feedback_messages, [])

    def test_real_utterance_is_never_mistaken_for_harness(self) -> None:
        for utterance in ("跟着我", "离我远一点", "随便说点什么"):
            turn = current_turn([_user(utterance)])
            self.assertEqual(turn.command, utterance)

    def test_completed_plan_ends_with_the_result_not_the_fallback(self) -> None:
        # The exact message order Pilot builds after a plan succeeds: utterance,
        # the model's own reply, the scope line, then the leaf feedback.  Before
        # the fix this answered "这句话不匹配跟随距离指令；未调用任何能力".
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("保持6米"),
                {"role": "assistant", "content": "{\"rtdl\": {}}"},
                _user(PILOT_SCOPE_MESSAGE),
                _feedback(_leaf(target_distance_m=6.0)),
            ]
        )
        self.assertNotIn("不匹配", decision.envelope["content"])
        self.assertIn("6 米", decision.envelope["content"])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertEqual(decision.envelope["task_update"]["status"], "done")

    def test_failed_plan_reports_failure_not_the_fallback(self) -> None:
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("保持9米"),
                _user(PILOT_SCOPE_MESSAGE),
                _feedback(_leaf(success=False, accepted=False)),
            ]
        )
        self.assertNotIn("不匹配", decision.envelope["content"])
        self.assertIn("失败", decision.envelope["content"])
        self.assertEqual(decision.envelope["rtdl"]["children"], [])

    def test_harness_line_never_triggers_a_second_capability_call(self) -> None:
        # A harness correction arriving after a relative request must not make
        # the router replay it; that would add another metre.
        decision = decide(
            [
                CAP_SYSTEM_MESSAGE,
                _user("离我远一点"),
                _user(PILOT_SCOPE_MESSAGE),
                _user(PILOT_HARNESS_MESSAGE),
                _feedback(_leaf(target_distance_m=7.0)),
            ]
        )
        self.assertEqual(decision.envelope["rtdl"]["children"], [])
        self.assertEqual(decision.envelope["task_update"]["status"], "done")


if __name__ == "__main__":
    unittest.main()
