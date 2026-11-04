"""Offline tests for the specified-person lock intent and its routing.

Mirrors ``test_follow_distance.py``: no Robonix, ROS, camera, or network.  The
router must turn one spoken utterance into exactly one RTDL leaf against the
existing distance capability, using its ``lock`` / ``unlock`` operations.
"""

from __future__ import annotations

import json
import unittest

from semantic_intent_router.follow_distance import parse_follow_distance_intent
from semantic_intent_router.person_lock import (
    PersonLockIntent,
    parse_person_lock_intent,
)
from semantic_intent_router.server import (
    FEEDBACK_PREFIX,
    FOLLOW_DISTANCE_CAPABILITY,
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


def _leaf(*, success: bool = True, status: str = "locked", detail: str = "") -> dict:
    return {
        "leaf_result": {
            "contract_id": "robonix/primitive/follow/distance",
            "success": success,
            "output": {"accepted": success, "status": status, "detail": detail},
        }
    }


def _children(decision) -> list[dict]:
    """The RTDL leaves the decision asks the executor to run."""

    return decision.envelope["rtdl"]["children"]


def _leaf_args(decision) -> dict:
    children = _children(decision)
    assert len(children) == 1, f"expected one leaf, got {children!r}"
    return children[0]["args"]


class PersonLockIntentTests(unittest.TestCase):
    ENROLL = (
        "跟着我",
        "跟随我",
        "跟着我。",
        "锁定我",
        "只跟着我",
        "认准这个人",
        "记住我",
        "请跟着我走",
        "跟着他",
        "锁定目标",
    )
    DISMISS = (
        "别跟了",
        "别跟着我了",
        "不用跟了",
        "不要跟着我了",
        "无需跟随",
        "取消跟随",
        "解除跟随",
        "停止跟随",
        "结束锁定",
        "解锁",
        "跟着大家",
    )

    def test_enroll_utterances(self) -> None:
        for text in self.ENROLL:
            with self.subTest(text=text):
                intent = parse_person_lock_intent(text)
                self.assertEqual(intent, PersonLockIntent("lock"))

    def test_dismiss_utterances(self) -> None:
        for text in self.DISMISS:
            with self.subTest(text=text):
                intent = parse_person_lock_intent(text)
                self.assertEqual(intent, PersonLockIntent("unlock"))

    def test_unrelated_utterances_are_ignored(self) -> None:
        for text in ("你好", "", "   ", "现在几点了", "往前走"):
            with self.subTest(text=text):
                self.assertIsNone(parse_person_lock_intent(text))

    def test_never_collides_with_follow_distance(self) -> None:
        """A lock utterance must never also parse as a distance change."""

        for text in ("离我远一点", "靠近我一点", "距离设为3米", "跟我保持2米距离"):
            with self.subTest(text=text):
                self.assertIsNone(parse_person_lock_intent(text))
                self.assertIsNotNone(parse_follow_distance_intent(text))

    def test_dismiss_is_not_read_as_enroll(self) -> None:
        """"别跟着我了" contains "跟着我": ordering must favour unlock."""

        self.assertEqual(
            parse_person_lock_intent("别跟着我了"), PersonLockIntent("unlock")
        )


class PersonLockRoutingTests(unittest.TestCase):
    def test_live_enroll_emits_one_lock_leaf(self) -> None:
        decision = decide(
            [_user("跟着我"), CAP_SYSTEM_MESSAGE], execution_mode="live"
        )
        arguments = _leaf_args(decision)
        self.assertEqual(arguments.get("operation"), "lock")

    def test_live_dismiss_emits_one_unlock_leaf(self) -> None:
        decision = decide(
            [_user("别跟了"), CAP_SYSTEM_MESSAGE], execution_mode="live"
        )
        arguments = _leaf_args(decision)
        self.assertEqual(arguments.get("operation"), "unlock")

    def test_preview_emits_no_capability_leaf(self) -> None:
        decision = decide(
            [_user("跟着我"), CAP_SYSTEM_MESSAGE], execution_mode="preview"
        )
        self.assertEqual(_children(decision), [])
        self.assertIn("预览模式", decision.envelope["content"])

    def test_feedback_terminates_without_replaying(self) -> None:
        decision = decide(
            [_user("跟着我"), CAP_SYSTEM_MESSAGE, _feedback(_leaf())],
            execution_mode="live",
        )
        self.assertEqual(_children(decision), [])
        self.assertIn("锁定", decision.envelope["content"])
        self.assertEqual(decision.envelope["task_update"]["status"], "done")

    def test_failed_feedback_is_reported(self) -> None:
        decision = decide(
            [
                _user("跟着我"),
                CAP_SYSTEM_MESSAGE,
                _feedback(
                    _leaf(success=False, status="reid_unavailable", detail="no model")
                ),
            ],
            execution_mode="live",
        )
        self.assertEqual(_children(decision), [])
        self.assertIn("失败", decision.envelope["content"])
        self.assertIn("no model", decision.envelope["content"])

    def test_missing_capability_is_reported(self) -> None:
        decision = decide([_user("跟着我")], execution_mode="live")
        self.assertEqual(_children(decision), [])
        self.assertIn("未公布", decision.envelope["content"])

    def test_distance_request_still_routes_to_adjust(self) -> None:
        """The new branch must not shadow the pre-existing distance routing."""

        decision = decide(
            [_user("离我远一点"), CAP_SYSTEM_MESSAGE], execution_mode="live"
        )
        arguments = _leaf_args(decision)
        self.assertEqual(arguments.get("operation"), "adjust")
        self.assertAlmostEqual(arguments.get("meters"), 1.0)

    def test_distance_set_still_routes_to_set(self) -> None:
        decision = decide(
            [_user("跟随距离设为3米"), CAP_SYSTEM_MESSAGE], execution_mode="live"
        )
        arguments = _leaf_args(decision)
        self.assertEqual(arguments.get("operation"), "set")
        self.assertAlmostEqual(arguments.get("meters"), 3.0)


if __name__ == "__main__":
    unittest.main()
