from __future__ import annotations

import unittest

from semantic_intent_router.follow_distance import (
    FollowDistanceIntent,
    parse_follow_distance_intent,
)


class ParseFollowDistanceIntentTests(unittest.TestCase):
    def test_relative_farther_is_one_default_step(self) -> None:
        for utterance in ("离我远一点", "再离我远点", "离我远点儿", "请离我远一点"):
            self.assertEqual(
                parse_follow_distance_intent(utterance),
                FollowDistanceIntent("adjust", 1.0),
                utterance,
            )

    def test_relative_closer_is_one_negative_step(self) -> None:
        for utterance in ("靠近一点", "再靠近点儿", "向我靠近一点", "请靠近一点"):
            self.assertEqual(
                parse_follow_distance_intent(utterance),
                FollowDistanceIntent("adjust", -1.0),
                utterance,
            )

    def test_absolute_set_accepts_arabic_digits(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("跟随距离设为3米"),
            FollowDistanceIntent("set", 3.0),
        )

    def test_absolute_set_accepts_chinese_numerals(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("保持五米"),
            FollowDistanceIntent("set", 5.0),
        )

    def test_absolute_set_accepts_chinese_decimal(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("设置为一点五米"),
            FollowDistanceIntent("set", 1.5),
        )

    def test_absolute_set_accepts_arabic_decimal(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("保持5.5米距离"),
            FollowDistanceIntent("set", 5.5),
        )

    def test_ten_metre_chinese_numeral_parses(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("保持十米"),
            FollowDistanceIntent("set", 10.0),
        )

    def test_surrounding_whitespace_and_punctuation_are_tolerated(self) -> None:
        self.assertEqual(
            parse_follow_distance_intent("  靠近一点。 "),
            FollowDistanceIntent("adjust", -1.0),
        )

    def test_unrelated_speech_returns_none(self) -> None:
        for utterance in (
            "",
            "你好",
            "去厨房",
            "跟着我",
            "前面那个人是谁",
            "停下来",
        ):
            self.assertIsNone(parse_follow_distance_intent(utterance), utterance)

    def test_distance_word_alone_is_not_a_command(self) -> None:
        # Without a set verb this is a question, not a setpoint change.
        self.assertIsNone(parse_follow_distance_intent("距离五米"))

    def test_partial_phrasing_is_rejected_rather_than_guessed(self) -> None:
        self.assertIsNone(parse_follow_distance_intent("稍微远一点"))
        self.assertIsNone(parse_follow_distance_intent("离我远"))
        self.assertIsNone(parse_follow_distance_intent("设置距离米"))


if __name__ == "__main__":
    unittest.main()
