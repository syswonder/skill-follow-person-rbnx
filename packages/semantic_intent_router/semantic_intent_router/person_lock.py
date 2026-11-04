"""Pure Chinese intent parsing for the specified-person lock.

Sibling of :mod:`semantic_intent_router.follow_distance`, with the same
deliberate lack of Robonix, ROS, camera, or model imports.  It only names the
requested lock operation; the follow provider performs it later through the
``lock`` / ``unlock`` operations of its existing distance capability.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import unicodedata
from typing import Literal

_TRAILING_PUNCTUATION = " ,，。.!！?？、;；:：'\"“”‘’"

# Enrol the person the robot is currently looking at.
_LOCK_PATTERNS = tuple(
    re.compile(pattern)
    for pattern in (
        r"^(?:请)?(?:跟着|跟随|锁定|记住|认准)(?:我|这个人|这位|他|她)$",
        r"^(?:请)?(?:只|就)(?:跟着|跟随)(?:我|这个人|这位|他|她)$",
        r"^(?:请)?(?:跟着|跟随)(?:我)(?:走|来)$",
        r"^(?:请)?锁定(?:目标|跟随目标)$",
    )
)

# Forget the enrolled person and return to ordinary following.
_UNLOCK_PATTERNS = tuple(
    re.compile(pattern)
    for pattern in (
        r"^(?:请)?(?:别|不要|不用|无需)(?:再)?(?:跟着|跟随|跟)(?:我|他|她|这个人)?(?:了)?$",
        r"^(?:请)?(?:取消|解除|停止|结束)(?:跟随|锁定)$",
        r"^(?:请)?解锁$",
        r"^(?:请)?(?:跟着|跟随)(?:所有人|大家)$",
    )
)


@dataclass(frozen=True)
class PersonLockIntent:
    """One requested change to the specified-person lock."""

    operation: Literal["lock", "unlock"]


def _normalize_utterance(text: str) -> str:
    normalized = unicodedata.normalize("NFKC", str(text)).casefold()
    normalized = re.sub(r"\s+", "", normalized)
    return normalized.strip(_TRAILING_PUNCTUATION)


def parse_person_lock_intent(text: str) -> PersonLockIntent | None:
    """Parse an enrol/dismiss utterance, or ``None`` for anything else.

    Unlock is tested first, because "别跟着我了" also contains the substring
    "跟着我" that the enroll patterns are built around.  Every pattern is a
    full match, so ordinary conversation and follow-distance requests such as
    "离我远一点" return ``None``.
    """

    command = _normalize_utterance(text)
    if not command:
        return None
    for pattern in _UNLOCK_PATTERNS:
        if pattern.fullmatch(command):
            return PersonLockIntent("unlock")
    for pattern in _LOCK_PATTERNS:
        if pattern.fullmatch(command):
            return PersonLockIntent("lock")
    return None


__all__ = ["PersonLockIntent", "parse_person_lock_intent"]
