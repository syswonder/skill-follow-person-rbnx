"""Loopback OpenAI-compatible follow-distance endpoint for Robonix Pilot.

Robonix Pilot is configured with ``vlm.upstream`` pointing at this endpoint
instead of a real vision-language model.  That is the whole trick: Pilot still
owns the conversation, the RTDL tree and the Executor call, but the "model"
behind it is a bounded exact-match grammar rather than a network LLM.  A user
utterance therefore becomes exactly one Robonix capability call, or nothing.

This is the follow-only profile.  The Go2 reference routers a second intent
family as well (semantic-navigation landmarks, which need a saved landmark
store); the pure-follow deployment deliberately carries no semantic-navigation
skill, so that family is absent here and an unrecognized utterance returns an
empty tree rather than a guess.

The only capability this router can emit is
``go2_robottrack.follow_distance`` on contract
``robonix/primitive/follow/distance`` (see ``packages/go2_robottrack``'s
``capabilities/distance.v1.toml``).  It never publishes a Twist and never
bypasses the guard: it only moves the controller's target setpoint.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
from pathlib import Path
import re
import time
from typing import Any, Iterable

from semantic_intent_router.follow_distance import (
    FollowDistanceIntent,
    parse_follow_distance_intent,
)
from semantic_intent_router.person_lock import (
    PersonLockIntent,
    parse_person_lock_intent,
)


FOLLOW_DISTANCE_CONTRACT = "robonix/primitive/follow/distance"
FOLLOW_DISTANCE_CAPABILITY = "go2_robottrack.follow_distance"
# Robonix Pilot stores executor results as user-role messages but prefixes them
# with this fixed sentence.  Treating every user-role message as a fresh command
# would let an old result contaminate a later turn, and for a *relative* request
# ("离我远一点") would add another metre on every replay.
FEEDBACK_PREFIX = (
    "Executor feedback for the current RTDL leaf (not a new user request): "
)
# Pilot injects *more* than one kind of harness message as a user turn.  Besides
# the leaf feedback above it pushes a per-tree scope line before that tree's
# results ("Executor feedback scope: plan_id=..."), an image note for
# observation results ("Executor feedback: the previous capability call returned
# this image."), and short plan-control corrections ("Pilot harness feedback: ...
# is not active or is already stopping").  None of them is the user's utterance.
#
# Only FEEDBACK_PREFIX carries a parseable ``leaf_result``; the rest exist to be
# skipped.  They MUST be skipped, because ``current_turn`` finds the utterance by
# scanning for the last user turn: an unrecognised scope line becomes "the
# command", fails both intent parsers, and makes every completed plan end with
# "这句话不匹配跟随距离指令；未调用任何能力" -- on success and on failure alike.
# Startswith-ing the label rather than the full sentence survives Pilot rewording
# the tail of any of them.
_HARNESS_LABELS = ("Executor feedback", "Pilot harness feedback")
_CAP_LINE = re.compile(r"^\s*-\s*capability_name:\s*(\S+)\s*$", re.MULTILINE)
_MAX_REQUEST_BYTES = 2 * 1024 * 1024
_EXECUTION_MODES = frozenset({"preview", "live"})


@dataclass(frozen=True)
class Decision:
    """One OpenAI response envelope plus an HTTP status."""

    envelope: dict[str, Any]
    status: int = 200


@dataclass(frozen=True)
class Turn:
    command: str
    boundary: int
    prior_messages: list[dict[str, Any]]
    feedback_messages: list[dict[str, Any]]


def _empty_tree(description: str = "no safe follow action") -> dict[str, Any]:
    return {
        "op": "sequence",
        "op_id": 0,
        "description": description,
        "children": [],
    }


def _do(capability: str, args: dict[str, Any], description: str) -> dict[str, Any]:
    return {
        "op": "sequence",
        "op_id": 0,
        "description": description,
        "children": [
            {
                "op": "do",
                "op_id": 0,
                "description": description,
                "cap": capability,
                "args": args,
            }
        ],
    }


def _envelope(
    *,
    content: str,
    description: str,
    tree: dict[str, Any] | None = None,
    task_update: dict[str, str] | None = None,
) -> dict[str, Any]:
    return {
        "content": content,
        "rtdl_description": description,
        "rtdl": tree if tree is not None else _empty_tree(description),
        "task_update": task_update,
    }


def _message_text(message: dict[str, Any]) -> str:
    content = message.get("content")
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "\n".join(
            part.get("text", "")
            for part in content
            if isinstance(part, dict)
            and part.get("type") == "text"
            and isinstance(part.get("text"), str)
        )
    return ""


def advertised_capabilities(messages: Iterable[dict[str, Any]]) -> set[str]:
    """Read capability names only from trusted Pilot system messages.

    Pilot advertises the discovered capability surface as ``capability_name:``
    lines inside a system message.  Reading them here (rather than assuming the
    provider is up) is what makes the router fail closed when the RobotTrack
    provider has not registered its distance contract.
    """

    capabilities: set[str] = set()
    for message in messages:
        if message.get("role") == "system":
            capabilities.update(_CAP_LINE.findall(_message_text(message)))
    return capabilities


def _is_feedback(message: dict[str, Any]) -> bool:
    """True only for the leaf feedback that carries a parseable result."""

    return (
        message.get("role") == "user"
        and _message_text(message).startswith(FEEDBACK_PREFIX)
    )


def _is_harness_message(message: dict[str, Any]) -> bool:
    """True for any user-role message Pilot injected rather than the user.

    A superset of :func:`_is_feedback`: used only to find the utterance, never to
    collect results.  Keeping the two separate means a scope or harness line can
    never be mistaken for a leaf result by ``leaf_results`` either.
    """

    return (
        message.get("role") == "user"
        and _message_text(message).startswith(_HARNESS_LABELS)
    )


def current_turn(messages: list[dict[str, Any]]) -> Turn:
    """Split history at the last real user command."""

    boundary = -1
    command = ""
    for index, message in enumerate(messages):
        if message.get("role") == "user" and not _is_harness_message(message):
            boundary = index
            command = _message_text(message).strip()
    if boundary < 0:
        return Turn("", len(messages), list(messages), [])
    feedback = [
        message for message in messages[boundary + 1 :] if _is_feedback(message)
    ]
    return Turn(command, boundary, list(messages[:boundary]), feedback)


def _json_values(text: str) -> Iterable[Any]:
    decoder = json.JSONDecoder()
    index = 0
    while index < len(text):
        if text[index] not in "[{":
            index += 1
            continue
        try:
            value, consumed = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            index += 1
            continue
        yield value
        index += max(1, consumed)


def _walk(value: Any) -> Iterable[Any]:
    yield value
    if isinstance(value, dict):
        for child in value.values():
            if isinstance(child, str) and child.lstrip().startswith(("{", "[")):
                try:
                    child = json.loads(child)
                except json.JSONDecodeError:
                    pass
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def leaf_results(messages: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Recover executor leaf results from Pilot's user-role history."""

    results: list[dict[str, Any]] = []
    for message in messages:
        if not _is_feedback(message):
            continue
        payload = _message_text(message)[len(FEEDBACK_PREFIX) :]
        for value in _json_values(payload):
            for node in _walk(value):
                if not isinstance(node, dict):
                    continue
                leaf = node.get("leaf_result")
                if isinstance(leaf, dict):
                    results.append(leaf)
                elif {"contract_id", "success", "output"}.issubset(node):
                    results.append(node)
    return results


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return {}
        return parsed if isinstance(parsed, dict) else {}
    return {}


def _follow_distance_leaves(
    messages: Iterable[dict[str, Any]],
) -> list[dict[str, Any]]:
    return [
        leaf
        for leaf in leaf_results(messages)
        if leaf.get("contract_id") == FOLLOW_DISTANCE_CONTRACT
    ]


def _follow_distance_output_target(output: dict[str, Any]) -> float | None:
    value = output.get("target_distance_m")
    if isinstance(value, bool):
        return None
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None
    return distance if math.isfinite(distance) and distance > 0.0 else None


def _follow_distance_rejected(output: dict[str, Any]) -> bool:
    """Treat only explicit provider negatives as rejection.

    Robonix already records transport/contract failure in ``leaf.success``.
    Providers may additionally expose ``accepted`` or ``ok``.
    """

    return any(output.get(key) is False for key in ("accepted", "ok", "success"))


def _follow_distance_detail(output: dict[str, Any]) -> str:
    detail = output.get("detail")
    if isinstance(detail, str) and detail.strip():
        parsed = _mapping(detail)
        if parsed:
            for key in ("detail", "message", "reason"):
                value = parsed.get(key)
                if value is not None and str(value).strip():
                    return str(value).strip()
        return detail.strip()
    return ""


def _format_meters(value: float) -> str:
    return f"{value:g} 米"


def _follow_distance_action(intent: FollowDistanceIntent) -> str:
    if intent.operation == "set":
        return f"设置跟随距离为 {_format_meters(intent.meters)}"
    if intent.meters > 0.0:
        return f"将跟随距离增加 {_format_meters(intent.meters)}"
    return f"将跟随距离缩短 {_format_meters(abs(intent.meters))}"


def _follow_distance_task(
    intent: FollowDistanceIntent,
    status: str,
) -> dict[str, str]:
    return {
        "goal": _follow_distance_action(intent),
        "success_criterion": "RobotTrack 距离能力返回本次设置结果",
        "status": status,
    }


def _follow_distance_decision(
    turn: Turn,
    capabilities: set[str],
    intent: FollowDistanceIntent,
) -> Decision:
    """Dispatch one synchronous setpoint update, then terminate the turn.

    A relative request must never be replayed after executor feedback, because
    replaying it would add or subtract another metre.  Any feedback for this
    turn is therefore terminal, including malformed or mismatched feedback.
    """

    action = _follow_distance_action(intent)
    leaves = _follow_distance_leaves(turn.feedback_messages)
    if leaves:
        latest = leaves[-1]
        output = _mapping(latest.get("output"))
        if latest.get("success") is True and not _follow_distance_rejected(output):
            target = _follow_distance_output_target(output)
            content = (
                f"{action}已完成，当前目标距离为 {_format_meters(target)}"
                if target is not None
                else f"{action}已完成"
            )
        else:
            detail = _follow_distance_detail(output)
            suffix = f"：{detail}" if detail else ""
            content = f"{action}失败{suffix}；未重复下发本次调整"
        return Decision(
            _envelope(
                content=content,
                description="RobotTrack follow distance terminal",
                task_update=_follow_distance_task(intent, "done"),
            )
        )

    if turn.feedback_messages:
        return Decision(
            _envelope(
                content=f"{action}的执行反馈无法解析；未重复下发本次调整",
                description="RobotTrack follow distance feedback invalid",
                task_update=_follow_distance_task(intent, "done"),
            )
        )

    if FOLLOW_DISTANCE_CAPABILITY not in capabilities:
        return Decision(
            _envelope(
                content="Pilot 未公布 RobotTrack 跟随距离能力，未调整距离",
                description="RobotTrack follow distance capability unavailable",
                task_update=_follow_distance_task(intent, "done"),
            )
        )

    return Decision(
        _envelope(
            content=f"已识别“{action}”，交给 RobotTrack 跟随距离能力",
            description="update RobotTrack follow distance",
            tree=_do(
                FOLLOW_DISTANCE_CAPABILITY,
                {"operation": intent.operation, "meters": intent.meters},
                action,
            ),
            task_update=_follow_distance_task(intent, "in_progress"),
        )
    )


def _person_lock_action(intent: PersonLockIntent) -> str:
    return "锁定当前看到的人" if intent.operation == "lock" else "解除指定跟随"


def _person_lock_task(intent: PersonLockIntent, status: str) -> dict[str, str]:
    return {
        "goal": _person_lock_action(intent),
        "success_criterion": "RobotTrack 距离能力返回本次锁定结果",
        "status": status,
    }


def _person_lock_decision(
    turn: Turn,
    capabilities: set[str],
    intent: PersonLockIntent,
) -> Decision:
    """Dispatch one lock/unlock request through the existing distance capability.

    The specified-person lock deliberately reuses
    ``robonix/primitive/follow/distance`` with the ``lock`` / ``unlock``
    operations instead of introducing a contract of its own, so the deployment
    needs no new IDL, codegen, or capability advertisement.
    """

    action = _person_lock_action(intent)
    leaves = _follow_distance_leaves(turn.feedback_messages)
    if leaves:
        latest = leaves[-1]
        output = _mapping(latest.get("output"))
        if latest.get("success") is True and not _follow_distance_rejected(output):
            detail = _follow_distance_detail(output)
            suffix = f"：{detail}" if detail else ""
            content = f"{action}已受理{suffix}"
        else:
            detail = _follow_distance_detail(output)
            suffix = f"：{detail}" if detail else ""
            content = f"{action}失败{suffix}"
        return Decision(
            _envelope(
                content=content,
                description="RobotTrack specified-person lock terminal",
                task_update=_person_lock_task(intent, "done"),
            )
        )

    if turn.feedback_messages:
        return Decision(
            _envelope(
                content=f"{action}的执行反馈无法解析",
                description="RobotTrack specified-person lock feedback invalid",
                task_update=_person_lock_task(intent, "done"),
            )
        )

    if FOLLOW_DISTANCE_CAPABILITY not in capabilities:
        return Decision(
            _envelope(
                content="Pilot 未公布 RobotTrack 跟随能力，未执行指定跟随",
                description="RobotTrack specified-person lock capability unavailable",
                task_update=_person_lock_task(intent, "done"),
            )
        )

    return Decision(
        _envelope(
            content=f"已识别“{action}”，交给 RobotTrack 跟随能力",
            description="update RobotTrack specified-person lock",
            tree=_do(
                FOLLOW_DISTANCE_CAPABILITY,
                {"operation": intent.operation, "meters": 0.0},
                action,
            ),
            task_update=_person_lock_task(intent, "in_progress"),
        )
    )


def _execution_mode(value: Any) -> str:
    normalized = str(value or "preview").strip().lower()
    if normalized not in _EXECUTION_MODES:
        raise ValueError("semantic intent execution mode must be preview or live")
    return normalized


def _preview(messages: list[dict[str, Any]]) -> Decision:
    """Resolve one intent for display while emitting no capability leaf.

    This is the boundary that lets the whole Speech -> Liaison -> Pilot ->
    router path be demonstrated while the stack is still in dry-run: the
    utterance is parsed and reported, and Executor receives nothing.
    """

    turn = current_turn(messages)
    lock_intent = parse_person_lock_intent(turn.command)
    if lock_intent is not None:
        action = _person_lock_action(lock_intent)
        return Decision(
            _envelope(
                content=(
                    f"已识别“{action}”；当前是预览模式，未调用 RobotTrack "
                    "跟随能力"
                ),
                description="preview: no capability calls",
                task_update={
                    "goal": f"指定跟随预览：{action}",
                    "success_criterion": "预览模式未调用 RobotTrack 跟随能力",
                    "status": "done",
                },
            )
        )
    intent = parse_follow_distance_intent(turn.command)
    if intent is not None:
        action = _follow_distance_action(intent)
        return Decision(
            _envelope(
                content=(
                    f"已识别“{action}”；当前是预览模式，未调用 RobotTrack "
                    "跟随距离能力"
                ),
                description="preview: no capability calls",
                task_update={
                    "goal": f"跟随距离预览：{action}",
                    "success_criterion": "预览模式未调用 RobotTrack 跟随距离能力",
                    "status": "done",
                },
            )
        )
    if not turn.command:
        return Decision(
            _envelope(
                content="未识别到可预览的跟随距离指令；未调用任何能力",
                description="preview: empty command",
            )
        )
    return Decision(
        _envelope(
            content="这句话不匹配跟随距离指令；未调用任何能力",
            description="preview: follow distance intent not recognized",
        )
    )


def decide(messages: list[dict[str, Any]], *, execution_mode: str = "live") -> Decision:
    """Build one fail-closed RTDL response from Pilot history."""

    if _execution_mode(execution_mode) == "preview":
        return _preview(messages)

    capabilities = advertised_capabilities(messages)
    turn = current_turn(messages)
    lock_intent = parse_person_lock_intent(turn.command)
    if lock_intent is not None:
        return _person_lock_decision(turn, capabilities, lock_intent)
    intent = parse_follow_distance_intent(turn.command)
    if intent is not None:
        return _follow_distance_decision(turn, capabilities, intent)

    if not turn.command:
        return Decision(
            _envelope(
                content="未收到可识别的跟随距离指令",
                description="no follow distance utterance",
            )
        )
    return Decision(
        _envelope(
            content="这句话不匹配跟随距离指令；未调用任何能力",
            description="follow distance intent not recognized",
        )
    )


class Handler(BaseHTTPRequestHandler):
    execution_mode: str = "preview"

    def log_message(self, _format: str, *_args: Any) -> None:
        """Avoid logging utterances, headers, or query payloads."""

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.rstrip("/").endswith("/models"):
            self._json(
                200,
                {
                    "object": "list",
                    "data": [
                        {"id": "ranger-follow-distance-router", "object": "model"}
                    ],
                },
            )
            return
        self.send_error(404)

    def do_POST(self) -> None:  # noqa: N802
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400, "invalid content length")
            return
        if length <= 0 or length > _MAX_REQUEST_BYTES:
            self.send_error(413, "request body outside allowed size")
            return
        try:
            request = json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.send_error(400, "invalid JSON")
            return
        messages = request.get("messages")
        if not isinstance(messages, list) or not all(
            isinstance(item, dict) for item in messages
        ):
            self.send_error(400, "messages must be a list")
            return

        decision = decide(messages, execution_mode=self.execution_mode)
        envelope = decision.envelope
        model = str(request.get("model") or "ranger-follow-distance-router")
        if not request.get("stream", False):
            self._json(
                decision.status,
                {
                    "id": "ranger-follow-distance-router",
                    "object": "chat.completion",
                    "created": int(time.time()),
                    "model": model,
                    "choices": [
                        {
                            "index": 0,
                            "message": {
                                "role": "assistant",
                                "content": json.dumps(envelope, ensure_ascii=False),
                            },
                            "finish_reason": "stop",
                        }
                    ],
                },
            )
            return
        self._stream(model, envelope)

    def _stream(self, model: str, envelope: dict[str, Any]) -> None:
        payload = json.dumps(envelope, ensure_ascii=False)
        base = {
            "id": "ranger-follow-distance-router",
            "object": "chat.completion.chunk",
            "created": int(time.time()),
            "model": model,
        }
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        for delta, finish in (
            ({"role": "assistant", "content": payload}, None),
            ({}, "stop"),
        ):
            chunk = dict(
                base, choices=[{"index": 0, "delta": delta, "finish_reason": finish}]
            )
            self.wfile.write(
                f"data: {json.dumps(chunk, ensure_ascii=False)}\n\n".encode("utf-8")
            )
            self.wfile.flush()
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def build_server(
    host: str,
    port: int,
    *,
    execution_mode: str = "preview",
) -> ThreadingHTTPServer:
    address = ipaddress.ip_address(host)
    if not address.is_loopback:
        raise ValueError("semantic intent endpoint must bind to a literal loopback address")
    selected_mode = _execution_mode(execution_mode)
    handler = type(
        "ConfiguredFollowDistanceHandler",
        (Handler,),
        {"execution_mode": selected_mode},
    )
    return ThreadingHTTPServer((host, port), handler)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Bounded local follow-distance intent endpoint"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument(
        "--execution-mode",
        choices=sorted(_EXECUTION_MODES),
        default="preview",
        help="preview emits no capability leaves; live may emit follow_distance",
    )
    args = parser.parse_args()
    server = build_server(
        args.host, args.port, execution_mode=args.execution_mode
    )
    print(
        f"[semantic-intent] listening on http://{args.host}:{args.port}/v1 "
        f"mode={args.execution_mode}",
        flush=True,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
