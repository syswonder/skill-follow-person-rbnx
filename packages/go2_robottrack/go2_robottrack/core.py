"""ROS-independent RobotTrack protocol and command-plan core."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Any, Mapping, Sequence
from urllib.parse import urlparse

logger = logging.getLogger(__name__)


WAYPOINT_COUNT = 8
WAYPOINT_DIMENSIONS = 3
WAYPOINT_STRATEGY = "first"
CONTROL_DT = 0.1
MAX_VX = 0.15
MAX_WZ = 0.30
# The package default stays at the first physical-test value above. Explicit
# live profiles may request a slightly stronger forward command, but never
# exceed the already-established downstream smoother/chassis contract.
MAX_CONFIGURED_VX = 0.50
MAX_CONFIGURED_WZ = 0.40
MAX_PLAN_AGE_S = 1.5
DEFAULT_DISPATCH_HZ = 50.0
MODEL_INPUT_MODE = "center_crop_height"
MODEL_CROP_SIZE = 384

# --------------------------------------------------------------------- guard
# Ranger chassis adaptation.  Everything this guard re-implements used to live
# inside go2_chassis, which the Ranger deployment does not have:
#
#   * the fail-closed motion gate (allow_motion/safety_ack/allowed_modes),
#   * the 0.25 m/s and 0.40 rad/s chassis clamps,
#   * the 0.25 s command timeout and the 0.50 s zero preamble,
#   * a Go2 sport-mode service that translates and rotates simultaneously.
#
# RangerROSMessenger::TwistCmdCallback replaces all of that with behaviour the
# upstream follow controller does not expect: it discards linear.x entirely and
# pivots in place whenever |linear.x| / |angular.z| drops below its
# min_turn_radius, and it switches the whole 4WS steering geometry the moment
# linear.y is non-zero.  The guard restores the Go2 contract on top of that.
# The full command chain this sits at the end of:
#   robottrack -> source mux -> /cmd_vel_nav
#              -> Nav2 velocity smoother -> /cmd_vel_guard_input
#              -> Nav2 velocity guard   -> /cmd_vel_guarded
#              -> THIS GUARD            -> /cmd_vel -> ranger_chassis
# A deployment without the Nav2 service would feed this guard /cmd_vel_nav
# directly; both are configurable via guard_input_topic.
GUARD_INPUT_TOPIC = "/cmd_vel_guarded"
GUARD_OUTPUT_TOPIC = "/cmd_vel"
GUARD_TURN_POLICIES = ("clamp_yaw", "allow_spin", "block")
# The vendored ranger_ros2 inside the Ranger chassis primitive reuses
# RangerMiniV2Params for robot_model=ranger_mini_v3, whose min_turn_radius is
# 0.4764 m.  The published Ranger Mini 3.0 manual states an 0.81 m Ackermann
# turning radius.  The larger value is used here: clamping yaw for the stricter
# radius guarantees a command can never land in the chassis's
# `radius < min_turn_radius` branch, which silently zeroes linear.x.
GUARD_MIN_TURN_RADIUS_M = 0.81


class ProtocolError(ValueError):
    """The inference server returned a response outside the official contract."""


def _finite_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _absolute_topic(value: Any, name: str, default: str) -> str:
    topic = str(value or default)
    if (
        not topic.startswith("/")
        or topic == "/"
        or topic.endswith("/")
        or "//" in topic
        or any(character.isspace() for character in topic)
        or len(topic) > 255
    ):
        raise ValueError(f"{name} must be one normalized absolute ROS topic")
    return topic


def _positive_float(value: Any, name: str, default: float, maximum: float) -> float:
    result = _finite_float(default if value is None else value, name)
    if not 0.0 < result <= maximum:
        raise ValueError(f"{name} must be in (0, {maximum}]")
    return result


def _bounded_float(
    value: Any,
    name: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
    include_minimum: bool = True,
) -> float:
    result = _finite_float(default if value is None else value, name)
    minimum_ok = result >= minimum if include_minimum else result > minimum
    if not minimum_ok or result > maximum:
        bracket = "[" if include_minimum else "("
        raise ValueError(f"{name} must be in {bracket}{minimum}, {maximum}]")
    return result


def _explicit_bool(value: Any, name: str, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    raise ValueError(f"{name} must be a bool")


@dataclass(frozen=True)
class RuntimeConfig:
    """Normalized runtime settings shared by ROS and the Robonix provider."""

    mode: str = "dry-run"
    rgb_topic: str = "/go2/d435i/color/image_raw"
    depth_topic: str = "/go2/d435i/aligned_depth_to_color/image_raw"
    command_topic: str = "/go2/robottrack/cmd_vel_raw"
    server_url: str = "http://127.0.0.1:5801/eval_dual"
    instruction: str = "Follow the person ahead"
    model_input_mode: str = MODEL_INPUT_MODE
    model_crop_size: int = MODEL_CROP_SIZE
    jpeg_quality: int = 60
    http_timeout_s: float = 30.0
    waypoint_strategy: str = WAYPOINT_STRATEGY
    control_dt: float = CONTROL_DT
    dispatch_hz: float = DEFAULT_DISPATCH_HZ
    max_plan_age_s: float = MAX_PLAN_AGE_S
    max_vx: float = MAX_VX
    max_wz: float = MAX_WZ
    nav_raw_topic: str = "/go2/robottrack/nav_cmd_vel_raw"
    robottrack_raw_topic: str = "/go2/robottrack/cmd_vel_raw"
    selected_output_topic: str = "/cmd_vel_nav"
    selected_source: str = "robottrack"
    source_max_age_s: float = 0.25
    # Fixed-distance following is an opt-in layer over the already validated
    # RGB-only RobotTrack path.  Keeping this false preserves ordinary follow
    # byte-for-byte at the command boundary.
    distance_enabled: bool = False
    target_distance_m: float = 5.0
    min_target_distance_m: float = 0.5
    max_target_distance_m: float = 7.0
    distance_deadband_m: float = 0.25
    distance_kp: float = 0.25
    distance_max_forward_mps: float = 0.15
    distance_max_reverse_mps: float = 0.0
    distance_measurement_max_age_s: float = 0.60
    # With the specified-person lock deployed a measurement only exists because
    # a frame showed the enrolled person, so its lifetime is capped at the frame
    # itself rather than at the ordinary jitter tolerance: when the camera stops,
    # the robot stops instead of coasting on the last thing it saw.
    #
    # The window must cover the WHOLE estimator, and the estimator is aged from
    # the frame's arrival, not from the moment the result lands.  Measured on the
    # Ranger's Orin: HOG 185 ms THEN OSNet 60 ms, i.e. 210 ms median and 419 ms
    # worst end to end.  This default (0.25) was sized for OSNet alone and still
    # is, which is why the deployment overrides it to 0.60 in the manifest --
    # at 0.25 the dispatcher expires every measurement before it can use one.
    # See docs/specified-person-follow.md, Step 17.
    distance_identity_max_age_s: float = 0.25
    distance_min_confidence: float = 0.50
    distance_pair_max_delta_s: float = 0.20
    distance_center_fallback: bool = False
    # Minimum detector confidence for a box to count as a person at all.  The
    # HOG detector reports confidence as sigmoid(SVM margin), and its
    # ``hitThreshold`` is 0.0, so every box it returns scores ABOVE 0.5 by
    # construction -- the weakly-positive windows, which on background clutter
    # are mostly furniture and wall edges, are exactly the ones this cuts.  They
    # matter because the ReID gate scores every remaining box with OSNet and a
    # single frame over ``reid_match_threshold`` is enough to produce a target,
    # so one junk box that happens to resemble the template walks the robot off.
    # 0.0 disables the filter and reproduces the pre-existing behaviour.
    detector_min_confidence: float = 0.0
    # Specified-person lock (ReID/OSNet).  Off by default: when false no OSNet
    # model is loaded and the follow loop is exactly the pre-existing one.  When
    # true the ROS node loads the model and the provider's ``follow_distance``
    # capability additionally accepts the ``lock`` / ``unlock`` operations.
    reid_enabled: bool = False
    reid_model_dir: str = "/root/deploy/models/osnet"
    # Same-camera, same-session comparison, so the measured margin is wide: on
    # this rig a true match of the enrolled person scores 0.96-0.99.  The gate
    # is per-frame with no temporal confirmation, so a threshold is the only
    # thing standing between a background crop and a non-zero vx -- keep it well
    # clear of the noise floor rather than just above it.
    reid_match_threshold: float = 0.70
    reid_enroll_samples: int = 5
    reid_device: str = "cuda:0"
    # Ranger chassis guard (see GUARD_* above).  Defaults reproduce the
    # effective Go2 ceiling exactly: min(robottrack 0.50, smoother 0.30,
    # chassis 0.25) == 0.25 m/s and min(0.40, 0.40) == 0.40 rad/s.
    guard_input_topic: str = GUARD_INPUT_TOPIC
    guard_output_topic: str = GUARD_OUTPUT_TOPIC
    guard_forward_only: bool = True
    guard_max_vx_mps: float = 0.25
    guard_max_wz_rps: float = 0.40
    guard_max_linear_accel_mps2: float = 0.30
    guard_max_angular_accel_rps2: float = 0.80
    guard_command_timeout_s: float = 0.25
    guard_publish_hz: float = 50.0
    guard_min_turn_radius_m: float = GUARD_MIN_TURN_RADIUS_M
    guard_turn_policy: str = "clamp_yaw"
    guard_spin_vx_epsilon_mps: float = 0.02

    @classmethod
    def from_mapping(cls, source: Mapping[str, Any] | None) -> "RuntimeConfig":
        cfg = dict(source or {})
        mux_value = cfg.get("source_mux", {})
        if mux_value is None:
            mux_value = {}
        if not isinstance(mux_value, Mapping):
            raise ValueError("source_mux must be a JSON object")
        mux = dict(mux_value)

        mode = str(cfg.get("mode") or "dry-run").strip().lower()
        if mode not in {"dry-run", "live"}:
            raise ValueError("mode must be exactly 'dry-run' or 'live'")

        guard_turn_policy = str(
            cfg.get("guard_turn_policy") or cls.guard_turn_policy
        ).strip().lower()
        if guard_turn_policy not in GUARD_TURN_POLICIES:
            raise ValueError(
                "guard_turn_policy must be one of "
                f"{', '.join(GUARD_TURN_POLICIES)}"
            )

        server_url = str(cfg.get("server_url") or cls.server_url).strip()
        parsed = urlparse(server_url)
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise ValueError("server_url must be an HTTP(S) URL")
        if parsed.username or parsed.password:
            raise ValueError("server_url must not contain credentials")
        if not parsed.path.endswith("/eval_dual"):
            raise ValueError("server_url must select the official /eval_dual endpoint")

        instruction = str(cfg.get("instruction") or cls.instruction).strip()
        if not instruction or len(instruction) > 1000:
            raise ValueError("instruction must contain 1..1000 characters")

        reid_model_dir = str(cfg.get("reid_model_dir") or cls.reid_model_dir).strip()
        if not reid_model_dir:
            raise ValueError("reid_model_dir must be a non-empty path")
        reid_device = str(cfg.get("reid_device") or cls.reid_device).strip()
        if not reid_device:
            raise ValueError("reid_device must be a non-empty torch device string")

        model_input_mode = str(
            cfg.get("model_input_mode") or MODEL_INPUT_MODE
        ).strip().lower()
        if model_input_mode != MODEL_INPUT_MODE:
            raise ValueError(
                "model_input_mode must be the official 'center_crop_height'"
            )
        model_crop_size = int(cfg.get("model_crop_size", MODEL_CROP_SIZE))
        if model_crop_size != MODEL_CROP_SIZE:
            raise ValueError("model_crop_size must be the official 384 pixels")

        jpeg_quality = int(cfg.get("jpeg_quality", cls.jpeg_quality))
        if not 1 <= jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")

        reid_enroll_samples = int(
            cfg.get("reid_enroll_samples", cls.reid_enroll_samples)
        )
        if not 1 <= reid_enroll_samples <= 60:
            raise ValueError("reid_enroll_samples must be in [1, 60]")

        strategy = str(cfg.get("waypoint_strategy") or WAYPOINT_STRATEGY).strip().lower()
        if strategy != WAYPOINT_STRATEGY:
            raise ValueError("waypoint_strategy must be 'first' for this integration")
        control_dt = _finite_float(cfg.get("control_dt", CONTROL_DT), "control_dt")
        if not math.isclose(control_dt, CONTROL_DT, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("control_dt must be 0.1 seconds")
        max_plan_age_s = _finite_float(
            cfg.get("max_plan_age_s", MAX_PLAN_AGE_S), "max_plan_age_s"
        )
        if not math.isclose(max_plan_age_s, MAX_PLAN_AGE_S, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("max_plan_age_s must be 1.5 seconds")

        max_vx = _positive_float(
            cfg.get("max_vx"),
            "max_vx",
            MAX_VX,
            MAX_CONFIGURED_VX,
        )
        max_wz = _positive_float(
            cfg.get("max_wz"),
            "max_wz",
            MAX_WZ,
            MAX_CONFIGURED_WZ,
        )
        distance_enabled = _explicit_bool(
            cfg.get("distance_enabled"),
            "distance_enabled",
            cls.distance_enabled,
        )
        reid_enabled = _explicit_bool(
            cfg.get("reid_enabled"),
            "reid_enabled",
            cls.reid_enabled,
        )
        if reid_enabled and not distance_enabled:
            # The lock is enforced inside the RGB-D estimator (it needs the
            # per-frame person boxes the distance path already computes), so it
            # cannot run without the distance path.  Switch it off and say so
            # loudly instead of refusing to start: a typo in the new feature's
            # configuration must never take the pre-existing follow loop down
            # with it.
            logger.warning(
                "reid_enabled requires distance_enabled; the specified-person "
                "lock is disabled and ordinary following continues"
            )
            reid_enabled = False
        min_target_distance_m = _bounded_float(
            cfg.get("min_target_distance_m"),
            "min_target_distance_m",
            cls.min_target_distance_m,
            minimum=0.50,
            maximum=7.0,
        )
        max_target_distance_m = _bounded_float(
            cfg.get("max_target_distance_m"),
            "max_target_distance_m",
            cls.max_target_distance_m,
            minimum=min_target_distance_m,
            maximum=7.0,
        )
        target_distance_m = _bounded_float(
            cfg.get("target_distance_m"),
            "target_distance_m",
            cls.target_distance_m,
            minimum=min_target_distance_m,
            maximum=max_target_distance_m,
        )
        distance_max_forward_mps = _bounded_float(
            cfg.get("distance_max_forward_mps"),
            "distance_max_forward_mps",
            min(cls.distance_max_forward_mps, max_vx),
            minimum=0.0,
            maximum=max_vx,
        )
        distance_max_reverse_mps = _bounded_float(
            cfg.get("distance_max_reverse_mps"),
            "distance_max_reverse_mps",
            cls.distance_max_reverse_mps,
            minimum=0.0,
            maximum=0.0,
        )
        selected_source = str(
            mux.get("selected_source", cfg.get("selected_source", "robottrack"))
        ).strip().lower()
        if selected_source not in {"navigation", "robottrack"}:
            raise ValueError("selected_source must be 'navigation' or 'robottrack'")

        robottrack_raw_topic = _absolute_topic(
            mux.get(
                "robottrack_input_topic",
                cfg.get("robottrack_raw_topic", cls.robottrack_raw_topic),
            ),
            "robottrack_raw_topic",
            cls.robottrack_raw_topic,
        )
        command_topic = _absolute_topic(
            cfg.get("command_topic"), "command_topic", cls.command_topic
        )
        if command_topic != robottrack_raw_topic:
            raise ValueError(
                "command_topic and source_mux.robottrack_input_topic must match"
            )

        return cls(
            mode=mode,
            rgb_topic=_absolute_topic(
                cfg.get("rgb_topic"), "rgb_topic", cls.rgb_topic
            ),
            depth_topic=_absolute_topic(
                cfg.get("depth_topic"), "depth_topic", cls.depth_topic
            ),
            command_topic=command_topic,
            server_url=server_url,
            instruction=instruction,
            model_input_mode=model_input_mode,
            model_crop_size=model_crop_size,
            jpeg_quality=jpeg_quality,
            http_timeout_s=_positive_float(
                cfg.get("http_timeout_s"), "http_timeout_s", cls.http_timeout_s, 120.0
            ),
            waypoint_strategy=strategy,
            control_dt=control_dt,
            dispatch_hz=_positive_float(
                cfg.get("dispatch_hz"), "dispatch_hz", DEFAULT_DISPATCH_HZ, 100.0
            ),
            max_plan_age_s=max_plan_age_s,
            max_vx=max_vx,
            max_wz=max_wz,
            nav_raw_topic=_absolute_topic(
                mux.get(
                    "nav_input_topic",
                    cfg.get("nav_raw_topic", cls.nav_raw_topic),
                ),
                "nav_raw_topic",
                cls.nav_raw_topic,
            ),
            robottrack_raw_topic=robottrack_raw_topic,
            selected_output_topic=_absolute_topic(
                mux.get(
                    "output_topic",
                    cfg.get("selected_output_topic", cls.selected_output_topic),
                ),
                "selected_output_topic",
                cls.selected_output_topic,
            ),
            selected_source=selected_source,
            source_max_age_s=_positive_float(
                mux.get("max_age_s", cfg.get("source_max_age_s")),
                "source_max_age_s",
                cls.source_max_age_s,
                MAX_PLAN_AGE_S,
            ),
            distance_enabled=distance_enabled,
            target_distance_m=target_distance_m,
            min_target_distance_m=min_target_distance_m,
            max_target_distance_m=max_target_distance_m,
            distance_deadband_m=_bounded_float(
                cfg.get("distance_deadband_m"),
                "distance_deadband_m",
                cls.distance_deadband_m,
                minimum=0.0,
                maximum=2.0,
            ),
            distance_kp=_bounded_float(
                cfg.get("distance_kp"),
                "distance_kp",
                cls.distance_kp,
                minimum=0.0,
                maximum=2.0,
                include_minimum=False,
            ),
            distance_max_forward_mps=distance_max_forward_mps,
            distance_max_reverse_mps=distance_max_reverse_mps,
            distance_measurement_max_age_s=_bounded_float(
                cfg.get("distance_measurement_max_age_s"),
                "distance_measurement_max_age_s",
                cls.distance_measurement_max_age_s,
                minimum=0.05,
                maximum=2.0,
            ),
            distance_identity_max_age_s=_bounded_float(
                cfg.get("distance_identity_max_age_s"),
                "distance_identity_max_age_s",
                cls.distance_identity_max_age_s,
                minimum=0.05,
                maximum=2.0,
            ),
            distance_min_confidence=_bounded_float(
                cfg.get("distance_min_confidence"),
                "distance_min_confidence",
                cls.distance_min_confidence,
                minimum=0.0,
                maximum=1.0,
            ),
            distance_pair_max_delta_s=_bounded_float(
                cfg.get("distance_pair_max_delta_s"),
                "distance_pair_max_delta_s",
                cls.distance_pair_max_delta_s,
                minimum=0.01,
                maximum=1.0,
            ),
            distance_center_fallback=_explicit_bool(
                cfg.get("distance_center_fallback"),
                "distance_center_fallback",
                cls.distance_center_fallback,
            ),
            detector_min_confidence=_bounded_float(
                cfg.get("detector_min_confidence"),
                "detector_min_confidence",
                cls.detector_min_confidence,
                minimum=0.0,
                maximum=1.0,
            ),
            reid_enabled=reid_enabled,
            reid_model_dir=reid_model_dir,
            reid_match_threshold=_bounded_float(
                cfg.get("reid_match_threshold"),
                "reid_match_threshold",
                cls.reid_match_threshold,
                minimum=0.01,
                maximum=1.0,
            ),
            reid_enroll_samples=reid_enroll_samples,
            reid_device=reid_device,
            guard_input_topic=_absolute_topic(
                cfg.get("guard_input_topic"),
                "guard_input_topic",
                cls.guard_input_topic,
            ),
            guard_output_topic=_absolute_topic(
                cfg.get("guard_output_topic"),
                "guard_output_topic",
                cls.guard_output_topic,
            ),
            guard_forward_only=_explicit_bool(
                cfg.get("guard_forward_only"),
                "guard_forward_only",
                cls.guard_forward_only,
            ),
            guard_max_vx_mps=_bounded_float(
                cfg.get("guard_max_vx_mps"),
                "guard_max_vx_mps",
                cls.guard_max_vx_mps,
                minimum=0.0,
                maximum=MAX_CONFIGURED_VX,
                include_minimum=False,
            ),
            guard_max_wz_rps=_bounded_float(
                cfg.get("guard_max_wz_rps"),
                "guard_max_wz_rps",
                cls.guard_max_wz_rps,
                minimum=0.0,
                maximum=MAX_CONFIGURED_WZ,
                include_minimum=False,
            ),
            guard_max_linear_accel_mps2=_bounded_float(
                cfg.get("guard_max_linear_accel_mps2"),
                "guard_max_linear_accel_mps2",
                cls.guard_max_linear_accel_mps2,
                minimum=0.0,
                maximum=10.0,
                include_minimum=False,
            ),
            guard_max_angular_accel_rps2=_bounded_float(
                cfg.get("guard_max_angular_accel_rps2"),
                "guard_max_angular_accel_rps2",
                cls.guard_max_angular_accel_rps2,
                minimum=0.0,
                maximum=20.0,
                include_minimum=False,
            ),
            guard_command_timeout_s=_bounded_float(
                cfg.get("guard_command_timeout_s"),
                "guard_command_timeout_s",
                cls.guard_command_timeout_s,
                minimum=0.0,
                maximum=5.0,
                include_minimum=False,
            ),
            guard_publish_hz=_bounded_float(
                cfg.get("guard_publish_hz"),
                "guard_publish_hz",
                cls.guard_publish_hz,
                minimum=1.0,
                maximum=200.0,
            ),
            guard_min_turn_radius_m=_bounded_float(
                cfg.get("guard_min_turn_radius_m"),
                "guard_min_turn_radius_m",
                cls.guard_min_turn_radius_m,
                minimum=0.0,
                maximum=10.0,
                include_minimum=False,
            ),
            guard_turn_policy=guard_turn_policy,
            guard_spin_vx_epsilon_mps=_bounded_float(
                cfg.get("guard_spin_vx_epsilon_mps"),
                "guard_spin_vx_epsilon_mps",
                cls.guard_spin_vx_epsilon_mps,
                minimum=0.0,
                maximum=1.0,
            ),
        )

    def as_ros_parameters(self) -> dict[str, Any]:
        """Return the flat parameter map consumed by ``RobotTrackNode``."""

        return {
            field: getattr(self, field)
            for field in self.__dataclass_fields__
        }


@dataclass(frozen=True)
class VelocityCommand:
    vx: float = 0.0
    wz: float = 0.0

    def clipped(self, max_vx: float = MAX_VX, max_wz: float = MAX_WZ) -> "VelocityCommand":
        vx = _finite_float(self.vx, "vx")
        wz = _finite_float(self.wz, "wz")
        return VelocityCommand(
            vx=max(-float(max_vx), min(float(max_vx), vx)),
            wz=max(-float(max_wz), min(float(max_wz), wz)),
        )


ZERO_COMMAND = VelocityCommand()


@dataclass(frozen=True)
class EncodedFrame:
    sequence: int
    jpeg: bytes
    source_timestamp: float
    received_monotonic: float


class LatestFrameMailbox:
    """A one-slot mailbox: every incoming frame atomically replaces the prior one."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._latest: EncodedFrame | None = None
        self._sequence = 0

    def put(
        self,
        jpeg: bytes,
        *,
        source_timestamp: float,
        received_monotonic: float | None = None,
    ) -> EncodedFrame:
        payload = bytes(jpeg)
        if not payload:
            raise ValueError("jpeg frame must not be empty")
        stamp = _finite_float(source_timestamp, "source_timestamp")
        received = (
            time.monotonic()
            if received_monotonic is None
            else _finite_float(received_monotonic, "received_monotonic")
        )
        with self._condition:
            self._sequence += 1
            frame = EncodedFrame(self._sequence, payload, stamp, received)
            self._latest = frame
            self._condition.notify_all()
            return frame

    def latest(self) -> EncodedFrame | None:
        with self._condition:
            return self._latest

    def wait_after(
        self,
        sequence: int,
        *,
        timeout_s: float,
        stop_event: threading.Event | None = None,
    ) -> EncodedFrame | None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._latest is None or self._latest.sequence <= sequence:
                if stop_event is not None and stop_event.is_set():
                    return None
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(min(remaining, 0.1))
            return self._latest

    @property
    def retained_count(self) -> int:
        with self._condition:
            return 0 if self._latest is None else 1


Waypoint = tuple[float, float, float]


@dataclass(frozen=True)
class InferencePlan:
    command: VelocityCommand
    waypoints: tuple[Waypoint, ...] | None
    control_dt: float = CONTROL_DT
    velocity_source: str = "base_velocity"


def _parse_vector(value: Any, name: str, length: int) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ProtocolError(f"{name} must be an array of length {length}")
    if len(value) != length:
        raise ProtocolError(f"{name} must be an array of length {length}")
    try:
        return tuple(_finite_float(item, f"{name}[{index}]") for index, item in enumerate(value))
    except (TypeError, ValueError) as error:
        raise ProtocolError(str(error)) from error


def _parse_waypoints(value: Any) -> tuple[Waypoint, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ProtocolError("waypoints must be an 8x3 array")
    if len(value) != WAYPOINT_COUNT:
        raise ProtocolError("waypoints must contain exactly 8 rows")
    rows: list[Waypoint] = []
    for index, row in enumerate(value):
        vector = _parse_vector(row, f"waypoints[{index}]", WAYPOINT_DIMENSIONS)
        rows.append((vector[0], vector[1], vector[2]))
    return tuple(rows)


def parse_inference_response(
    response: Mapping[str, Any],
    *,
    max_vx: float = MAX_VX,
    max_wz: float = MAX_WZ,
) -> InferencePlan:
    """Parse official ``/eval_dual`` responses and return a bounded command.

    The full server response carries both ``waypoints`` and ``base_velocity``.
    The official server also supports a velocity-only trimmed response, so the
    bridge accepts either representation while validating every representation
    that is present. If velocity is omitted, the direct controller uses waypoint
    index 1 over the official 0.1 second horizon.
    """

    if not isinstance(response, Mapping):
        raise ProtocolError("inference response must be a JSON object")
    if response.get("error"):
        raise ProtocolError(f"inference server error: {response['error']}")

    waypoints_value = response.get("waypoints")
    if waypoints_value is None:
        waypoints_value = response.get("trajectory")
    waypoints = None if waypoints_value is None else _parse_waypoints(waypoints_value)

    velocity_value = response.get("base_velocity")
    velocity_name = "base_velocity"
    if velocity_value is None:
        velocity_value = response.get("velocity")
        velocity_name = "velocity"

    if velocity_value is not None:
        velocity = _parse_vector(velocity_value, velocity_name, 3)
        command = VelocityCommand(velocity[0], velocity[2]).clipped(max_vx, max_wz)
        source = velocity_name
    elif waypoints is not None:
        first_motion_waypoint = waypoints[1]
        command = VelocityCommand(
            first_motion_waypoint[0] / CONTROL_DT,
            first_motion_waypoint[2] / CONTROL_DT,
        ).clipped(max_vx, max_wz)
        source = "waypoints[1]"
    else:
        raise ProtocolError("response must contain waypoints/trajectory or velocity")

    if "control_dt" in response:
        try:
            server_dt = _finite_float(response["control_dt"], "control_dt")
        except (TypeError, ValueError) as error:
            raise ProtocolError(str(error)) from error
        if not math.isclose(server_dt, CONTROL_DT, rel_tol=0.0, abs_tol=1e-6):
            raise ProtocolError("server control_dt does not match the official 0.1 seconds")

    return InferencePlan(
        command=command,
        waypoints=waypoints,
        control_dt=CONTROL_DT,
        velocity_source=source,
    )


@dataclass(frozen=True)
class DispatchState:
    command: VelocityCommand
    reason: str
    age_s: float | None
    version: int


class PlanStore:
    """Thread-safe latest inference plan with the official 1.5 second expiry."""

    def __init__(
        self,
        *,
        max_plan_age_s: float = MAX_PLAN_AGE_S,
        max_vx: float = MAX_VX,
        max_wz: float = MAX_WZ,
    ) -> None:
        self._max_plan_age_s = float(max_plan_age_s)
        self._max_vx = float(max_vx)
        self._max_wz = float(max_wz)
        self._lock = threading.Lock()
        self._plan: InferencePlan | None = None
        self._arrival_monotonic = 0.0
        self._version = 0

    def update(self, plan: InferencePlan, *, now: float | None = None) -> int:
        arrival = time.monotonic() if now is None else _finite_float(now, "now")
        bounded = InferencePlan(
            command=plan.command.clipped(self._max_vx, self._max_wz),
            waypoints=plan.waypoints,
            control_dt=plan.control_dt,
            velocity_source=plan.velocity_source,
        )
        with self._lock:
            self._plan = bounded
            self._arrival_monotonic = arrival
            self._version += 1
            return self._version

    def clear(self) -> None:
        with self._lock:
            self._plan = None
            self._arrival_monotonic = 0.0
            self._version += 1

    def dispatch(self, *, now: float | None = None) -> DispatchState:
        current = time.monotonic() if now is None else _finite_float(now, "now")
        with self._lock:
            plan = self._plan
            arrival = self._arrival_monotonic
            version = self._version
        if plan is None:
            return DispatchState(ZERO_COMMAND, "no_plan", None, version)
        age = max(0.0, current - arrival)
        if age > self._max_plan_age_s:
            return DispatchState(ZERO_COMMAND, "stale_plan", age, version)
        return DispatchState(plan.command, "active_plan", age, version)
