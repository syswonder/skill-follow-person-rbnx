"""Prove that vx is non-zero only while the enrolled person is in frame.

The requirement is one line: with the specified-person lock deployed, forward
motion only while "the person I enrolled" is in the picture.  In-place rotation
is explicitly allowed, so only ``vx`` is under test here -- ``wz`` is whatever
the model asked for.

Every phase drives the *real* estimator, the *real* ``FollowDistanceRuntime``
and the *real* ``FixedDistanceController`` with the numbers actually deployed in
``robonix_manifest.yaml`` (read from the manifest, never copied here), at the
real rates: the camera delivers at 30 Hz and the dispatch timer runs at 50 Hz.
The detector is a stub that reports one person-sized box -- exactly what a HOG
false positive on furniture looks like -- and the depth image says 3.0 m, far
outside the deadband, so a live target would command full forward speed.

  A  lock not deployed, box present     moves   (the pre-existing loop)
  B  deployed, nobody said "follow me"  never   (there is no "me")
  C  deployed, nobody in frame yet      never   (waiting for the person)
  D  deployed, box present, frame 1     never   (identity not settled yet)
  E  enrolled, person in frame          moves   (the point of the feature)
  F  enrolled, person leaves the frame  bounded (stops within one window)
  G  enrolment failed, box present      never   (no template to verify)
  H  enrolled, camera stops entirely    bounded (no coast on a dead camera)
  I  deployed but no OSNet behind it    never   (no identity possible)

Nothing here touches ROS, a camera, or the robot.
"""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import yaml

DEPLOY_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(DEPLOY_DIR / "packages" / "go2_robottrack"))

from go2_robottrack.core import RuntimeConfig, VelocityCommand  # noqa: E402
from go2_robottrack.distance_control import DistanceMeasurement  # noqa: E402
from go2_robottrack.follow_distance_runtime import (  # noqa: E402
    configure_distance_runtime,
    distance_runtime,
)
from go2_robottrack.person_reid import ReidLockState  # noqa: E402
from go2_robottrack.rgbd_distance import (  # noqa: E402
    BoundingBox,
    PersonDetection,
    RgbdDistanceConfig,
    RgbdPersonDistanceEstimator,
)

FRAME_HZ = 30.0  # camera
DISPATCH_HZ = 50.0  # ros_node dispatch timer
INFERENCE_LATENCY_S = 0.06  # OSNet forward pass, measured on the Orin
DURATION_S = 1.2
T0 = 1000.0
PERSON = BoundingBox(10, 100, 200, 400)  # person-sized and centred
DEPTH_MM = 3000  # 3.0 m: far outside the 1.0 m deadband
MODEL_COMMAND = VelocityCommand(vx=0.40, wz=0.30)


def deployed_config() -> RuntimeConfig:
    """The robottrack block of the manifest, exactly as the node reads it."""

    with open(DEPLOY_DIR / "robonix_manifest.yaml", encoding="utf-8") as handle:
        manifest = yaml.safe_load(handle)
    block = next(
        entry for entry in manifest["primitive"] if entry["name"] == "go2_robottrack"
    )["config"]
    return RuntimeConfig.from_mapping(block)


class BoxDetector:
    """Reports whatever boxes the phase last assigned."""

    def __init__(self) -> None:
        self.boxes: list[PersonDetection] = []

    def detect(self, _bgr):
        return list(self.boxes)


class OneHotReid:
    """Feature keyed on the box's left edge: the same box scores 1.0."""

    feature_dim = 512

    def extract(self, _bgr, bboxes):
        features = []
        for box in bboxes:
            vector = np.zeros(512, dtype=np.float32)
            vector[box.x % 512] = 1.0
            features.append(vector)
        return features


class ZeroReid:
    """OSNet that cannot score the crop: every feature comes back all-zero."""

    feature_dim = 512

    def extract(self, _bgr, bboxes):
        return [np.zeros(512, dtype=np.float32) for _ in bboxes]


class Phase:
    """One timeline: so many frames carrying a person, then silence."""

    def __init__(
        self,
        label: str,
        *,
        deployed: bool = True,
        person_frames: int = 0,
        frame_count: int = 30,
        request_enroll: bool = True,
        lock_state: bool = True,
        enroll_samples: int | None = None,
        reid=None,
    ) -> None:
        self.label = label
        self.deployed = deployed
        self.person_frames = person_frames
        self.frame_count = frame_count
        self.request_enroll = request_enroll
        self.lock_state = lock_state
        self.enroll_samples = enroll_samples
        self.reid = reid

    @property
    def last_frame_at_s(self) -> float:
        """When the camera delivered its last frame, relative to the start."""

        return max(1, self.frame_count) / FRAME_HZ

    def run(self, config: RuntimeConfig):
        if not self.deployed:
            # The same deployment with the lock switched off: the pre-existing
            # loop, byte for byte.
            config = replace(config, reid_enabled=False)

        lock = None
        if self.deployed and self.lock_state:
            samples = self.enroll_samples or config.reid_enroll_samples
            lock = ReidLockState(
                threshold=config.reid_match_threshold, enroll_samples=samples
            )
            lock.configure(
                available=True,
                threshold=config.reid_match_threshold,
                enroll_samples=samples,
            )
            if self.request_enroll:
                lock.request_enroll()

        detector = BoxDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                enable_center_fallback=config.distance_center_fallback,
                reid_enabled=config.reid_enabled,
                reid_match_threshold=config.reid_match_threshold,
                reid_enroll_samples=config.reid_enroll_samples,
            ),
            # The lock is deployed but no model is behind it (phase I): the node
            # hands the estimator reid_state=None and no targets may exist.
            reid=(self.reid or OneHotReid()) if (self.deployed and self.lock_state) else None,
            reid_state=lock,
        )
        runtime = distance_runtime
        configure_distance_runtime(config, runtime=runtime)
        runtime.set_active(True)

        image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = np.full((480, 640), DEPTH_MM, dtype=np.uint16)
        limit = (
            config.distance_identity_max_age_s
            if config.reid_enabled
            else config.distance_measurement_max_age_s
        )

        received: float | None = None
        pending: list[tuple[float, object]] = []  # (commit_at, estimate)
        sources: list[str] = []
        vx_values: list[float] = []

        now = T0
        end = T0 + DURATION_S
        next_frame_at = T0
        frame_index = 0
        while now <= end:
            # The camera fires; the estimate lands one inference pass later.
            while next_frame_at <= now:
                if frame_index < self.frame_count:
                    detector.boxes = (
                        [PersonDetection(PERSON, 0.99)]
                        if frame_index < self.person_frames
                        else []
                    )
                    estimate = estimator.estimate(
                        image, depth, frame_timestamp_s=next_frame_at, now_s=now
                    )
                    pending.append(
                        (next_frame_at + INFERENCE_LATENCY_S, estimate)
                    )
                next_frame_at += 1.0 / FRAME_HZ
                frame_index += 1

            # ros_node._on_distance_estimate, delayed by the inference pass.
            due = [item for item in pending if item[0] <= now]
            pending = [item for item in pending if item[0] > now]
            for committed_at, estimate in due:
                sources.append(f"{estimate.status}/{estimate.source}")
                if estimate.valid and estimate.distance_m is not None:
                    runtime.update_measurement(
                        DistanceMeasurement(
                            distance_m=estimate.distance_m,
                            confidence=estimate.confidence,
                            received_monotonic=committed_at,
                        ),
                        status=estimate.status,
                        source=estimate.source,
                    )
                    received = committed_at
                else:
                    runtime.update_measurement(
                        None, status=estimate.status, source=estimate.source
                    )
                    received = None

            # ros_node._expire_distance_measurement
            if received is not None and now - received > limit:
                runtime.update_measurement(
                    None, status="stale_measurement", source=""
                )
                received = None

            result = runtime.apply(MODEL_COMMAND, now_monotonic=now)
            vx_values.append(result.command.vx)
            now += 1.0 / DISPATCH_HZ

        if lock is not None:
            status = lock.snapshot().status
        else:
            status = "not-deployed" if not self.deployed else "unavailable"
        return vx_values, sources, status


def main() -> int:
    config = deployed_config()
    window = config.distance_identity_max_age_s

    def phases():
        yield (
            Phase("A  lock not deployed, box present", deployed=False,
                  person_frames=30),
            "moves",
        )
        yield (
            Phase("B  deployed, nobody said follow me", person_frames=30,
                  request_enroll=False),
            "never",
        )
        yield (
            Phase("C  deployed, nobody in frame yet", person_frames=0),
            "never",
        )
        yield (
            Phase("D  deployed, box present, frame 1", person_frames=1,
                  enroll_samples=5),
            "never",
        )
        yield (Phase("E  enrolled, person in frame", person_frames=30), "moves")
        yield (
            Phase("F  enrolled, person leaves the frame", person_frames=6,
                  frame_count=6, enroll_samples=5),
            "bounded",
        )
        yield (
            Phase("G  enrolment failed, box present", person_frames=30,
                  enroll_samples=1, reid=ZeroReid()),
            "never",
        )
        yield (
            Phase("H  enrolled, camera stops entirely", person_frames=25,
                  frame_count=25, enroll_samples=5),
            "bounded",
        )
        yield (
            Phase("I  deployed but no OSNet behind it", person_frames=30,
                  lock_state=False),
            "never",
        )

    failures = []
    for phase, expectation in phases():
        vx_values, _sources, status = phase.run(config)
        moved = [index for index, vx in enumerate(vx_values) if vx != 0.0]
        if expectation == "never":
            ok = not moved
        elif expectation == "moves":
            ok = bool(moved)
        else:
            # Bounded: nothing past the last live frame plus one identity
            # window (plus the inference latency already in the timeline).
            deadline = int((phase.last_frame_at_s + window) * DISPATCH_HZ) + 2
            ok = not moved or max(moved) <= deadline
        if not ok:
            failures.append(phase.label)
        print(
            f"{'ok' if ok else 'FAIL':4} {phase.label:34} lock={status:14} "
            f"vx!=0 at {len(moved)}/{len(vx_values)} dispatches"
            + (f", last={max(moved)}" if moved else "")
            + ("" if expectation == "never" else "  (allowed)")
        )

    print()
    print(
        f"identity window = {window}s, camera {FRAME_HZ:.0f} Hz, "
        f"dispatch {DISPATCH_HZ:.0f} Hz, OSNet latency {INFERENCE_LATENCY_S}s"
    )
    if failures:
        print("FAILED:", ", ".join(failures))
        return 1
    print("OK: vx is non-zero only while the enrolled person is in frame.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
