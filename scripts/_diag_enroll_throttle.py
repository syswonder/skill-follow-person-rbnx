"""Reproduce the enrolment-wait prompt cadence, and the path that terminates.

Ways a frame can fail to yield a *usable* enrolment sample, all run through the
real estimator at ~30 Hz for 6 s:

  empty    -- HOG sees nobody            -> waits: prompts once per 2 s
  short    -- HOG sees a box too short   -> waits: prompts once per 2 s
  zerofeat -- a tall box, all-zero feature -> NOT a wait: the candidate is
              found, ``add_enroll_feature`` counts it down, and enrolment
              correctly fails after ``enroll_samples`` frames (prompts=0).

Measured: empty/short give 3 prompts in 6 s with ~2.0 s gaps; zerofeat gives
``enroll_failed`` with no prompts at all.  A burst (or an ``empty``/``short``
run that stops prompting) means the throttle window is being reopened.
"""

import time

import numpy as np

from go2_robottrack.person_reid import ReidLockState
from go2_robottrack.rgbd_distance import (
    BoundingBox,
    PersonDetection,
    RgbdDistanceConfig,
    RgbdPersonDistanceEstimator,
)


class BoxDetector:
    def __init__(self, boxes):
        self._boxes = boxes

    def detect(self, _bgr):
        return list(self._boxes)


class ZeroReid:
    feature_dim = 512

    def extract(self, _bgr, bboxes):
        return [np.zeros(512, dtype=np.float32) for _ in bboxes]


def run(label, boxes):
    lock = ReidLockState(threshold=0.70, enroll_samples=5)
    lock.configure(available=True, threshold=0.70, enroll_samples=5)
    estimator = RgbdPersonDistanceEstimator(
        detector=BoxDetector(boxes),
        config=RgbdDistanceConfig(
            detection_confirmation_frames=1,
            enable_short_term_tracking=False,
            reid_enabled=True,
        ),
        reid=ZeroReid(),
        reid_state=lock,
    )
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    lock.request_enroll()

    stamps = []
    started = time.monotonic()
    while time.monotonic() - started < 6.0:
        before = lock.snapshot().prompts
        estimator._select_target(image, list(boxes), 640, 480)
        if lock.snapshot().prompts > before:
            stamps.append(time.monotonic() - started)
        time.sleep(1.0 / 30.0)

    gaps = [round(b - a, 3) for a, b in zip(stamps, stamps[1:])]
    snap = lock.snapshot()
    print(
        f"{label:9s} prompts={snap.prompts}  status={snap.status:20s} "
        f"pending={snap.pending_enroll}  gaps={gaps}"
    )


print("expect ~3 prompts each, gaps all ~2.0 s")
run("empty", [])
run("short", [PersonDetection(BoundingBox(0, 0, 10, 10), 0.9)])
run("zerofeat", [PersonDetection(BoundingBox(10, 100, 200, 400), 0.9)])
