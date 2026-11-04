"""Offline tests for the specified-person lock (ReID gate).

No torch, CUDA, camera, or ROS involvement: the OSNet model is replaced by a
stub whose feature is a one-hot keyed on ``bbox.x``, so cosine similarity is
fully controlled by the test.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

from go2_robottrack.person_reid import (
    PersonReID,
    ReidLockState,
    enroll_template,
)
from go2_robottrack.rgbd_distance import (
    BoundingBox,
    PersonDetection,
    RgbdDistanceConfig,
    RgbdPersonDistanceEstimator,
)

FEATURE_DIM = 512


def one_hot(key: int) -> np.ndarray:
    vector = np.zeros(FEATURE_DIM, dtype=np.float32)
    vector[key % FEATURE_DIM] = 1.0
    return vector


class StaticDetector:
    """Returns whatever boxes the test last assigned."""

    def __init__(self) -> None:
        self.boxes: list[PersonDetection] = []

    def detect(self, _bgr):
        return list(self.boxes)


class OneHotReid:
    """Feature lookup keyed on the bbox left edge."""

    feature_dim = FEATURE_DIM

    def extract(self, _bgr, bboxes):
        return [one_hot(box.x) for box in bboxes]


class ZeroFeatureReid:
    """OSNet that cannot score the crop: every feature comes back all-zero."""

    feature_dim = FEATURE_DIM

    def extract(self, _bgr, bboxes):
        return [np.zeros(FEATURE_DIM, dtype=np.float32) for _ in bboxes]


class GradedReid:
    """Features whose cosine to the enrolled one is set per box top edge.

    Lets a test give two candidates *different* similarities above the
    threshold, which a one-hot feature cannot do.
    """

    feature_dim = FEATURE_DIM

    def __init__(self, similarity_by_y: dict[int, float]) -> None:
        self._by_y = similarity_by_y

    def extract(self, _bgr, bboxes):
        features = []
        for box in bboxes:
            cosine = float(self._by_y.get(box.y, 0.0))
            vector = np.zeros(FEATURE_DIM, dtype=np.float32)
            vector[0] = cosine
            vector[1] = math.sqrt(max(0.0, 1.0 - cosine * cosine))
            features.append(vector)
        return features


class CosineSimilarityTests(unittest.TestCase):
    def test_identical_vectors_are_one(self) -> None:
        vector = one_hot(3)
        self.assertAlmostEqual(
            PersonReID.cosine_similarity(vector, vector), 1.0, places=6
        )

    def test_orthogonal_vectors_are_zero(self) -> None:
        self.assertAlmostEqual(
            PersonReID.cosine_similarity(one_hot(3), one_hot(9)), 0.0, places=6
        )

    def test_zero_vector_is_not_a_match(self) -> None:
        self.assertEqual(
            PersonReID.cosine_similarity(one_hot(3), np.zeros(FEATURE_DIM)), 0.0
        )


class EnrollTemplateTests(unittest.TestCase):
    def test_average_is_l2_normalized(self) -> None:
        template = enroll_template([one_hot(3), one_hot(3)])
        self.assertAlmostEqual(float(np.linalg.norm(template)), 1.0, places=6)

    def test_empty_input_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            enroll_template([])


class ReidLockStateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.lock = ReidLockState(threshold=0.70, enroll_samples=2)
        self.lock.configure(available=True, threshold=0.70, enroll_samples=2)

    def test_starts_unlocked_and_available(self) -> None:
        snapshot = self.lock.snapshot()
        self.assertTrue(snapshot.available)
        self.assertFalse(snapshot.active)
        self.assertFalse(snapshot.enrolled)
        self.assertEqual(snapshot.status, "unlocked")

    def test_enroll_activates_only_after_all_samples(self) -> None:
        self.lock.request_enroll()
        self.assertEqual(self.lock.snapshot().status, "enrolling")
        self.lock.add_enroll_feature(one_hot(3))
        self.assertFalse(self.lock.is_active())
        self.lock.add_enroll_feature(one_hot(3))
        self.assertTrue(self.lock.is_active())
        self.assertEqual(self.lock.snapshot().status, "locked")

    def test_enroll_without_any_valid_crop_fails(self) -> None:
        self.lock.request_enroll()
        self.lock.add_enroll_feature(np.zeros(FEATURE_DIM, dtype=np.float32))
        self.lock.add_enroll_feature(np.zeros(FEATURE_DIM, dtype=np.float32))
        snapshot = self.lock.snapshot()
        self.assertEqual(snapshot.status, "enroll_failed")
        self.assertFalse(snapshot.active)

    def test_filter_keeps_only_matching_features(self) -> None:
        self.lock.request_enroll()
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        mask, best = self.lock.filter_features([one_hot(3), one_hot(9)])
        self.assertEqual(mask, [True, False])
        self.assertAlmostEqual(best, 1.0, places=6)

    def test_filter_is_permissive_while_unlocked(self) -> None:
        mask, best = self.lock.filter_features([one_hot(3), one_hot(9)])
        self.assertEqual(mask, [True, True])
        self.assertIsNone(best)

    def test_needs_features_only_while_enrolling_or_locked(self) -> None:
        self.assertFalse(self.lock.needs_features())
        self.lock.request_enroll()
        self.assertTrue(self.lock.needs_features())

    def test_clear_returns_to_unlocked(self) -> None:
        self.lock.request_enroll()
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        self.assertTrue(self.lock.is_active())
        self.lock.clear()
        snapshot = self.lock.snapshot()
        self.assertFalse(snapshot.active)
        self.assertFalse(snapshot.enrolled)
        self.assertEqual(snapshot.status, "unlocked")

    def test_engagement_spans_the_command_and_outlives_a_failed_enrolment(self) -> None:
        # "Engaged" means: the operator asked for specified follow and has not
        # cancelled it.  It has to survive a failed enrolment -- otherwise the
        # robot silently falls back to following anybody, which is exactly the
        # hole the command was meant to close.
        self.assertFalse(self.lock.is_engaged())
        self.lock.request_enroll()
        self.assertTrue(self.lock.snapshot().engaged)
        self.lock.add_enroll_feature(np.zeros(FEATURE_DIM, dtype=np.float32))
        self.lock.add_enroll_feature(np.zeros(FEATURE_DIM, dtype=np.float32))
        self.assertEqual(self.lock.snapshot().status, "enroll_failed")
        self.assertTrue(self.lock.is_engaged())
        self.assertFalse(self.lock.is_active())
        self.lock.clear()
        self.assertFalse(self.lock.is_engaged())

    def test_unavailable_lock_reports_itself(self) -> None:
        self.lock.configure(available=False)
        self.assertFalse(self.lock.available)
        self.assertEqual(self.lock.snapshot().status, "unavailable")

    def test_threshold_and_samples_are_validated(self) -> None:
        with self.assertRaises(ValueError):
            self.lock.configure(threshold=0.0)
        with self.assertRaises(ValueError):
            self.lock.configure(enroll_samples=0)

    def test_enroll_ignores_a_sample_of_somebody_else(self) -> None:
        # The template must be one person.  Averaging a second person in used to
        # make it match both of them, which is what let the robot follow a
        # stranger.
        self.lock.request_enroll()
        self.assertTrue(self.lock.add_enroll_feature(one_hot(3)))
        self.assertFalse(self.lock.add_enroll_feature(one_hot(9)))
        snapshot = self.lock.snapshot()
        self.assertIn("somebody else", snapshot.detail)
        # The rejected sample did not count down: still one sample short.
        self.assertEqual(snapshot.pending_enroll, 1)
        self.assertFalse(snapshot.active)
        self.assertTrue(self.lock.add_enroll_feature(one_hot(3)))
        mask, _best = self.lock.filter_features([one_hot(3), one_hot(9)])
        self.assertEqual(mask, [True, False])

    def test_match_features_grades_every_detection(self) -> None:
        self.lock.request_enroll()
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        similarities, best = self.lock.match_features([one_hot(3), one_hot(9)])
        self.assertAlmostEqual(similarities[0], 1.0, places=6)
        self.assertAlmostEqual(similarities[1], 0.0, places=6)
        self.assertAlmostEqual(best, 1.0, places=6)

    def test_match_features_is_neutral_while_unlocked(self) -> None:
        similarities, best = self.lock.match_features([one_hot(3)])
        self.assertEqual(similarities, [None])
        self.assertIsNone(best)

    def test_unscorable_detection_scores_none(self) -> None:
        self.lock.request_enroll()
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        similarities, _best = self.lock.match_features(
            [np.zeros(FEATURE_DIM, dtype=np.float32), one_hot(3)]
        )
        self.assertIsNone(similarities[0])
        self.assertAlmostEqual(similarities[1], 1.0, places=6)


class EnrollWaitPromptTests(unittest.TestCase):
    """An enrolment with nobody in view waits and keeps saying so."""

    def setUp(self) -> None:
        self.lock = ReidLockState(threshold=0.70, enroll_samples=2)
        self.lock.configure(available=True, threshold=0.70, enroll_samples=2)

    def test_first_stall_prompts_at_once_then_throttles(self) -> None:
        self.lock.request_enroll()
        first = self.lock.note_enroll_wait(now=100.0)
        self.assertIsNotNone(first)
        self.assertIn("no person in view", first)
        # Inside the interval: silent, so a 30 fps loop cannot flood.
        self.assertIsNone(self.lock.note_enroll_wait(now=100.5))
        self.assertIsNone(self.lock.note_enroll_wait(now=101.9))
        second = self.lock.note_enroll_wait(now=102.0)
        self.assertIsNotNone(second)
        snapshot = self.lock.snapshot()
        self.assertEqual(snapshot.prompts, 2)
        self.assertEqual(snapshot.status, "enrolling_no_person")
        self.assertIn("prompt #2", snapshot.detail)

    def test_waiting_never_expires_the_enrolment(self) -> None:
        self.lock.request_enroll()
        # A full minute of empty frames: still pending, never "failed".
        for tenth in range(600):
            self.lock.note_enroll_wait(now=200.0 + tenth * 0.1)
        snapshot = self.lock.snapshot()
        self.assertEqual(snapshot.pending_enroll, 2)
        self.assertNotEqual(snapshot.status, "enroll_failed")
        self.assertFalse(snapshot.active)

    def test_prompt_is_silent_when_nothing_is_enrolling(self) -> None:
        self.assertIsNone(self.lock.note_enroll_wait(now=100.0))
        self.assertEqual(self.lock.snapshot().status, "unlocked")
        self.assertEqual(self.lock.snapshot().prompts, 0)

    def test_prompts_are_rate_limited_even_when_detections_flicker(self) -> None:
        # A person who appears and vanishes between frames must not turn the
        # wait announcement into a burst: the interval is a hard ceiling.
        self.lock.request_enroll()
        self.assertIsNotNone(self.lock.note_enroll_wait(now=100.0))
        self.lock.add_enroll_feature(one_hot(3))
        self.assertIsNone(self.lock.note_enroll_wait(now=100.1))
        self.assertIsNone(self.lock.note_enroll_wait(now=101.9))
        self.assertIsNotNone(self.lock.note_enroll_wait(now=102.0))

    def test_waiting_time_is_reported_only_while_enrolling(self) -> None:
        self.assertEqual(self.lock.snapshot().waiting_s, 0.0)
        self.lock.request_enroll()
        self.assertGreaterEqual(self.lock.snapshot().waiting_s, 0.0)
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        self.assertTrue(self.lock.is_active())
        self.assertEqual(self.lock.snapshot().waiting_s, 0.0)
        self.assertEqual(self.lock.snapshot().prompts, 0)

    def test_an_appearing_person_ends_the_wait_and_locks(self) -> None:
        self.lock.request_enroll()
        self.lock.note_enroll_wait(now=100.0)
        self.lock.add_enroll_feature(one_hot(3))
        self.lock.add_enroll_feature(one_hot(3))
        snapshot = self.lock.snapshot()
        self.assertEqual(snapshot.status, "locked")
        self.assertTrue(snapshot.active)
        self.assertEqual(snapshot.pending_enroll, 0)

    def test_clear_drops_the_wait_state(self) -> None:
        self.lock.request_enroll()
        self.lock.note_enroll_wait(now=100.0)
        snapshot = self.lock.clear()
        self.assertEqual(snapshot.status, "unlocked")
        self.assertEqual(snapshot.prompts, 0)
        self.assertEqual(snapshot.waiting_s, 0.0)
        self.assertIsNone(self.lock.note_enroll_wait(now=100.1))


class LockGateTests(unittest.TestCase):
    """The gate must stop the follow loop from ever switching person."""

    PERSON_A = BoundingBox(10, 100, 200, 400)  # nearest -> enroll candidate
    PERSON_B = BoundingBox(300, 100, 60, 120)  # the stranger

    def _estimator(self, lock: ReidLockState | None) -> tuple[
        RgbdPersonDistanceEstimator, StaticDetector
    ]:
        detector = StaticDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=lock is not None,
            ),
            reid=OneHotReid() if lock is not None else None,
            reid_state=lock,
        )
        return estimator, detector

    def _locked(self) -> tuple[RgbdPersonDistanceEstimator, StaticDetector, ReidLockState]:
        lock = ReidLockState(threshold=0.70, enroll_samples=1)
        lock.configure(available=True, threshold=0.70, enroll_samples=1)
        estimator, detector = self._estimator(lock)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        # Enroll while both people are visible: the *larger* box wins, which is
        # the person standing nearest and therefore issuing the command.
        detector.boxes = [
            PersonDetection(self.PERSON_B, 0.90),
            PersonDetection(self.PERSON_A, 0.50),
        ]
        lock.request_enroll()
        estimator._select_target(image, detector.detect(image), 640, 480)
        self.assertEqual(lock.snapshot().status, "locked")
        return estimator, detector, lock

    def test_enroll_picks_the_largest_box(self) -> None:
        estimator, detector, _lock = self._locked()
        self.assertEqual(estimator._previous_bbox, self.PERSON_A.clipped(640, 480))

    def test_stranger_alone_is_never_followed(self) -> None:
        estimator, detector, _lock = self._locked()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.PERSON_B, 0.99)]
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-no-match")
        self.assertEqual(confidence, 0.0)

    def test_higher_confidence_stranger_loses_to_the_enrolled_person(self) -> None:
        estimator, detector, _lock = self._locked()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        # B outranks A on HOG confidence by a wide margin and is still ignored.
        detector.boxes = [
            PersonDetection(self.PERSON_B, 0.99),
            PersonDetection(self.PERSON_A, 0.10),
        ]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(bbox, self.PERSON_A.clipped(640, 480))
        self.assertEqual(source, "reid")

    def test_locked_with_no_detections_reports_no_match(self) -> None:
        estimator, detector, _lock = self._locked()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = []
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-no-match")

    def test_unlocking_does_not_reopen_plain_association(self) -> None:
        # "Stop following me" forgets the person; it does not hand the robot
        # back to the pre-existing "follow whoever is in frame" loop.  In a
        # deployment with the lock on there is no enrolled person afterwards,
        # so there is nobody the robot is allowed to follow and it holds vx=0.
        estimator, detector, lock = self._locked()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.clear()
        detector.boxes = [PersonDetection(self.PERSON_B, 0.99)]
        sources = []
        for _ in range(5):
            bbox, _confidence, source = estimator._select_target(
                image, detector.detect(image), 640, 480
            )
            self.assertIsNone(bbox)
            sources.append(source)
        self.assertEqual(set(sources), {"reid-inactive"})

    def test_without_reid_the_plain_loop_is_byte_for_byte_unchanged(self) -> None:
        # The way back to ordinary following is the deployment switch, not the
        # unlock command: with reid_enabled false the pre-existing loop runs
        # exactly as it did before this feature existed.
        detector = StaticDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=False,
            ),
            reid=None,
            reid_state=None,
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.PERSON_B, 0.99)]
        _bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(source, "hog")

    def test_estimator_without_lock_is_unchanged(self) -> None:
        estimator, detector = self._estimator(None)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.PERSON_B, 0.99)]
        _bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(source, "hog")


class EnrollFirstPersonTests(unittest.TestCase):
    """Enrolment takes the first person seen and nobody else, ever."""

    PERSON_A = BoundingBox(10, 100, 200, 400)  # first person seen
    PERSON_B = BoundingBox(300, 100, 60, 120)  # walked in later

    def _setup(self, enroll_samples: int = 2):
        detector = StaticDetector()
        lock = ReidLockState(threshold=0.70, enroll_samples=enroll_samples)
        lock.configure(available=True, threshold=0.70, enroll_samples=enroll_samples)
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=True,
            ),
            reid=OneHotReid(),
            reid_state=lock,
        )
        return estimator, detector, lock

    def test_a_second_person_is_never_averaged_into_the_template(self) -> None:
        estimator, detector, lock = self._setup()
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        def frame(boxes):
            detector.boxes = [PersonDetection(*item) for item in boxes]
            return estimator._select_target(image, detector.detect(image), 640, 480)

        lock.request_enroll()
        # Frame 1: both visible, the nearest (largest) box is the operator.
        frame([(self.PERSON_B, 0.90), (self.PERSON_A, 0.50)])
        self.assertEqual(lock.snapshot().pending_enroll, 1)
        # Frame 2: only the stranger is left.  He is not enrolled, and the
        # frame does not count down.
        frame([(self.PERSON_B, 0.99)])
        self.assertEqual(lock.snapshot().pending_enroll, 1)
        self.assertIn("not in view", lock.snapshot().detail)
        # Frame 3: the operator is back, enrolment completes on him alone.
        frame([(self.PERSON_B, 0.99), (self.PERSON_A, 0.50)])
        self.assertTrue(lock.is_active())
        similarities, _best = lock.match_features(
            [one_hot(self.PERSON_A.x), one_hot(self.PERSON_B.x)]
        )
        self.assertAlmostEqual(similarities[0], 1.0, places=6)
        self.assertAlmostEqual(similarities[1], 0.0, places=6)

    def test_a_stranger_alone_after_enrolment_is_blocked(self) -> None:
        estimator, detector, lock = self._setup(enroll_samples=1)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = [PersonDetection(self.PERSON_A, 0.50)]
        estimator._select_target(image, detector.detect(image), 640, 480)
        self.assertTrue(lock.is_active())
        detector.boxes = [PersonDetection(self.PERSON_B, 0.99)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-no-match")


class IdentityBeatsPositionTests(unittest.TestCase):
    """Who is followed is decided by appearance, not by where the last box was."""

    # Similarity to the enrolled template, keyed on the box top edge: the
    # person is identified by appearance, the position is free to differ.
    TARGET_START = BoundingBox(100, 100, 200, 400)  # 1.00
    TARGET_MOVED = BoundingBox(420, 200, 100, 200)  # 0.90
    STRANGER = BoundingBox(100, 300, 200, 400)  # 0.80

    def _setup(self, similarity_by_y: dict[int, float], threshold: float = 0.70):
        detector = StaticDetector()
        lock = ReidLockState(threshold=threshold, enroll_samples=1)
        lock.configure(available=True, threshold=threshold, enroll_samples=1)
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=True,
            ),
            reid=GradedReid(similarity_by_y),
            reid_state=lock,
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = [PersonDetection(self.TARGET_START, 0.50)]
        estimator._select_target(image, detector.detect(image), 640, 480)
        self.assertTrue(lock.is_active())
        return estimator, detector, image

    def test_the_best_match_wins_over_the_box_that_continues_the_previous_one(
        self,
    ) -> None:
        estimator, detector, image = self._setup(
            {100: 1.0, 200: 0.90, 300: 0.80}
        )
        # The stranger stands exactly where the target was (perfect overlap with
        # the previous box) and has twice the HOG confidence.  The target has
        # moved.  The old rule picked whoever continued the previous box, so it
        # would have followed the stranger.
        detector.boxes = [
            PersonDetection(self.STRANGER, 0.99),
            PersonDetection(self.TARGET_MOVED, 0.10),
        ]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(bbox, self.TARGET_MOVED.clipped(640, 480))
        self.assertEqual(source, "reid")

    def test_a_box_in_the_targets_place_below_the_threshold_is_not_followed(
        self,
    ) -> None:
        estimator, detector, image = self._setup({100: 1.0, 300: 0.55})
        # Same position as the target's last box, but only 0.55 similar: not the
        # enrolled person, so nobody is followed and the robot stops.
        detector.boxes = [PersonDetection(self.STRANGER, 0.99)]
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(confidence, 0.0)
        self.assertEqual(source, "reid-no-match")


class LockedBridgeTests(unittest.TestCase):
    """The optical-flow bridge follows a box, so the lock must switch it off."""

    def _run(self, lock: ReidLockState | None) -> list[str]:
        random = np.random.RandomState(7)
        texture = random.randint(0, 256, (100, 80, 3), dtype=np.uint8)

        def frame(offset: int) -> np.ndarray:
            canvas = np.zeros((120, 160, 3), dtype=np.uint8)
            canvas[10:110, 40 + offset : 120 + offset] = texture
            return canvas

        detector = StaticDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                tracking_max_frames=3,
                reid_enabled=lock is not None,
            ),
            reid=OneHotReid() if lock is not None else None,
            reid_state=lock,
        )
        depth = np.full((120, 160), 3000, dtype=np.uint16)
        if lock is not None:
            lock.request_enroll()
        detector.boxes = [PersonDetection(BoundingBox(40, 10, 80, 100), 0.90)]
        sources = [
            estimator.estimate(
                frame(offset), depth, frame_timestamp_s=1.0, now_s=1.0
            ).source
            for offset in (0,)
        ]
        # The person vanishes: HOG reports nobody for the next two frames.
        detector.boxes = []
        for step, offset in enumerate((4, 8), start=1):
            sources.append(
                estimator.estimate(
                    frame(offset),
                    depth,
                    frame_timestamp_s=1.0 + 0.2 * step,
                    now_s=1.0 + 0.2 * step,
                ).source
            )
        return sources

    def test_the_bridge_bridges_a_hog_miss_without_a_lock(self) -> None:
        sources = self._run(None)
        self.assertEqual(sources, ["hog", "short-track", "short-track"])

    def test_the_bridge_is_switched_off_while_locked(self) -> None:
        lock = ReidLockState(threshold=0.70, enroll_samples=1)
        lock.configure(available=True, threshold=0.70, enroll_samples=1)
        sources = self._run(lock)
        # Frames 2 and 3: nobody in view is the enrolled person, so no target is
        # produced at all -- the bridge would have carried the box (and the
        # robot) onto whoever appeared in its place.
        self.assertEqual(sources, ["reid", "reid-no-match", "reid-no-match"])
        # The lock itself is untouched by the misses: only the target stopped
        # being reported, and it is still enrolled.
        self.assertTrue(lock.is_active())


class UnverifiedIdentityBlocksVxTests(unittest.TestCase):
    """No target may exist between "follow me" and a verified identity.

    The requirement is one line: forward motion only while the enrolled person
    is in frame.  Everything in between -- waiting for that person to appear,
    an enrolment that failed on an unusable crop -- has no identity to verify
    against, so the estimator must report no target whatever the person
    detector says.  A HOG false positive on furniture looks exactly like a
    person standing there, which is why the gate cannot live in the detector.
    """

    # Large enough for OSNet's crop and centred like a person would be: the
    # shape a wall or a chair has to produce for HOG to call it a person.
    FALSE_POSITIVE = BoundingBox(10, 100, 200, 400)

    def _setup(
        self, *, enroll_samples: int = 2, reid=None
    ) -> tuple[RgbdPersonDistanceEstimator, StaticDetector, ReidLockState]:
        detector = StaticDetector()
        lock = ReidLockState(threshold=0.70, enroll_samples=enroll_samples)
        lock.configure(available=True, threshold=0.70, enroll_samples=enroll_samples)
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=True,
            ),
            reid=reid if reid is not None else OneHotReid(),
            reid_state=lock,
        )
        return estimator, detector, lock

    def test_waiting_for_the_person_produces_no_target(self) -> None:
        estimator, detector, lock = self._setup()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = []
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(confidence, 0.0)
        self.assertEqual(source, "reid-unverified")

    def test_waiting_does_not_follow_whatever_the_detector_reports(self) -> None:
        # Frame 1 of the enrolment: a box is there, but the identity is not
        # settled yet.  Nothing may move on it.
        estimator, detector, lock = self._setup()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-unverified")

    def test_a_failed_enrolment_keeps_the_robot_still(self) -> None:
        # OSNet could not score the crop at all, so enrolment fails.  The lock
        # is engaged but has no template: no frame may produce a target, and
        # ordinary following must not quietly take over.
        estimator, detector, lock = self._setup(
            enroll_samples=1, reid=ZeroFeatureReid()
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        estimator._select_target(image, detector.detect(image), 640, 480)
        self.assertEqual(lock.snapshot().status, "enroll_failed")

        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(confidence, 0.0)
        self.assertEqual(source, "reid-unverified")

    def test_cancelling_the_command_stops_the_follow_loop(self) -> None:
        # After "stop following me" the robot holds: a person-shaped box in
        # frame is not a reason to move, because the deployment requires a
        # verified identity and there is none any more.
        estimator, detector, lock = self._setup()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        lock.clear()
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(confidence, 0.0)
        self.assertEqual(source, "reid-inactive")

    def test_nobody_has_asked_yet_produces_no_target(self) -> None:
        # The deployment has the lock on; until someone says "follow me" there
        # is no "me", so the pre-existing loop must not command forward motion
        # on whatever the detector reports.
        estimator, detector, lock = self._setup()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-inactive")

    def test_a_lock_with_no_model_behind_it_produces_no_target(self) -> None:
        # reid_enabled true but OSNet failed to load: no identity can ever be
        # verified, so the robot must stand still rather than silently fall
        # back to following whoever is in frame.
        detector = StaticDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                reid_enabled=True,
            ),
            reid=None,
            reid_state=None,
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(confidence, 0.0)
        self.assertEqual(source, "reid-unavailable")

    def test_an_enrolled_person_in_frame_is_still_followed(self) -> None:
        # The gate closes the window before verification; it must not block the
        # verified case itself.
        estimator, detector, lock = self._setup(enroll_samples=1)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        lock.request_enroll()
        detector.boxes = [PersonDetection(self.FALSE_POSITIVE, 0.99)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(bbox, self.FALSE_POSITIVE.clipped(640, 480))
        self.assertEqual(source, "reid")


class ConfigTests(unittest.TestCase):
    def test_reid_without_distance_disables_the_lock_instead_of_failing(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        # A typo in the new feature's configuration must not take the
        # pre-existing follow stack down with it: the lock switches itself off
        # and says so, and ordinary following keeps running.
        with self.assertLogs("go2_robottrack.core", level="WARNING"):
            config = RuntimeConfig.from_mapping({"reid_enabled": True})
        self.assertFalse(config.reid_enabled)
        self.assertFalse(config.distance_enabled)

    def test_reid_fields_round_trip(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        config = RuntimeConfig.from_mapping(
            {
                "distance_enabled": True,
                "reid_enabled": True,
                "reid_model_dir": "/root/deploy/models/osnet",
                "reid_match_threshold": 0.66,
                "reid_enroll_samples": 9,
            }
        )
        self.assertTrue(config.reid_enabled)
        self.assertEqual(config.reid_model_dir, "/root/deploy/models/osnet")
        self.assertAlmostEqual(config.reid_match_threshold, 0.66)
        self.assertEqual(config.reid_enroll_samples, 9)

    def test_reid_defaults_are_off(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        config = RuntimeConfig.from_mapping({})
        self.assertFalse(config.reid_enabled)


class DetectorConfidenceFloorTests(unittest.TestCase):
    """Weak detections must never become candidates -- for enrolment or matching.

    HOG's ``hitThreshold`` is 0.0, so every box it returns is a weakly-positive
    window and its confidence is above 0.5 by construction.  On background
    clutter those are furniture and wall edges.  The gate scores *every* box
    with OSNet and one frame over the threshold yields a target, so a single
    junk box that resembles the template walks the robot off after nobody.
    """

    WEAK = BoundingBox(10, 100, 200, 400)
    STRONG = BoundingBox(300, 100, 200, 400)

    def _estimator(self, floor: float, lock: ReidLockState | None = None):
        detector = StaticDetector()
        estimator = RgbdPersonDistanceEstimator(
            detector=detector,
            config=RgbdDistanceConfig(
                detection_confirmation_frames=1,
                enable_short_term_tracking=False,
                detector_min_confidence=floor,
                reid_enabled=lock is not None,
            ),
            reid=OneHotReid() if lock is not None else None,
            reid_state=lock,
        )
        return estimator, detector

    def test_floor_zero_keeps_every_detection(self) -> None:
        # The pre-existing behaviour: with no floor a weak box is still a
        # candidate, so turning the filter on is the only thing that changes.
        estimator, detector = self._estimator(floor=0.0)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.WEAK, 0.10)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(bbox, self.WEAK.clipped(640, 480))
        self.assertEqual(source, "hog")

    def test_weak_box_is_not_a_candidate(self) -> None:
        estimator, detector = self._estimator(floor=0.60)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.WEAK, 0.55)]
        bbox, _confidence, _source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)

    def test_floor_keeps_the_strong_box_beside_a_weak_one(self) -> None:
        estimator, detector = self._estimator(floor=0.60)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [
            PersonDetection(self.WEAK, 0.20),
            PersonDetection(self.STRONG, 0.95),
        ]
        bbox, confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertEqual(bbox, self.STRONG.clipped(640, 480))
        self.assertAlmostEqual(confidence, 0.95)
        self.assertEqual(source, "hog")

    def test_weak_box_cannot_be_enrolled(self) -> None:
        # Otherwise "follow me" enrols a chair: the anchor is the largest box,
        # and a big junk box is exactly what the floor exists to reject.
        lock = ReidLockState(threshold=0.70, enroll_samples=1)
        lock.configure(available=True, threshold=0.70, enroll_samples=1)
        estimator, detector = self._estimator(floor=0.60, lock=lock)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.WEAK, 0.30)]
        lock.request_enroll()
        _bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(estimator._enroll_anchor_bbox)
        self.assertEqual(lock.snapshot().status, "enrolling_no_person")
        self.assertEqual(source, "reid-unverified")

    def test_weak_box_cannot_match_a_locked_template(self) -> None:
        lock = ReidLockState(threshold=0.70, enroll_samples=1)
        lock.configure(available=True, threshold=0.70, enroll_samples=1)
        estimator, detector = self._estimator(floor=0.60, lock=lock)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        detector.boxes = [PersonDetection(self.STRONG, 0.90)]
        lock.request_enroll()
        estimator._select_target(image, detector.detect(image), 640, 480)
        self.assertEqual(lock.snapshot().status, "locked")
        # The same box, now too weak to be a candidate at all: the enrolled
        # person cannot be confirmed and nothing may move.
        detector.boxes = [PersonDetection(self.STRONG, 0.10)]
        bbox, _confidence, source = estimator._select_target(
            image, detector.detect(image), 640, 480
        )
        self.assertIsNone(bbox)
        self.assertEqual(source, "reid-no-match")


class DetectorFloorConfigTests(unittest.TestCase):
    def test_floor_defaults_to_off(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        self.assertEqual(RuntimeConfig.from_mapping({}).detector_min_confidence, 0.0)

    def test_floor_is_read_from_the_deployment(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        config = RuntimeConfig.from_mapping({"detector_min_confidence": 0.6})
        self.assertAlmostEqual(config.detector_min_confidence, 0.6)

    def test_floor_outside_the_unit_interval_is_rejected(self) -> None:
        from go2_robottrack.core import RuntimeConfig

        with self.assertRaises(ValueError):
            RuntimeConfig.from_mapping({"detector_min_confidence": 1.5})


if __name__ == "__main__":
    unittest.main()
