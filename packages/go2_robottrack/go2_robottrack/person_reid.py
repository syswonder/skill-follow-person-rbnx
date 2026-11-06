"""Person Re-Identification via OSNet.

Loads OSNet x1.0 model (pre-trained on Market1501) and extracts 512-dim feature
vectors from person bounding boxes. Used for "specified person follow": after
voice command "跟着我", the current person's appearance is enrolled (stored as a
template); subsequent frames only follow people matching this template.

Usage:
    reid = PersonReID(model_dir="/root/deploy/models/osnet")
    features = reid.extract(image_bgr, [bbox1, bbox2, ...])
    # features: list of 512-dim numpy arrays (one per bbox)
"""

from __future__ import annotations

import sys
import logging
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

logger = logging.getLogger(__name__)

# OSNet expects 256x128 RGB, normalized with ImageNet stats.
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
_INPUT_SIZE = (256, 128)  # (height, width)

# How often the lock re-announces that it is still waiting for someone to
# enrol.  Enrolment itself never expires: the operator asked to follow a person
# and the lock keeps waiting, and keeps saying so, until one appears.
_ENROLL_PROMPT_INTERVAL_S = 2.0

# A new enrolment sample must look like the samples already collected.  OSNet
# keeps one person's features well above this across pose and lighting, and
# puts a different person far below it.  Samples under the floor are dropped
# instead of averaged in, so the template can never become the average of two
# people -- the blend that used to match a stranger as strongly as the target.
_ENROLL_CONSISTENCY_FLOOR = 0.50


class BoundingBoxLike(Protocol):
    """Anything with pixel-space ``x, y, width, height`` (rgbd_distance.BoundingBox)."""

    x: int
    y: int
    width: int
    height: int


class PersonReID:
    """OSNet-based person re-identification.

    Attributes:
        feature_dim: Output feature dimension (512 for OSNet x1.0).
        device: Torch device string (e.g., "cuda:0").
    """

    def __init__(
        self,
        model_dir: str | Path,
        *,
        num_classes: int = 751,
        device: str = "cuda:0",
        use_fp16: bool = True,
    ) -> None:
        """Load OSNet model from ``model_dir``.

        Args:
            model_dir: Directory containing ``osnet.py`` and
                ``osnet_x1_0_market1501_softmax.pth``.
            num_classes: Number of training classes (751 for Market1501).
            device: Torch device to load model onto.
            use_fp16: Whether to use FP16 inference (faster on Orin).
        """
        import torch

        self._torch = torch
        self._device = device
        self._use_fp16 = use_fp16 and torch.cuda.is_available()
        self.feature_dim = 512

        model_dir = Path(model_dir)
        if not model_dir.is_dir():
            raise FileNotFoundError(f"Model directory not found: {model_dir}")

        # Add model_dir to sys.path so we can import osnet.py
        model_dir_str = str(model_dir)
        if model_dir_str not in sys.path:
            sys.path.insert(0, model_dir_str)

        from osnet import osnet_x1_0

        logger.info(f"Loading OSNet x1.0 from {model_dir} ...")
        self._model = osnet_x1_0(num_classes=num_classes, pretrained=False, loss="softmax")
        weights_path = model_dir / "osnet_x1_0_market1501_softmax.pth"
        if not weights_path.is_file():
            raise FileNotFoundError(f"Weights not found: {weights_path}")

        state_dict = torch.load(weights_path, map_location="cpu", weights_only=False)
        missing, unexpected = self._model.load_state_dict(state_dict, strict=False)
        if missing or unexpected:
            logger.warning(f"OSNet load: missing={len(missing)}, unexpected={len(unexpected)}")

        self._model.eval().to(device)
        if self._use_fp16:
            self._model = self._model.half()
            logger.info("OSNet loaded in FP16 mode")
        else:
            logger.info("OSNet loaded in FP32 mode")

        logger.info(f"OSNet ready on {device}, feature_dim={self.feature_dim}")

    def extract(
        self, image_bgr: np.ndarray, bboxes: Sequence[BoundingBoxLike]
    ) -> list[np.ndarray]:
        """Extract 512-dim features for each bbox in ``image_bgr``.

        Args:
            image_bgr: HxWx3 BGR image (OpenCV format).
            bboxes: List of bounding boxes to extract features from.

        Returns:
            List of 512-dim numpy arrays (float32), one per bbox. If a bbox is
            invalid (out of bounds, too small), the corresponding feature is
            all zeros.
        """
        import torch
        import cv2

        if not bboxes:
            return []

        crops = []
        valid_indices = []
        for i, bbox in enumerate(bboxes):
            # Clip bbox to image bounds
            x0 = max(0, min(image_bgr.shape[1], bbox.x))
            y0 = max(0, min(image_bgr.shape[0], bbox.y))
            x1 = max(0, min(image_bgr.shape[1], bbox.x + bbox.width))
            y1 = max(0, min(image_bgr.shape[0], bbox.y + bbox.height))
            if x1 <= x0 or y1 <= y0:
                continue
            crop_bgr = image_bgr[y0:y1, x0:x1]
            crops.append(crop_bgr)
            valid_indices.append(i)

        # Initialize all features to zeros
        features = [np.zeros(self.feature_dim, dtype=np.float32) for _ in bboxes]

        if not crops:
            return features

        # Preprocess: resize to 256x128, BGR→RGB, normalize
        tensor_list = []
        for crop_bgr in crops:
            # Resize to (width=128, height=256) → shape (256, 128, 3)
            resized = cv2.resize(crop_bgr, _INPUT_SIZE[::-1], interpolation=cv2.INTER_LINEAR)
            # BGR → RGB
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            # Normalize to [0, 1] then ImageNet stats
            rgb_float = rgb.astype(np.float32) / 255.0
            normalized = (rgb_float - _IMAGENET_MEAN) / _IMAGENET_STD
            # HWC → CHW
            chw = normalized.transpose(2, 0, 1)
            tensor_list.append(chw)

        # Stack into batch tensor: (N, 3, 256, 128)
        batch_np = np.stack(tensor_list, axis=0)
        batch_tensor = torch.from_numpy(batch_np).to(self._device)
        if self._use_fp16:
            batch_tensor = batch_tensor.half()

        # Forward pass
        with torch.no_grad():
            # OSNet.forward() returns classifier logits; we want the feature vector
            # before the classifier. Use the internal methods directly.
            feature_maps = self._model.featuremaps(batch_tensor)
            pooled = self._model.global_avgpool(feature_maps)
            # AdaptiveAvgPool2d keeps the spatial dims: (N, 512, 1, 1) -> (N, 512)
            features_tensor = pooled.flatten(1)

        # Move to CPU, convert to float32 numpy
        features_np = features_tensor.float().cpu().numpy()

        # Fill in valid features
        for i, feat_vec in zip(valid_indices, features_np):
            features[i] = feat_vec

        return features

    @staticmethod
    def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
        """Compute cosine similarity between two feature vectors."""
        dot = np.dot(a, b)
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a < 1e-8 or norm_b < 1e-8:
            return 0.0
        return float(dot / (norm_a * norm_b))


def enroll_template(features: list[np.ndarray]) -> np.ndarray:
    """Average multiple features into a single template vector.

    Args:
        features: List of 512-dim feature vectors (from multiple frames/bboxes).

    Returns:
        Averaged and L2-normalized template vector.
    """
    if not features:
        raise ValueError("Cannot enroll empty feature list")
    stacked = np.stack(features, axis=0)  # (N, 512)
    mean_vec = stacked.mean(axis=0)  # (512,)
    # L2 normalize
    norm = np.linalg.norm(mean_vec)
    if norm < 1e-8:
        return mean_vec
    return mean_vec / norm


@dataclass(frozen=True)
class ReidLockSnapshot:
    """Thread-safe view of the specified-person lock."""

    active: bool
    available: bool
    enrolled: bool
    pending_enroll: int
    threshold: float
    last_similarity: float | None
    candidates: int
    matches: int
    status: str
    detail: str
    revision: int
    # Seconds since ``lock`` was requested, and how many "still waiting"
    # prompts have been issued.  Non-zero ``waiting_s`` with a pending
    # enrolment means the lock is up and looking for a person.
    waiting_s: float = 0.0
    prompts: int = 0
    # True from "follow me" until the lock is cleared -- the operator has asked
    # for specified follow and no verified identity exists yet.
    engaged: bool = False


class ReidLockState:
    """Shared lock state between the provider thread and the estimator thread.

    Written by the Robonix provider thread (``request_enroll`` / ``clear`` via
    the ``follow_distance`` capability's ``lock`` / ``unlock`` operations) and
    read by the RGB-D worker thread (``match_features`` / ``add_enroll_feature``
    / ``note_enroll_wait``).  All access is guarded by one lock.

    The lock owns the *identity*, the estimator owns the *choice*: while
    ``active``, ``match_features`` tells the estimator how strongly each
    detection resembles the enrolled person and the estimator follows only the
    best one above ``threshold``.  A detection that is not the enrolled person
    is never a candidate, however close to the previous box it happens to be.

    Enrolment follows the same rule in the other direction: the template is
    built from the *first* person seen after the command, and a later frame
    showing somebody else is ignored rather than averaged in.
    """

    def __init__(
        self,
        *,
        threshold: float = 0.70,
        enroll_samples: int = 5,
    ) -> None:
        if not 0.0 < float(threshold) <= 1.0:
            raise ValueError("threshold must be in (0, 1]")
        if int(enroll_samples) < 1:
            raise ValueError("enroll_samples must be a positive integer")
        self._lock = threading.RLock()
        self._threshold = float(threshold)
        self._enroll_samples = int(enroll_samples)
        # ``available`` is set once the ROS node has actually loaded OSNet.  The
        # provider refuses ``lock`` while it is false, so a voice command can
        # never be acknowledged when no model is behind it.
        self._available = False
        self._template: np.ndarray | None = None
        self._active = False
        self._pending_enroll = 0
        self._enroll_buffer: list[np.ndarray] = []
        self._last_similarity: float | None = None
        self._candidates = 0
        self._matches = 0
        self._status = "unlocked"
        self._detail = "specified-person lock is off"
        self._revision = 0
        self._enroll_started_at: float | None = None
        self._last_prompt_at: float | None = None
        self._prompts = 0
        self._engaged = False

    # ------------------------------------------------------------ provider side

    def configure(
        self,
        *,
        threshold: float | None = None,
        enroll_samples: int | None = None,
        available: bool | None = None,
    ) -> ReidLockSnapshot:
        """Apply deployment settings to the lock (called once at node start)."""

        with self._lock:
            if threshold is not None:
                if not 0.0 < float(threshold) <= 1.0:
                    raise ValueError("threshold must be in (0, 1]")
                self._threshold = float(threshold)
            if enroll_samples is not None:
                if int(enroll_samples) < 1:
                    raise ValueError("enroll_samples must be a positive integer")
                self._enroll_samples = int(enroll_samples)
            if available is not None:
                self._available = bool(available)
                if not self._available:
                    self._status = "unavailable"
                    self._detail = "OSNet model is not loaded; lock is unavailable"
                elif self._template is not None:
                    self._status = "locked" if self._active else "unlocked"
                    self._detail = "enrolled target retained"
                else:
                    self._status = "unlocked"
                    self._detail = "specified-person lock is off"
            self._revision += 1
            return self._snapshot_locked()

    @property
    def available(self) -> bool:
        with self._lock:
            return self._available

    def request_enroll(self) -> ReidLockSnapshot:
        """Begin enrolling the current target as the person to follow."""

        with self._lock:
            self._engaged = True
            self._pending_enroll = self._enroll_samples
            self._enroll_buffer = []
            self._active = False
            self._template = None
            self._last_similarity = None
            self._status = "enrolling"
            self._detail = (
                f"collecting {self._enroll_samples} samples of the target person"
            )
            self._enroll_started_at = time.monotonic()
            self._last_prompt_at = None
            self._prompts = 0
            self._revision += 1
            return self._snapshot_locked()

    def clear(self) -> ReidLockSnapshot:
        """Forget the enrolled person and return to ordinary following."""

        with self._lock:
            self._template = None
            self._active = False
            self._engaged = False
            self._pending_enroll = 0
            self._enroll_buffer = []
            self._last_similarity = None
            self._candidates = 0
            self._matches = 0
            self._status = "unlocked"
            self._detail = "specified-person lock cleared"
            self._enroll_started_at = None
            self._last_prompt_at = None
            self._prompts = 0
            self._revision += 1
            return self._snapshot_locked()

    def snapshot(self) -> ReidLockSnapshot:
        with self._lock:
            return self._snapshot_locked()

    @property
    def threshold(self) -> float:
        with self._lock:
            return self._threshold

    # --------------------------------------------------------- estimator side

    def needs_features(self) -> bool:
        """Whether the next frame must run OSNet (skip the cost otherwise)."""

        with self._lock:
            return self._active or self._pending_enroll > 0

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def is_engaged(self) -> bool:
        """Whether specified follow was asked for and not yet cancelled.

        True from the moment the operator says "follow me" until ``clear()`` --
        while enrolment waits for a person, after enrolment failed, and while
        locked.  The estimator uses it as the gate on forward motion: between
        the command and a verified identity there is nobody the robot is
        allowed to follow, so no frame may produce a target and ``vx`` stays 0.
        """

        with self._lock:
            return self._engaged

    def pending_enroll(self) -> int:
        with self._lock:
            return self._pending_enroll

    def add_enroll_feature(self, feature: np.ndarray) -> bool:
        """Accumulate one enrollment sample; activates once enough are seen.

        Returns ``True`` when the sample was taken, ``False`` when the frame
        showed somebody who is *not* the person being enrolled.  A rejected
        sample does not count towards ``enroll_samples``: enrolment keeps
        collecting from the first person it saw instead of averaging a second
        person into the template.
        """

        with self._lock:
            if self._pending_enroll <= 0:
                return True
            usable = feature is not None and np.any(feature)
            if usable and self._enroll_buffer:
                similarity = PersonReID.cosine_similarity(
                    enroll_template(self._enroll_buffer), feature
                )
                if similarity < _ENROLL_CONSISTENCY_FLOOR:
                    self._status = "enrolling"
                    self._detail = (
                        "ignored a sample of somebody else "
                        f"(similarity {similarity:.2f} < "
                        f"{_ENROLL_CONSISTENCY_FLOOR:.2f}); still enrolling the "
                        "first person seen"
                    )
                    self._revision += 1
                    return False
            if usable:
                self._enroll_buffer.append(np.asarray(feature, dtype=np.float32))
            self._pending_enroll -= 1
            if self._pending_enroll > 0:
                self._status = "enrolling"
                self._detail = (
                    f"collecting target samples "
                    f"({len(self._enroll_buffer)}/{self._enroll_samples})"
                )
                self._revision += 1
                return True
            if not self._enroll_buffer:
                self._pending_enroll = 0
                self._status = "enroll_failed"
                self._detail = "no usable person crop during enrollment"
                self._revision += 1
                return True
            self._template = enroll_template(self._enroll_buffer)
            used = len(self._enroll_buffer)
            self._active = True
            self._enroll_buffer = []
            self._status = "locked"
            self._detail = f"locked onto target from {used} sample(s)"
            self._revision += 1
            return True

    def note_enroll_wait(
        self, now: float | None = None, reason: str | None = None
    ) -> str | None:
        """Report that an enrolment frame produced nobody to enrol.

        The estimator calls this once per frame while an enrolment is pending
        and the frame yielded no usable person -- nobody in view, nobody tall
        enough for OSNet's crop, or somebody who is not the person being
        enrolled (``reason``).  Returns the prompt text when the announce
        interval has elapsed (so the caller re-announces the wait), or ``None``
        when it is still too soon.

        Enrolment is deliberately *not* abandoned here.  The operator asked to
        follow a person; the lock keeps waiting for one, and keeps saying so,
        for as long as it takes.  Without this the lock used to sit in
        ``enrolling`` forever with nothing said -- neither success nor failure.
        """

        with self._lock:
            if self._pending_enroll <= 0:
                return None
            now = time.monotonic() if now is None else float(now)
            since = None if self._last_prompt_at is None else now - self._last_prompt_at
            if since is not None and since < _ENROLL_PROMPT_INTERVAL_S:
                return None
            self._last_prompt_at = now
            self._prompts += 1
            waited = (
                0.0
                if self._enroll_started_at is None
                else max(0.0, now - self._enroll_started_at)
            )
            self._status = "enrolling_no_person"
            self._detail = (
                f"{reason or 'no person in view'}; still waiting to enrol the "
                f"target ({waited:.0f}s elapsed, prompt #{self._prompts})"
            )
            self._revision += 1
            return self._detail

    def match_features(
        self, features: Sequence[np.ndarray]
    ) -> tuple[list[float | None], float | None]:
        """Score every detection against the enrolled person.

        Returns ``(similarities, best)`` where each similarity is the cosine
        distance to the template, or ``None`` for a detection OSNet could not
        score (crop too small or out of frame).  The caller decides who is the
        target from these numbers; the lock itself never picks a winner.

        When the lock is not active every score is ``None`` and nothing is
        filtered, so the caller falls back to its ordinary selection.
        """

        with self._lock:
            if not self._active or self._template is None:
                return [None] * len(features), None
            template = self._template
            threshold = self._threshold
            similarities: list[float | None] = []
            best: float | None = None
            for feature in features:
                if feature is None or not np.any(feature):
                    similarities.append(None)
                    continue
                similarity = PersonReID.cosine_similarity(template, feature)
                if best is None or similarity > best:
                    best = similarity
                similarities.append(similarity)
            self._last_similarity = best
            self._candidates = len(features)
            self._matches = sum(
                1
                for score in similarities
                if score is not None and score >= threshold
            )
            self._status = "locked" if self._matches else "locked_no_match"
            self._detail = (
                f"{self._matches}/{self._candidates} candidate(s) match, "
                f"threshold {threshold:.2f}"
                + (f", best {best:.3f}" if best is not None else "")
            )
            self._revision += 1
            return similarities, best

    def filter_features(
        self, features: Sequence[np.ndarray]
    ) -> tuple[list[bool], float | None]:
        """Mark which detections match the enrolled person.

        Returns ``(keep_mask, best_similarity)``.  When the lock is not active
        every detection is kept, so the caller can use this unconditionally.
        """

        with self._lock:
            if not self._active or self._template is None:
                return [True] * len(features), None
            similarities, best = self.match_features(features)
            threshold = self._threshold
            return [
                score is not None and score >= threshold for score in similarities
            ], best

    def _snapshot_locked(self) -> ReidLockSnapshot:
        # ``waiting_s`` only means anything while a person is still being
        # looked for; once enrolled (or idle) it reads as zero.
        if self._pending_enroll > 0 and self._enroll_started_at is not None:
            waiting_s = max(0.0, time.monotonic() - self._enroll_started_at)
        else:
            waiting_s = 0.0
        return ReidLockSnapshot(
            active=self._active,
            available=self._available,
            enrolled=self._template is not None,
            pending_enroll=self._pending_enroll,
            threshold=self._threshold,
            last_similarity=self._last_similarity,
            candidates=self._candidates,
            matches=self._matches,
            status=self._status,
            detail=self._detail,
            revision=self._revision,
            waiting_s=waiting_s,
            prompts=self._prompts,
            engaged=self._engaged,
        )


# Module-level singleton: the provider writes lock/unlock, the ROS node's
# estimator thread reads it.  Importing this module does NOT import torch.
reid_lock_state = ReidLockState()


__all__ = [
    "BoundingBoxLike",
    "PersonReID",
    "ReidLockSnapshot",
    "ReidLockState",
    "enroll_template",
    "reid_lock_state",
]
