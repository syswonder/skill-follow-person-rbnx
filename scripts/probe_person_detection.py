#!/usr/bin/env python3
"""Diagnose why the fixed-distance layer reports no measurement.

Run INSIDE the follow container:

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    python3 scripts/probe_person_detection.py 2>/dev/null

It drives the *real* estimator from packages/go2_robottrack with the *live*
camera streams, so whatever it reports is exactly what the follow stack sees.
It only reads topics -- it publishes nothing and cannot move the robot.

Printed every 2 s:

    frames   RGB/depth frame counts and their image sizes
    pair     whether RGB and depth are being paired (and why not, if not)
    detect   HOG's answer: person found or not, bbox, and confidence
    range    the metric distance the stack would act on

The three failure modes this separates:

  * `pair` never succeeds  -> the RGB/depth timestamps do not line up
  * `pair` ok, `detect no` -> OpenCV HOG cannot see the person at this size
                              (this is the usual cause after a resolution change)
  * `detect yes`, no range -> the depth inside the box is invalid
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, "/root/deploy/packages/go2_robottrack")

from go2_robottrack.rgbd_distance import (  # noqa: E402
    RgbdDistanceConfig,
    RgbdPersonDistanceEstimator,
)
from go2_robottrack.rgbd_sync import LatestRgbdPairer  # noqa: E402


RGB_TOPIC = "/camera_435i/color/image_raw"
DEPTH_TOPIC = "/camera_435i/depth/image_raw"
PAIR_MAX_DELTA_S = 0.20
MAX_FRAME_AGE_S = 0.60


def _manifest_detector_floor() -> float:
    """The deployed ``detector_min_confidence``, so the report matches the stack.

    Read from the manifest rather than copied here: a probe carrying its own
    copy of the value reports a floor the robot is not actually using.
    """

    import yaml

    for candidate in (
        Path(os.environ.get("ROBONIX_DEPLOY_DIR", "/root/deploy"))
        / "robonix_manifest.yaml",
        Path(__file__).resolve().parent.parent / "robonix_manifest.yaml",
    ):
        try:
            manifest = yaml.safe_load(candidate.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - a missing manifest is not fatal here
            continue
        try:
            block = next(
                item
                for item in manifest["primitive"]
                if item["name"] == "go2_robottrack"
            )["config"]
        except Exception:  # noqa: BLE001
            continue
        value = block.get("detector_min_confidence")
        if value is not None:
            return float(value)
    return 0.0


DETECTOR_MIN_CONFIDENCE = _manifest_detector_floor()


def _stamp(message: Image) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("person_detection_probe")
        self._bridge = CvBridge()
        self._pairer = LatestRgbdPairer(
            max_source_delta_s=PAIR_MAX_DELTA_S,
            max_frame_age_s=MAX_FRAME_AGE_S,
        )
        self._estimator = RgbdPersonDistanceEstimator(
            config=RgbdDistanceConfig(
                max_frame_age_s=MAX_FRAME_AGE_S,
                enable_center_fallback=False,
                detector_min_confidence=DETECTOR_MIN_CONFIDENCE,
            )
        )
        self.rgb_n = 0
        self.depth_n = 0
        self.rgb_size = "-"
        self.depth_size = "-"
        self.rgb_stamp = None
        self.depth_stamp = None
        self.last_pair_reason = "no frames yet"
        self.last_result = None
        self.last_bgr = None
        self.create_subscription(
            Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data
        )
        self.create_timer(2.0, self._report)

    def _on_rgb(self, message: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"rgb decode failed: {error}")
            return
        self.rgb_n += 1
        self.rgb_size = f"{bgr.shape[1]}x{bgr.shape[0]}"
        self.rgb_stamp = _stamp(message)
        self.last_bgr = bgr
        self._consume(
            self._pairer.offer_rgb(bgr, source_timestamp_s=self.rgb_stamp)
        )

    def _on_depth(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f"depth decode failed: {error}")
            return
        self.depth_n += 1
        self.depth_size = f"{depth.shape[1]}x{depth.shape[0]}"
        self.depth_stamp = _stamp(message)
        self._consume(
            self._pairer.offer_depth(depth, source_timestamp_s=self.depth_stamp)
        )

    def _consume(self, decision) -> None:
        # RgbdPairDecision exposes `status` (not `reason`).
        self.last_pair_reason = getattr(decision, "status", "?")
        pair = getattr(decision, "pair", None)
        if pair is None:
            return
        # The estimator validates [and rejects] mismatched RGB/depth shapes by
        # raising, so the exception text is the single most useful thing here.
        try:
            self.last_result = self._estimator.estimate(pair.rgb.image, pair.depth.image)
        except Exception as error:  # noqa: BLE001
            self.last_result = f"RAISED {type(error).__name__}: {error}"

    def _report(self) -> None:
        lines = [f"person-detection probe   {time.strftime('%H:%M:%S')}", ""]
        lines.append(f"rgb frames   {self.rgb_n:6d}   size {self.rgb_size}")
        lines.append(f"depth frames {self.depth_n:6d}   size {self.depth_size}")
        if self.rgb_stamp and self.depth_stamp:
            lines.append(
                f"stamp delta  {abs(self.rgb_stamp - self.depth_stamp):.3f} s"
                f"   (pairer allows {PAIR_MAX_DELTA_S} s)"
            )
        lines.append(f"pair         {self.last_pair_reason}")

        # Every box HOG returned, not just the one that won: this is the
        # distribution `detector_min_confidence` has to cut through.  A real
        # person sitting at the bottom of this list means the floor is set too
        # high (the robot will stop); a long tail of weak boxes means it is set
        # too low (the robot will walk off after furniture).
        detector = getattr(self._estimator, "_detector", None)
        if detector is not None and self.last_bgr is not None:
            try:
                boxes = detector.detect(self.last_bgr)
            except Exception as error:  # noqa: BLE001
                lines.append(f"detector     RAISED {type(error).__name__}: {error}")
            else:
                ranked = sorted(
                    (float(d.confidence), int(d.bbox.height)) for d in boxes
                )
                lines.append(
                    f"detector     {len(boxes)} box(es)"
                    f"   floor={DETECTOR_MIN_CONFIDENCE}"
                )
                for confidence, height in ranked[-8:]:
                    lines.append(f"             conf={confidence:.3f}  h={height}px")
                if len(ranked) > 8:
                    lines.append(f"             (+{len(ranked) - 8} weaker box(es))")
                weak = sum(1 for c, _ in ranked if c < DETECTOR_MIN_CONFIDENCE)
                lines.append(
                    f"             {weak} below the floor "
                    f"({len(ranked) - weak} kept)"
                )

        result = self.last_result
        if result is None:
            lines.append("estimate     (no pair consumed yet)")
        elif isinstance(result, str):
            lines.append(f"estimate     {result}")
        else:
            box = result.bbox
            lines.append(
                f"estimate     valid={result.valid}  status={result.status}"
                f"  source={result.source}"
            )
            lines.append(
                f"             conf={result.confidence:.3f}"
                f"  distance_m={result.distance_m}"
            )
            if box is not None:
                lines.append(
                    f"bbox         x={box.x:.0f} y={box.y:.0f} "
                    f"w={box.width:.0f} h={box.height:.0f}"
                )
            lines.append(
                f"depth        valid_fraction={result.valid_fraction:.3f}"
                f"  samples={result.valid_samples}  mad={result.depth_mad_m}"
            )

        lines.append("")
        lines.append("判读:")
        lines.append("  estimate RAISED ...match the BGR height  -> RGB 与深度尺寸不一致")
        lines.append("  pair status 一直 awaiting_matching_depth  -> 时间戳或尺寸对不上")
        lines.append("  valid=False, status=no_detection          -> HOG 认不出人")
        lines.append("  valid=False, status= 深度类               -> 框内深度无效")
        print("\033[2J\033[H" + "\n".join(lines), flush=True)


def main() -> int:
    rclpy.init(args=sys.argv[1:])
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
