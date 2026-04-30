#!/usr/bin/env python3
"""Validate rosbags recorded during data collection.

Checks:
  - All three expected topics are present in every bag
  - Each topic has enough messages (configurable minimum)
  - No bag is suspiciously short (< MIN_EPISODE_DURATION_S seconds)
  - Camera frame count matches joint-state frame count within a tolerance

Usage:
  python3 validate_bags.py [--bag-dir rosbags] [--min-frames 50]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaac_data_collection.config import CAMERA1_TOPIC, CAMERA2_TOPIC, JOINT_STATE_TOPIC

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("Install rosbags: pip install rosbags", file=sys.stderr)
    sys.exit(1)

REQUIRED_TOPICS = [JOINT_STATE_TOPIC, CAMERA1_TOPIC, CAMERA2_TOPIC]
MIN_EPISODE_DURATION_S = 5.0
CAMERA_FRAME_TOLERANCE = 0.20  # allow 20% deviation between joint and camera frame counts


def validate_bag(bag_dir: Path, min_frames: int) -> tuple[bool, str]:
    """Return (ok, message)."""
    if not bag_dir.exists():
        return False, "directory does not exist"

    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with Reader(bag_dir) as reader:
            topic_counts: dict[str, int] = {t: 0 for t in REQUIRED_TOPICS}
            start_ns: int | None = None
            end_ns: int | None = None

            connections = {c.topic: c for c in reader.connections}
            for topic in REQUIRED_TOPICS:
                if topic not in connections:
                    return False, f"missing topic: {topic}"

            for conn, timestamp, _ in reader.messages():
                if conn.topic in topic_counts:
                    topic_counts[conn.topic] += 1
                    if start_ns is None or timestamp < start_ns:
                        start_ns = timestamp
                    if end_ns is None or timestamp > end_ns:
                        end_ns = timestamp

            if start_ns is None:
                return False, "bag is empty"

            duration_s = (end_ns - start_ns) / 1e9
            if duration_s < MIN_EPISODE_DURATION_S:
                return False, f"too short: {duration_s:.1f}s < {MIN_EPISODE_DURATION_S}s"

            js_count = topic_counts[JOINT_STATE_TOPIC]
            c1_count = topic_counts[CAMERA1_TOPIC]
            c2_count = topic_counts[CAMERA2_TOPIC]

            for topic, count in topic_counts.items():
                if count < min_frames:
                    return False, f"{topic}: only {count} frames (< {min_frames})"

            for cam_topic, cam_count in [(CAMERA1_TOPIC, c1_count), (CAMERA2_TOPIC, c2_count)]:
                if js_count > 0:
                    ratio = abs(cam_count - js_count) / js_count
                    if ratio > CAMERA_FRAME_TOLERANCE:
                        return False, (
                            f"frame-count mismatch: joint_states={js_count}, "
                            f"{cam_topic.split('/')[-2]}={cam_count} (ratio={ratio:.2f})"
                        )

            return True, (
                f"OK — {duration_s:.1f}s, js={js_count}, cam1={c1_count}, cam2={c2_count}"
            )

    except Exception as exc:
        return False, f"read error: {exc}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate collected rosbags.")
    parser.add_argument("--bag-dir", default="rosbags")
    parser.add_argument("--min-frames", type=int, default=50)
    args = parser.parse_args()

    bag_root = Path(args.bag_dir)
    if not bag_root.exists():
        print(f"Bag directory not found: {bag_root}", file=sys.stderr)
        return 1

    bag_dirs = sorted(bag_root.iterdir())
    if not bag_dirs:
        print("No bags found.", file=sys.stderr)
        return 1

    ok_count = 0
    fail_count = 0
    for bag_dir in bag_dirs:
        if not bag_dir.is_dir():
            continue
        ok, msg = validate_bag(bag_dir, args.min_frames)
        status = "✓" if ok else "✗"
        print(f"  {status}  {bag_dir.name}: {msg}")
        if ok:
            ok_count += 1
        else:
            fail_count += 1

    total = ok_count + fail_count
    print(f"\n{ok_count}/{total} bags passed validation.")
    if fail_count > 0:
        print("Failed bags should be re-recorded or excluded from training.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
