#!/usr/bin/env python3
"""Verify the latest recorded rosbag contains the required topics.

This checks that the newest rosbag directory contains readable messages for:
- /joint_states
- /camera1/realsense_camera/color/image_raw/compressed
- /camera2/realsense_camera/color/image_raw/compressed
- xarm5_joint1..xarm5_joint5 and xarm_gripper_right_drive_joint in /joint_states

The script prints the bag it is reading, logs the verification process, and
returns a non-zero exit code on failure.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BAGS_DIR = SCRIPT_DIR / "pick_place_rosbags"
DEFAULT_JOINT_TOPIC = "/joint_states"
DEFAULT_CAMERA1_TOPIC = "/camera1/realsense_camera/color/image_raw/compressed"
DEFAULT_CAMERA2_TOPIC = "/camera2/realsense_camera/color/image_raw/compressed"
REQUIRED_JOINTS = [
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "xarm_gripper_right_drive_joint",
]
MIN_POSITION_SPAN = 1e-4
MIN_ARM_POSITION_SPAN = 1e-3
MIN_GRIPPER_POSITION_SPAN = 5e-4


def log(msg: str) -> None:
    print(f"[verify_latest_rosbag] {msg}", flush=True)


def find_latest_bag(bags_dir: Path) -> Path:
    candidates = [
        path
        for path in bags_dir.iterdir()
        if path.is_dir() and (path / "metadata.yaml").exists()
    ]
    if not candidates:
        raise FileNotFoundError(f"No rosbag directories found in {bags_dir}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def summarize_connections(reader: AnyReader, topics: list[str]) -> dict[str, int]:
    counts = {topic: 0 for topic in topics}
    for conn in reader.connections:
        if conn.topic in counts:
            counts[conn.topic] += 1
    return counts


def summarize_joint_data(joint_samples: dict[str, list[float]]) -> tuple[bool, list[str]]:
    success = True
    details: list[str] = []
    for joint_name in REQUIRED_JOINTS:
        samples = joint_samples[joint_name]
        if not samples:
            details.append(f"Failure: required joint never observed: {joint_name}")
            success = False
            continue

        joint_min = min(samples)
        joint_max = max(samples)
        span = joint_max - joint_min
        min_required_span = (
            MIN_GRIPPER_POSITION_SPAN
            if joint_name == "xarm_gripper_right_drive_joint"
            else MIN_ARM_POSITION_SPAN
        )
        msg = (
            f"Joint {joint_name}: samples={len(samples)} "
            f"min={joint_min:.6f} max={joint_max:.6f} span={span:.6f}"
        )
        if span <= MIN_POSITION_SPAN:
            msg += " [static]"
        if span < min_required_span:
            msg += f" [insufficient motion, need >= {min_required_span:.6f}]"
            success = False
        details.append(msg)
    return success, details


def verify_bag(
    bag_path: Path,
    joint_topic: str,
    camera1_topic: str,
    camera2_topic: str,
) -> bool:
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    required_topics = [joint_topic, camera1_topic, camera2_topic]
    readable_counts = {topic: 0 for topic in required_topics}
    joint_message_count = 0
    complete_joint_message_count = 0
    joint_samples = {joint_name: [] for joint_name in REQUIRED_JOINTS}
    failure_reasons: list[str] = []

    log(f"Reading bag: {bag_path}")
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connection_counts = summarize_connections(reader, required_topics)
        for topic in required_topics:
            log(f"Found {connection_counts[topic]} connection(s) for topic: {topic}")

        connections = [conn for conn in reader.connections if conn.topic in required_topics]
        if not connections:
            log("Failure: none of the required topics are present in the bag")
            log(
                "Summary: FAIL - missing all required topics "
                f"({', '.join(required_topics)})"
            )
            return False

        log("Starting message verification")
        for conn, _ts, rawdata in reader.messages(connections=connections):
            if conn.topic == joint_topic:
                msg = typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/JointState")
                joint_message_count += 1
                name_to_pos = dict(zip(msg.name, msg.position))
                if all(joint_name in name_to_pos for joint_name in REQUIRED_JOINTS):
                    complete_joint_message_count += 1
                for joint_name in REQUIRED_JOINTS:
                    if joint_name in name_to_pos:
                        joint_samples[joint_name].append(float(name_to_pos[joint_name]))
            elif conn.topic in (camera1_topic, camera2_topic):
                typestore.deserialize_cdr(rawdata, "sensor_msgs/msg/CompressedImage")
            readable_counts[conn.topic] += 1

        success = True
        for topic in required_topics:
            count = readable_counts[topic]
            log(f"Readable messages for {topic}: {count}")
            if count == 0:
                log(f"Failure: topic has no readable messages: {topic}")
                failure_reasons.append(f"no readable messages on {topic}")
                success = False

        log(f"Readable joint-state messages: {joint_message_count}")
        log(
            "Joint-state messages containing full xarm5 + gripper set: "
            f"{complete_joint_message_count}"
        )
        if joint_message_count == 0:
            log("Failure: no readable joint state messages found")
            failure_reasons.append("no readable joint state messages")
            success = False
        if complete_joint_message_count == 0:
            log("Failure: no joint state message contained the full required joint set")
            failure_reasons.append("required xarm5/gripper joint set never appeared together")
            success = False

        joints_ok, joint_details = summarize_joint_data(joint_samples)
        for detail in joint_details:
            log(detail)
            if "[insufficient motion" in detail:
                failure_reasons.append(detail)
            elif detail.startswith("Failure:"):
                failure_reasons.append(detail)
        success = success and joints_ok

    if success:
        log("Verification succeeded")
        log("Summary: PASS - cameras and required xarm5/gripper joint data look good")
    else:
        log("Verification failed")
        log(f"Summary: FAIL - {'; '.join(failure_reasons)}")
    return success


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Verify the latest recorded rosbag.")
    parser.add_argument("--bags-dir", default=str(DEFAULT_BAGS_DIR))
    parser.add_argument("--bag", default=None, help="Optional explicit rosbag directory to verify.")
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--camera1-topic", default=DEFAULT_CAMERA1_TOPIC)
    parser.add_argument("--camera2-topic", default=DEFAULT_CAMERA2_TOPIC)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    bags_dir = Path(args.bags_dir).expanduser().resolve()
    if not bags_dir.exists():
        print(f"Bags directory does not exist: {bags_dir}", file=sys.stderr)
        return 1

    try:
        bag_path = (
            Path(args.bag).expanduser().resolve()
            if args.bag is not None
            else find_latest_bag(bags_dir)
        )
        ok = verify_bag(
            bag_path=bag_path,
            joint_topic=args.joint_topic,
            camera1_topic=args.camera1_topic,
            camera2_topic=args.camera2_topic,
        )
        return 0 if ok else 1
    except Exception as exc:
        print(f"[verify_latest_rosbag] Verification error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
