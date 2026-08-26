#!/usr/bin/env python3
"""Verify that recorded ACT rosbags contain usable training data.

By default, this checks the most recently modified rosbag under ``rosbags/``.
Use ``--bag`` to check one explicit episode or ``--all`` to check every episode.

The verifier returns a nonzero exit code when a required topic is missing,
messages cannot be deserialized, required xArm5/gripper joints are absent, or
the recorded joint motion is too small for useful ACT demonstrations.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BAGS_DIR = SCRIPT_DIR / "rosbags"
DEFAULT_JOINT_TOPIC = "/joint_states"
DEFAULT_CAMERA1_TOPIC = "/camera1/realsense_camera/color/image_raw/compressed"
DEFAULT_CAMERA2_TOPIC = "/camera2/realsense_camera/color/image_raw/compressed"
JOINT_MESSAGE_TYPE = "sensor_msgs/msg/JointState"
CAMERA_MESSAGE_TYPE = "sensor_msgs/msg/CompressedImage"
REQUIRED_JOINTS = (
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "xarm_gripper_right_drive_joint",
)
MIN_POSITION_SPAN = 1e-4
MIN_ARM_POSITION_SPAN = 1e-3
MIN_GRIPPER_POSITION_SPAN = 5e-4


def log(message: str) -> None:
    print(f"[verify_latest_rosbag] {message}", flush=True)


def find_bags(bags_dir: Path) -> list[Path]:
    """Return rosbag directories in stable name order."""
    if not bags_dir.is_dir():
        raise FileNotFoundError(f"Bags directory does not exist: {bags_dir}")

    bags = sorted(path for path in bags_dir.iterdir() if path.is_dir() and (path / "metadata.yaml").is_file())
    if not bags:
        raise FileNotFoundError(f"No rosbag directories found in {bags_dir}")
    return bags


def find_latest_bag(bags_dir: Path) -> Path:
    """Return the most recently modified rosbag directory."""
    return max(find_bags(bags_dir), key=lambda path: path.stat().st_mtime)


def summarize_joint_data(
    joint_samples: dict[str, list[float]],
) -> tuple[bool, list[str], list[str]]:
    success = True
    details: list[str] = []
    failures: list[str] = []

    for joint_name in REQUIRED_JOINTS:
        samples = joint_samples[joint_name]
        if not samples:
            failure = f"required joint never observed: {joint_name}"
            details.append(f"Failure: {failure}")
            failures.append(failure)
            success = False
            continue

        joint_min = min(samples)
        joint_max = max(samples)
        span = joint_max - joint_min
        minimum_span = (
            MIN_GRIPPER_POSITION_SPAN
            if joint_name == "xarm_gripper_right_drive_joint"
            else MIN_ARM_POSITION_SPAN
        )
        message = (
            f"Joint {joint_name}: samples={len(samples)} "
            f"min={joint_min:.6f} max={joint_max:.6f} span={span:.6f}"
        )
        if span <= MIN_POSITION_SPAN:
            message += " [static]"
        if span < minimum_span:
            failure = f"{joint_name} has insufficient motion ({span:.6f} < {minimum_span:.6f})"
            message += f" [insufficient motion, need >= {minimum_span:.6f}]"
            failures.append(failure)
            success = False
        details.append(message)

    return success, details, failures


def verify_bag(
    bag_path: Path,
    joint_topic: str,
    camera1_topic: str,
    camera2_topic: str,
) -> bool:
    """Verify required topics, messages, joints, and motion in one rosbag."""
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        log(f"Failure: not a ROS 2 bag directory: {bag_path}")
        return False

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    required_topics = (joint_topic, camera1_topic, camera2_topic)
    expected_types = {
        joint_topic: JOINT_MESSAGE_TYPE,
        camera1_topic: CAMERA_MESSAGE_TYPE,
        camera2_topic: CAMERA_MESSAGE_TYPE,
    }
    readable_counts = {topic: 0 for topic in required_topics}
    connection_types = {topic: set() for topic in required_topics}
    joint_message_count = 0
    complete_joint_message_count = 0
    empty_image_count = {camera1_topic: 0, camera2_topic: 0}
    joint_samples = {joint_name: [] for joint_name in REQUIRED_JOINTS}
    failure_reasons: list[str] = []

    log(f"Reading bag: {bag_path}")
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for connection in reader.connections:
            if connection.topic in connection_types:
                connection_types[connection.topic].add(connection.msgtype)

        for topic in required_topics:
            observed_types = connection_types[topic]
            if not observed_types:
                failure = f"required topic is missing: {topic}"
                log(f"Failure: {failure}")
                failure_reasons.append(failure)
            elif expected_types[topic] not in observed_types:
                failure = f"{topic} has type(s) {sorted(observed_types)}, expected {expected_types[topic]}"
                log(f"Failure: {failure}")
                failure_reasons.append(failure)
            else:
                log(f"Found {topic} [{expected_types[topic]}]")

        connections = [connection for connection in reader.connections if connection.topic in required_topics]
        if not connections:
            log("Summary: FAIL - none of the required topics are present")
            return False

        log("Starting message verification")
        for connection, _timestamp, rawdata in reader.messages(connections=connections):
            if connection.topic == joint_topic:
                message = typestore.deserialize_cdr(rawdata, JOINT_MESSAGE_TYPE)
                joint_message_count += 1
                name_to_position = dict(zip(message.name, message.position, strict=False))
                if all(joint_name in name_to_position for joint_name in REQUIRED_JOINTS):
                    complete_joint_message_count += 1
                for joint_name in REQUIRED_JOINTS:
                    if joint_name in name_to_position:
                        joint_samples[joint_name].append(float(name_to_position[joint_name]))
            else:
                message = typestore.deserialize_cdr(rawdata, CAMERA_MESSAGE_TYPE)
                if len(message.data) == 0:
                    empty_image_count[connection.topic] += 1
            readable_counts[connection.topic] += 1

    for topic in required_topics:
        count = readable_counts[topic]
        log(f"Readable messages for {topic}: {count}")
        if count == 0:
            failure_reasons.append(f"no readable messages on {topic}")

    for topic, count in empty_image_count.items():
        if count:
            failure_reasons.append(f"{count} empty compressed-image message(s) on {topic}")

    log(f"Readable joint-state messages: {joint_message_count}")
    log(f"Joint-state messages containing the full xArm5 + gripper set: {complete_joint_message_count}")
    if joint_message_count == 0:
        failure_reasons.append("no readable joint-state messages")
    if complete_joint_message_count == 0:
        failure_reasons.append("required xArm5/gripper joint set never appeared together")

    joints_ok, joint_details, joint_failures = summarize_joint_data(joint_samples)
    for detail in joint_details:
        log(detail)
    failure_reasons.extend(joint_failures)

    success = not failure_reasons and joints_ok
    if success:
        log("Summary: PASS - cameras and required xArm5/gripper data look good")
    else:
        log(f"Summary: FAIL - {'; '.join(failure_reasons)}")
    return success


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Verify recorded ROS 2 bags for ACT training.")
    parser.add_argument("--bags-dir", default=str(DEFAULT_BAGS_DIR))
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--bag", help="Explicit rosbag directory to verify.")
    selection.add_argument(
        "--all",
        action="store_true",
        help="Verify every rosbag directory under --bags-dir.",
    )
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--camera1-topic", default=DEFAULT_CAMERA1_TOPIC)
    parser.add_argument("--camera2-topic", default=DEFAULT_CAMERA2_TOPIC)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    bags_dir = Path(args.bags_dir).expanduser().resolve()

    try:
        if args.bag:
            bag_paths = [Path(args.bag).expanduser().resolve()]
        elif args.all:
            bag_paths = find_bags(bags_dir)
        else:
            bag_paths = [find_latest_bag(bags_dir)]
    except OSError as error:
        print(f"[verify_latest_rosbag] Selection error: {error}", file=sys.stderr)
        return 1

    failed_bags: list[Path] = []
    for index, bag_path in enumerate(bag_paths):
        if index:
            print()
        try:
            passed = verify_bag(
                bag_path=bag_path,
                joint_topic=args.joint_topic,
                camera1_topic=args.camera1_topic,
                camera2_topic=args.camera2_topic,
            )
        # Corrupt SQLite/CDR data can raise backend-specific exception types.
        except Exception as error:  # noqa: BLE001
            log(f"Verification error for {bag_path}: {error}")
            passed = False
        if not passed:
            failed_bags.append(bag_path)

    if failed_bags:
        log(f"Overall result: FAIL ({len(failed_bags)}/{len(bag_paths)} bag(s) failed)")
        return 1

    log(f"Overall result: PASS ({len(bag_paths)} bag(s) verified)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
