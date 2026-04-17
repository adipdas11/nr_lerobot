#!/usr/bin/env python3
"""Python 3.10 ROS relay for SmolVLA policy inference."""

from __future__ import annotations

import argparse
import socket
import sys
import threading
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState

from nr_dual_arm_moveit_config.motion_backend import MotionBackend
from smolvla_ipc import recv_message, send_message


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SOCKET_PATH = "/tmp/smolvla_policy.sock"
DEFAULT_CAMERA1_TOPIC = "/camera1/realsense_camera/color/image_raw/compressed"
DEFAULT_CAMERA2_TOPIC = "/camera2/realsense_camera/color/image_raw/compressed"
DEFAULT_TASK = "pick up the cube"
DEFAULT_ARM_JOINTS = (
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
)
DEFAULT_STATE_JOINTS = (
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "xarm_gripper_right_drive_joint",
)
DEFAULT_GRIPPER_JOINT = "xarm_gripper_right_drive_joint"


def parse_csv_list(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def resolve_joint_topic(hardware_type: str, override: str | None) -> str:
    if override:
        return override
    if hardware_type in {"isaac", "twin"}:
        return "/filtered_joint_states"
    if hardware_type == "real":
        return "/robot_joint_states"
    return "/joint_states"


class SmolVLARosClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("smolvla_ros_client")
        self._lock = threading.Lock()
        self._socket_path = args.socket_path
        self._task = args.task
        self._dry_run = args.dry_run
        self._action_fps = float(args.action_fps)
        self._prediction_period_s = float(args.prediction_period_s)
        self._prefetch_fraction = float(args.prefetch_fraction)
        self._arm_only = args.arm_only
        self._state_joint_names = parse_csv_list(args.state_joints)
        self._arm_joint_names = parse_csv_list(args.arm_joints)
        self._gripper_joint_name = args.gripper_joint
        self._latest_joint_state: JointState | None = None
        self._latest_images: dict[str, bytes] = {}

        # Current chunk being streamed.
        self._chunk_active = False
        self._pending_actions: np.ndarray | None = None
        self._next_action_index = 0
        self._chunks_sent = 0
        self._max_chunks = args.max_chunks
        self._last_wait_reason: str | None = None

        # Prefetch state — next chunk fetched in background while current one streams.
        self._prefetch_lock = threading.Lock()
        self._prefetch_in_flight = False          # background thread running
        self._prefetched_actions: np.ndarray | None = None  # result ready
        self._prefetch_triggered = False          # already triggered for this chunk

        self.declare_parameter("hardware_type", args.hardware_type)
        self._arm_backend = MotionBackend(self, "xarm5_arm_no_slide", defer_exotica_init=True)
        self._gripper_backend = MotionBackend(self, "xarm_gripper", defer_exotica_init=True)

        self.create_subscription(JointState, args.joint_topic, self._joint_state_cb, 10)
        self.create_subscription(
            CompressedImage,
            args.camera1_topic,
            lambda msg: self._image_cb("camera1", msg),
            5,
        )
        self.create_subscription(
            CompressedImage,
            args.camera2_topic,
            lambda msg: self._image_cb("camera2", msg),
            5,
        )
        # Idle timer: fires only to kick off the very first chunk (or recover after errors).
        self._timer = self.create_timer(self._prediction_period_s, self._timer_cb)
        self._stream_timer = self.create_timer(1.0 / self._action_fps, self._stream_timer_cb)
        self.get_logger().info(f"SmolVLA socket path: {self._socket_path}")
        self.get_logger().info(f"Joint topic: {args.joint_topic}")
        self.get_logger().info(
            f"Prefetch fraction: {self._prefetch_fraction} "
            f"(next prediction starts at {self._prefetch_fraction*100:.0f}% through current chunk)"
        )
        if self._dry_run:
            self.get_logger().info("Dry-run mode enabled: controller goals will not be sent.")
        if self._arm_only:
            self.get_logger().info("Arm-only mode enabled: gripper commands will not be streamed.")

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg

    def _image_cb(self, camera_name: str, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_images[camera_name] = bytes(msg.data)

    # ------------------------------------------------------------------
    # Prediction helpers
    # ------------------------------------------------------------------

    def _fetch_prediction(self) -> np.ndarray | None:
        """Blocking prediction call — run this in a background thread."""
        request = self._build_request()
        if request is None:
            return None
        try:
            response = self._request_prediction(request)
        except Exception as exc:
            self.get_logger().error(f"Policy server request failed: {exc}")
            return None
        if not response.get("ok", False):
            self.get_logger().error(f"Policy server returned error: {response.get('error')}")
            return None
        return np.asarray(response["actions"], dtype=np.float32)

    def _start_prefetch(self) -> None:
        """Kick off a background prediction thread if none is in flight."""
        with self._prefetch_lock:
            if self._prefetch_in_flight or self._prefetched_actions is not None:
                return
            self._prefetch_in_flight = True

        def _worker():
            actions = self._fetch_prediction()
            with self._prefetch_lock:
                self._prefetch_in_flight = False
                if actions is not None:
                    self._prefetched_actions = actions
                    self.get_logger().info(
                        f"Prefetch ready: next chunk in buffer "
                        f"(arm=[{float(actions[:, :len(self._arm_joint_names)].min()):.3f}, "
                        f"{float(actions[:, :len(self._arm_joint_names)].max()):.3f}] "
                        f"grip=[{float(actions[:, -1].min()):.3f}, "
                        f"{float(actions[:, -1].max()):.3f}])"
                    )
                else:
                    self.get_logger().warning("Prefetch returned no actions — will retry on next cycle.")

        threading.Thread(target=_worker, daemon=True).start()

    def _activate_chunk(self, actions: np.ndarray) -> None:
        """Make a fetched action array the active chunk. Called from stream timer."""
        self._chunks_sent += 1
        if self._dry_run:
            arm_min = float(actions[:, : len(self._arm_joint_names)].min())
            arm_max = float(actions[:, : len(self._arm_joint_names)].max())
            grip_min = float(actions[:, -1].min())
            grip_max = float(actions[:, -1].max())
            self.get_logger().info(
                f"Predicted chunk {self._chunks_sent}: "
                f"arm=[{arm_min:.4f}, {arm_max:.4f}] grip=[{grip_min:.4f}, {grip_max:.4f}]"
            )
            return
        self._pending_actions = actions
        self._next_action_index = 0
        self._prefetch_triggered = False
        self._chunk_active = True

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        """Idle-path timer: starts the first chunk or recovers after a prefetch miss."""
        if self._chunk_active:
            return
        if self._max_chunks is not None and self._chunks_sent >= self._max_chunks:
            self.get_logger().info("Reached max_chunks limit, stopping inference loop.")
            self._timer.cancel()
            return

        # If the prefetch already landed, activate it immediately.
        with self._prefetch_lock:
            prefetched = self._prefetched_actions
            if prefetched is not None:
                self._prefetched_actions = None

        if prefetched is not None:
            self._activate_chunk(prefetched)
            return

        # No prefetch ready — block and fetch now (first chunk or error recovery).
        with self._prefetch_lock:
            if self._prefetch_in_flight:
                return  # background thread is running, wait for it

        actions = self._fetch_prediction()
        if actions is None:
            return
        self._activate_chunk(actions)

    def _stream_timer_cb(self) -> None:
        if not self._chunk_active or self._pending_actions is None:
            return

        chunk_len = len(self._pending_actions)

        # Trigger prefetch when we reach prefetch_fraction through the current chunk.
        if not self._prefetch_triggered:
            prefetch_threshold = int(chunk_len * self._prefetch_fraction)
            if self._next_action_index >= prefetch_threshold:
                if self._max_chunks is None or self._chunks_sent < self._max_chunks:
                    self._prefetch_triggered = True
                    self._start_prefetch()

        # Chunk finished — hand off to prefetched result with zero gap.
        if self._next_action_index >= chunk_len:
            self.get_logger().info(f"Chunk {self._chunks_sent} finished — checking prefetch buffer")
            with self._prefetch_lock:
                prefetched = self._prefetched_actions
                if prefetched is not None:
                    self._prefetched_actions = None

            self._pending_actions = None
            self._next_action_index = 0
            self._chunk_active = False

            if prefetched is not None:
                self.get_logger().info("Prefetch hit — starting next chunk immediately (no gap)")
                self._activate_chunk(prefetched)
            else:
                self.get_logger().warning(
                    "Prefetch miss — waiting for background prediction to complete"
                )
                # The idle _timer_cb will pick up when the prefetch lands.
            return

        # Stream current step.
        row = self._pending_actions[self._next_action_index]
        arm_targets = {
            name: float(value)
            for name, value in zip(self._arm_joint_names, row[: len(self._arm_joint_names)])
        }
        gripper_target = {self._gripper_joint_name: float(row[-1])}
        self._arm_backend._publish_direct_joint_command(arm_targets)
        if not self._arm_only:
            self._gripper_backend._publish_direct_joint_command(gripper_target)
        if self._next_action_index == 0:
            self.get_logger().info(
                f"Streaming chunk {self._chunks_sent} start: "
                f"arm={arm_targets} gripper={gripper_target}"
            )
        self._next_action_index += 1

    def _build_request(self) -> dict | None:
        with self._lock:
            joint_msg = self._latest_joint_state
            images = dict(self._latest_images)
        if joint_msg is None:
            self._log_wait_reason("waiting for joint state")
            return None
        if set(images) != {"camera1", "camera2"}:
            missing_images = sorted({"camera1", "camera2"} - set(images))
            self._log_wait_reason(f"waiting for images: {missing_images}")
            return None

        name_to_pos = {name: pos for name, pos in zip(joint_msg.name, joint_msg.position)}
        missing = [name for name in self._state_joint_names if name not in name_to_pos]
        if missing:
            self._log_wait_reason(f"joint state missing required names: {missing}")
            return None

        self._last_wait_reason = None
        state = [float(name_to_pos[name]) for name in self._state_joint_names]
        return {
            "state": state,
            "camera1": images["camera1"],
            "camera2": images["camera2"],
            "task": self._task,
        }

    def _log_wait_reason(self, reason: str) -> None:
        if reason != self._last_wait_reason:
            self.get_logger().info(reason)
            self._last_wait_reason = reason

    def _request_prediction(self, request: dict) -> dict:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(self._socket_path)
            send_message(sock, request)
            return recv_message(sock)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the ROS-side SmolVLA relay.")
    parser.add_argument("--hardware-type", default="isaac", choices=["isaac", "twin", "real", "fake"])
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--joint-topic", default=None)
    parser.add_argument("--camera1-topic", default=DEFAULT_CAMERA1_TOPIC)
    parser.add_argument("--camera2-topic", default=DEFAULT_CAMERA2_TOPIC)
    parser.add_argument("--action-fps", type=float, default=30.0)
    parser.add_argument("--prediction-period-s", type=float, default=1.67)
    parser.add_argument(
        "--prefetch-fraction",
        type=float,
        default=0.5,
        help="Start fetching the next chunk when this fraction of the current one has been streamed "
             "(0.5 = halfway). Prediction must complete before the chunk ends to avoid a gap.",
    )
    parser.add_argument("--arm-joints", default=",".join(DEFAULT_ARM_JOINTS))
    parser.add_argument("--gripper-joint", default=DEFAULT_GRIPPER_JOINT)
    parser.add_argument("--state-joints", default=",".join(DEFAULT_STATE_JOINTS))
    parser.add_argument("--max-chunks", type=int, default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--arm-only", action="store_true")
    return parser


def main() -> int:
    args, _unknown = build_arg_parser().parse_known_args()
    args.joint_topic = resolve_joint_topic(args.hardware_type, args.joint_topic)
    rclpy.init()
    try:
        node = SmolVLARosClient(args)
    except Exception as exc:
        print(f"Failed to start SmolVLA ROS client: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

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
