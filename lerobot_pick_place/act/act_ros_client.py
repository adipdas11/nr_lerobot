#!/usr/bin/env python3
"""Python 3.10 ROS relay for ACT policy inference.

Subscribes to joint states and compressed camera images, batches them into
an observation, sends it to the ACT policy server, and streams the returned
action chunk at a fixed rate. Uses background prefetching to eliminate
pauses between consecutive chunks.
"""

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
from act_ipc import recv_message, send_message


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SOCKET_PATH = "/tmp/act_policy.sock"
DEFAULT_CAMERA1_TOPIC = "/camera1/realsense_camera/color/image_raw/compressed"
DEFAULT_CAMERA2_TOPIC = "/camera2/realsense_camera/color/image_raw/compressed"

DEFAULT_ARM_JOINTS = (
    "xarm5_joint1", "xarm5_joint2", "xarm5_joint3",
    "xarm5_joint4", "xarm5_joint5",
)
DEFAULT_STATE_JOINTS = DEFAULT_ARM_JOINTS + ("xarm_gripper_right_drive_joint",)
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


class ACTRosClient(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("act_ros_client")
        self._lock = threading.Lock()
        self._socket_path = args.socket_path
        self._dry_run = args.dry_run
        self._action_fps = float(args.action_fps)
        self._prediction_period_s = float(args.prediction_period_s)
        self._arm_only = args.arm_only
        self._state_joint_names = parse_csv_list(args.state_joints)
        self._arm_joint_names = parse_csv_list(args.arm_joints)
        self._gripper_joint_name = args.gripper_joint
        self._latest_joint_state: JointState | None = None
        self._latest_images: dict[str, bytes] = {}

        # Active chunk.
        self._chunk_active = False
        self._pending_actions: np.ndarray | None = None
        self._next_action_index = 0
        self._chunks_sent = 0
        self._max_chunks = args.max_chunks
        self._last_wait_reason: str | None = None

        # Prefetch state.
        self._prefetch_lock = threading.Lock()
        self._prefetch_in_flight = False
        self._prefetched_actions: np.ndarray | None = None

        self.declare_parameter("hardware_type", args.hardware_type)
        self._arm_backend = MotionBackend(self, "xarm5_arm_no_slide", defer_exotica_init=True)
        self._gripper_backend = MotionBackend(self, "xarm_gripper", defer_exotica_init=True)

        self.create_subscription(JointState, args.joint_topic, self._joint_state_cb, 10)
        self.create_subscription(
            CompressedImage, args.camera1_topic,
            lambda msg: self._image_cb("camera1", msg), 5,
        )
        self.create_subscription(
            CompressedImage, args.camera2_topic,
            lambda msg: self._image_cb("camera2", msg), 5,
        )
        self._timer = self.create_timer(self._prediction_period_s, self._timer_cb)
        self._stream_timer = self.create_timer(1.0 / self._action_fps, self._stream_timer_cb)

        self.get_logger().info(f"ACT socket path: {self._socket_path}")
        self.get_logger().info(f"Joint topic: {args.joint_topic}")
        self.get_logger().info("Prediction mode: fresh prediction after each chunk (no mid-chunk prefetch)")
        if self._dry_run:
            self.get_logger().info("Dry-run mode: commands will NOT be sent to the robot.")
        if self._arm_only:
            self.get_logger().info("Arm-only mode: gripper commands suppressed.")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_state = msg

    def _image_cb(self, camera_name: str, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_images[camera_name] = bytes(msg.data)

    # ------------------------------------------------------------------
    # Prediction helpers
    # ------------------------------------------------------------------

    def _build_request(self) -> dict | None:
        with self._lock:
            joint_msg = self._latest_joint_state
            images = dict(self._latest_images)

        if joint_msg is None:
            self._log_wait("waiting for joint state")
            return None
        if set(images) != {"camera1", "camera2"}:
            missing = sorted({"camera1", "camera2"} - set(images))
            self._log_wait(f"waiting for images: {missing}")
            return None

        name_to_pos = {name: pos for name, pos in zip(joint_msg.name, joint_msg.position)}
        missing_joints = [n for n in self._state_joint_names if n not in name_to_pos]
        if missing_joints:
            self._log_wait(f"joint state missing: {missing_joints}")
            return None

        self._last_wait_reason = None
        state = [float(name_to_pos[n]) for n in self._state_joint_names]
        # ACT has no task string — omit from request.
        return {"state": state, "camera1": images["camera1"], "camera2": images["camera2"]}

    def _log_wait(self, reason: str) -> None:
        if reason != self._last_wait_reason:
            self.get_logger().info(reason)
            self._last_wait_reason = reason

    def _fetch_prediction(self) -> np.ndarray | None:
        request = self._build_request()
        if request is None:
            return None
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(self._socket_path)
                send_message(sock, request)
                response = recv_message(sock)
        except Exception as exc:
            self.get_logger().error(f"ACT server request failed: {exc}")
            return None
        if not response.get("ok", False):
            self.get_logger().error(f"ACT server error: {response.get('error')}")
            return None
        return np.asarray(response["actions"], dtype=np.float32)

    def _start_prefetch(self) -> None:
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
                        f"Prefetch ready: chunk shape {actions.shape} "
                        f"arm=[{float(actions[:, :len(self._arm_joint_names)].min()):.3f}, "
                        f"{float(actions[:, :len(self._arm_joint_names)].max()):.3f}] "
                        f"grip=[{float(actions[:, -1].min()):.3f}, "
                        f"{float(actions[:, -1].max()):.3f}]"
                    )
                else:
                    self.get_logger().warning("Prefetch returned no actions — will retry.")

        threading.Thread(target=_worker, daemon=True).start()

    def _activate_chunk(self, actions: np.ndarray) -> None:
        self._chunks_sent += 1
        arm_min = float(actions[:, : len(self._arm_joint_names)].min())
        arm_max = float(actions[:, : len(self._arm_joint_names)].max())
        grip_min = float(actions[:, -1].min())
        grip_max = float(actions[:, -1].max())

        if self._dry_run:
            self.get_logger().info(
                f"[DRY-RUN] Chunk {self._chunks_sent}: shape={actions.shape} "
                f"arm=[{arm_min:.4f}, {arm_max:.4f}] grip=[{grip_min:.4f}, {grip_max:.4f}]"
            )
            self._pending_actions = actions
            self._next_action_index = 0
            self._chunk_active = True
            return

        self.get_logger().info(
            f"Activating chunk {self._chunks_sent}: shape={actions.shape} "
            f"arm=[{arm_min:.4f}, {arm_max:.4f}] grip=[{grip_min:.4f}, {grip_max:.4f}]"
        )

        # Publish the full chunk as a single JointTrajectory so the controller
        # receives the complete motion plan in one message.  Sending individual
        # single-point trajectories at 30 Hz causes Isaac Sim's JTC to queue
        # them rather than execute immediately, so motion would be deferred
        # until the publisher disconnects.
        n_arm = len(self._arm_joint_names)
        arm_positions = [list(row[:n_arm]) for row in actions]
        self._arm_backend.publish_action_chunk(
            self._arm_joint_names, arm_positions, self._action_fps
        )
        if not self._arm_only:
            grip_positions = [[float(row[-1])] for row in actions]
            self._gripper_backend.publish_action_chunk(
                [self._gripper_joint_name], grip_positions, self._action_fps
            )

        self.get_logger().info(
            f"Chunk {self._chunks_sent} sent: arm[0]="
            + str({n: f"{v:.3f}" for n, v in zip(self._arm_joint_names, arm_positions[0])})
        )

        self._pending_actions = actions
        self._next_action_index = 0
        self._chunk_active = True

    # ------------------------------------------------------------------
    # Timer callbacks
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        """Idle timer: kick off the first chunk or recover after a prefetch miss."""
        if self._chunk_active:
            return
        if self._max_chunks is not None and self._chunks_sent >= self._max_chunks:
            self.get_logger().info("Reached max_chunks — stopping.")
            self._timer.cancel()
            return

        with self._prefetch_lock:
            prefetched = self._prefetched_actions
            if prefetched is not None:
                self._prefetched_actions = None
        if prefetched is not None:
            self._activate_chunk(prefetched)
            return

        with self._prefetch_lock:
            if self._prefetch_in_flight:
                return

        actions = self._fetch_prediction()
        if actions is None:
            return
        self._activate_chunk(actions)

    def _stream_timer_cb(self) -> None:
        # When idle, poll at stream rate for a completed prediction and
        # activate it as soon as it arrives.  This gives a gap of only
        # ~inference-latency (~0.5-2 s) instead of the full _timer period.
        if not self._chunk_active or self._pending_actions is None:
            if self._max_chunks is not None and self._chunks_sent >= self._max_chunks:
                return
            with self._prefetch_lock:
                prefetched = self._prefetched_actions
                if prefetched is not None:
                    self._prefetched_actions = None
            if prefetched is not None:
                self._activate_chunk(prefetched)
            return

        chunk_len = len(self._pending_actions)

        # Chunk exhausted — request a fresh prediction immediately.
        if self._next_action_index >= chunk_len:
            self.get_logger().info(f"Chunk {self._chunks_sent} finished")
            self._pending_actions = None
            self._next_action_index = 0
            self._chunk_active = False
            # Discard any prefetch that was taken from a mid-chunk observation.
            # Reusing it would make the robot move backward to that earlier
            # position, causing the "backing-off" jerk between chunks.
            with self._prefetch_lock:
                self._prefetched_actions = None
            # Start a fresh prediction from the robot's current (end-of-chunk)
            # state.  _stream_timer_cb will activate it within one poll cycle
            # (~33 ms) of it arriving.
            if self._max_chunks is None or self._chunks_sent < self._max_chunks:
                self._start_prefetch()
            return

        # Advance progress counter (the full trajectory was already published
        # in _activate_chunk; this counter drives chunk-end detection only).
        self._next_action_index += 1


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ACT ROS relay for inference")
    parser.add_argument("--hardware-type", default="real",
                        choices=["isaac", "twin", "real", "fake"])
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET_PATH)
    parser.add_argument("--joint-topic", default=None)
    parser.add_argument("--camera1-topic", default=DEFAULT_CAMERA1_TOPIC)
    parser.add_argument("--camera2-topic", default=DEFAULT_CAMERA2_TOPIC)
    parser.add_argument("--action-fps", type=float, default=30.0,
                        help="Rate at which action steps are streamed to the robot (Hz)")
    parser.add_argument("--prediction-period-s", type=float, default=3.33,
                        help="Timer period for the initial/recovery prediction (100 steps / 30 Hz)")
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
        node = ACTRosClient(args)
    except Exception as exc:
        print(f"Failed to start ACT ROS client: {exc}", file=sys.stderr)
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
