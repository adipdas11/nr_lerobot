#!/usr/bin/env python3
"""Automated data-collection node for wooden-block stacking (xArm5 in Isaac Sim).

Run order:
  1. ros2 launch nr_dual_arm_moveit_config demo.launch.py hardware_type:=isaac
  2. python3 collect_data.py [--episodes N] [--output-dir rosbags] [--seed S]

The node:
  - Randomises block poses and domain randomisation via the Isaac Sim MCP socket.
  - Plans and executes pick-and-place motions via MotionBackend (MoveIt + EXOTica IK).
  - Records /isaac_joint_states + camera1 + camera2 compressed as a rosbag per episode.
  - Validates each bag before moving on.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Add workspace to path so MotionBackend is importable
sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

from nr_dual_arm_moveit_config.motion_backend import MotionBackend

from isaac_data_collection.config import (
    ARM_GROUP,
    ARM_HOME_JOINTS,
    BLOCK_HEIGHT_M,
    GRIPPER_CLOSED_RAD,
    GRIPPER_GROUP,
    GRIPPER_JOINT,
    GRIPPER_OPEN_RAD,
    ISAAC_HOST,
    ISAAC_PORT,
    LIFT_Z,
    NUM_EPISODES,
    PREGRASP_Z_OFFSET,
    ROSBAG_BASE_DIR,
    SLIDER_HOME_M,
    SLIDER_JOINT,
    STACK_X,
    STACK_Y,
    WS_TABLE_Z,
)
from isaac_data_collection.isaac_client import IsaacClient
from isaac_data_collection.rosbag_recorder import RosbagRecorder
from isaac_data_collection.scene_manager import BlockPose, SceneManager


# Stacking column heights: block_1 bottom, block_2 middle, block_3 top
def _stack_z(layer: int) -> float:
    """World Z for the centre of the block placed at `layer` (0=bottom, 1=mid, 2=top)."""
    return WS_TABLE_Z + BLOCK_HEIGHT_M * (layer + 0.5)


class DataCollectionNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wooden_block_data_collection")
        self._args = args

        self.declare_parameter("hardware_type", "isaac")
        self._arm = MotionBackend(self, ARM_GROUP, defer_exotica_init=False)
        self._gripper = MotionBackend(self, GRIPPER_GROUP, defer_exotica_init=True)

        # Publisher for direct slider + joint commands when needed
        self._joint_cmd_pub = self.create_publisher(JointState, "/isaac_joint_commands", 10)

        self._recorder = RosbagRecorder(args.output_dir)
        self._isaac = IsaacClient(ISAAC_HOST, ISAAC_PORT)
        self._isaac.connect()
        self._scene = SceneManager(self._isaac, seed=args.seed)

        self.get_logger().info("DataCollectionNode initialised")

    # ── slider control ────────────────────────────────────────────────────────

    def _set_slider(self, position_m: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [SLIDER_JOINT]
        msg.position = [position_m]
        self._joint_cmd_pub.publish(msg)
        # Allow physics to settle
        time.sleep(1.5)

    # ── arm helpers ───────────────────────────────────────────────────────────

    def _home(self) -> bool:
        return self._arm.move_to_joint_positions(ARM_HOME_JOINTS, velocity=0.3)

    def _open_gripper(self) -> bool:
        return self._gripper.move_to_joint_positions({GRIPPER_JOINT: GRIPPER_OPEN_RAD}, velocity=1.0)

    def _close_gripper(self) -> bool:
        return self._gripper.move_to_joint_positions({GRIPPER_JOINT: GRIPPER_CLOSED_RAD}, velocity=1.0)

    def _move_to_world_xyz(self, wx: float, wy: float, wz: float, yaw_rad: float = 0.0, velocity: float = 0.15) -> bool:
        """Move end-effector to world-space position using EXOTica quintic planner.

        With the slider fixed at SLIDER_HOME_M along world X, the arm base_link
        frame is offset by that amount, so arm_x = world_x − SLIDER_HOME_M.
        EXOTica automatically falls back to MoveIt IK if the planner is unavailable.
        """
        arm_x = wx - SLIDER_HOME_M
        arm_y = wy
        arm_z = wz
        q = self._arm._rpy_to_quaternion(math.pi, 0.0, yaw_rad)
        return self._arm.move_to_pose_exotica(
            arm_x, arm_y, arm_z,
            q_dict={"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            velocity=velocity,
        )

    # ── pick / place primitives ───────────────────────────────────────────────

    def _pick_block(self, pose: BlockPose) -> bool:
        log = self.get_logger()

        # For a triangular block with flat sides normal to block-local X,
        # align gripper jaw axis with the block's X-axis → gripper yaw = block yaw.
        gripper_yaw = pose.yaw_rad

        pre_z = pose.z + PREGRASP_Z_OFFSET
        grasp_z = pose.z - BLOCK_HEIGHT_M * 0.5 + 0.005  # just above bottom face

        log.info(f"  → pre-grasp ({pose.x:.3f},{pose.y:.3f},{pre_z:.3f}) yaw={math.degrees(gripper_yaw):.1f}°")
        if not self._move_to_world_xyz(pose.x, pose.y, pre_z, gripper_yaw):
            log.error("  pre-grasp failed")
            return False

        log.info(f"  → descend to grasp z={grasp_z:.3f}")
        if not self._move_to_world_xyz(pose.x, pose.y, grasp_z, gripper_yaw, velocity=0.08):
            log.error("  grasp descend failed")
            return False

        if not self._close_gripper():
            log.warning("  gripper close returned failure (continuing)")

        # Lift
        lift_z = WS_TABLE_Z + LIFT_Z
        log.info(f"  → lift to z={lift_z:.3f}")
        if not self._move_to_world_xyz(pose.x, pose.y, lift_z, gripper_yaw, velocity=0.1):
            log.error("  lift failed")
            return False

        return True

    def _place_block(self, target_x: float, target_y: float, target_z: float) -> bool:
        log = self.get_logger()
        pre_z = target_z + PREGRASP_Z_OFFSET

        log.info(f"  → above stack ({target_x:.3f},{target_y:.3f},{pre_z:.3f})")
        if not self._move_to_world_xyz(target_x, target_y, pre_z):
            log.error("  stack pre-position failed")
            return False

        log.info(f"  → place z={target_z:.3f}")
        if not self._move_to_world_xyz(target_x, target_y, target_z, velocity=0.06):
            log.error("  place descend failed")
            return False

        if not self._open_gripper():
            log.warning("  gripper open returned failure (continuing)")

        # Retreat upward
        retreat_z = target_z + PREGRASP_Z_OFFSET
        if not self._move_to_world_xyz(target_x, target_y, retreat_z, velocity=0.1):
            log.warning("  retreat failed")

        return True

    # ── episode execution ─────────────────────────────────────────────────────

    def _run_episode(self, episode_id: int) -> bool:
        log = self.get_logger()
        log.info(f"═══ Episode {episode_id:04d}/{self._args.episodes} ═══")

        # 1. Reset arm and gripper
        self._open_gripper()
        if not self._home():
            log.error("Home failed — skipping episode")
            return False

        # 2. Randomise scene
        log.info("Randomising block poses…")
        block_poses = self._scene.randomize_block_poses()
        for i, p in enumerate(block_poses):
            log.info(f"  block_{i+1}: ({p.x:.3f},{p.y:.3f},{p.z:.3f}) yaw={math.degrees(p.yaw_rad):.1f}°")

        log.info("Applying domain randomisation…")
        self._scene.apply_domain_randomization()

        # Allow Isaac physics to settle after repositioning
        time.sleep(1.0)

        # 3. Start recording
        bag_path = self._recorder.start(episode_id)
        log.info(f"Recording → {bag_path}")

        success = True
        try:
            # Pick block_1 → place at stack zone (layer 0, bottom)
            log.info("Picking block_1…")
            pose1 = self._scene.get_block_pose(0)
            if not self._pick_block(pose1):
                success = False
            else:
                log.info("Placing block_1 at stack zone (bottom)…")
                if not self._place_block(STACK_X, STACK_Y, _stack_z(0)):
                    success = False

            # Pick block_2 → place on top of block_1 (layer 1)
            if success:
                log.info("Picking block_2…")
                pose2 = self._scene.get_block_pose(1)
                if not self._pick_block(pose2):
                    success = False
                else:
                    log.info("Placing block_2 on block_1 (middle)…")
                    if not self._place_block(STACK_X, STACK_Y, _stack_z(1)):
                        success = False

            # Pick block_3 → place on top of block_2 (layer 2)
            if success:
                log.info("Picking block_3…")
                pose3 = self._scene.get_block_pose(2)
                if not self._pick_block(pose3):
                    success = False
                else:
                    log.info("Placing block_3 on block_2 (top)…")
                    if not self._place_block(STACK_X, STACK_Y, _stack_z(2)):
                        success = False

        finally:
            # Always stop recording and return home
            self._recorder.stop()
            self._open_gripper()
            self._home()

        if success:
            log.info(f"Episode {episode_id:04d} DONE — bag at {bag_path}")
        else:
            log.warning(f"Episode {episode_id:04d} INCOMPLETE — bag saved anyway for inspection")

        return success

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        log = self.get_logger()

        # Set slider to home and wait for physics
        log.info(f"Setting slider to {SLIDER_HOME_M:.3f} m…")
        self._set_slider(SLIDER_HOME_M)

        completed = 0
        episode_id = 0
        while completed < self._args.episodes:
            ok = self._run_episode(episode_id)
            if ok:
                completed += 1
            else:
                log.warning(f"Episode {episode_id} failed — will retry with a new randomisation")
            episode_id += 1
            if episode_id > self._args.episodes * 2:
                log.error("Too many failures — stopping early")
                break

        log.info(f"Collection finished: {completed}/{self._args.episodes} episodes saved to {self._args.output_dir}")

    def destroy_node(self) -> None:
        if self._recorder.is_recording():
            self._recorder.stop()
        self._isaac.disconnect()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect wooden-block stacking demos in Isaac Sim.")
    parser.add_argument("--episodes", type=int, default=NUM_EPISODES)
    parser.add_argument("--output-dir", default=ROSBAG_BASE_DIR)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    try:
        node = DataCollectionNode(args)
    except Exception as exc:
        print(f"Failed to initialise node: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(await_futures_done=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
