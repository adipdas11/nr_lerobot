#!/usr/bin/env python3
"""Interactive grasp test for wooden-block stacking.

Drives the xArm5 through the full pick sequence for one or all blocks, then
checks whether each block actually left the table by querying its Z from Isaac Sim.

Usage:
  python3 test_grasp.py                   # test block 1 at current pose
  python3 test_grasp.py --block 2         # test block 2
  python3 test_grasp.py --all             # test all 3 blocks in sequence
  python3 test_grasp.py --randomise       # randomise block poses first
  python3 test_grasp.py --all --randomise --seed 7
  python3 test_grasp.py --no-lift         # grasp only, stay at grasp height (visual check)

Prerequisites (separate terminals):
  Terminal 1 — Isaac Sim open with nr_dual_arm_scene_wooden_blocks.usd + MCP extension
  Terminal 2 — ros2 launch nr_dual_arm_moveit_config demo.launch.py hardware_type:=isaac
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

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from nr_dual_arm_moveit_config.motion_backend import MotionBackend

sys.path.insert(0, str(Path(__file__).resolve().parent))
from isaac_data_collection.config import (
    ARM_GROUP,
    ARM_HOME_JOINTS,
    BLOCK_HEIGHT_M,
    BLOCK_PRIM_PATHS,
    GRIPPER_CLOSED_RAD,
    GRIPPER_GROUP,
    GRIPPER_JOINT,
    GRIPPER_OPEN_RAD,
    ISAAC_HOST,
    ISAAC_PORT,
    LIFT_Z,
    PREGRASP_Z_OFFSET,
    SLIDER_HOME_M,
    SLIDER_JOINT,
    WS_TABLE_Z,
)
from isaac_data_collection.isaac_client import IsaacClient
from isaac_data_collection.scene_manager import BlockPose, SceneManager


# ── helpers ───────────────────────────────────────────────────────────────────

_LINE = "─" * 60

def _sep(title: str = "") -> str:
    return f"\n{_LINE}\n  {title}\n{_LINE}" if title else f"\n{_LINE}"


def _block_z_from_isaac(client: IsaacClient, prim_path: str) -> float | None:
    """Read the current world Z of a block prim (returns None on error)."""
    try:
        info = client.get_prim_info(prim_path)
        return info.get("transform", {}).get("position", [None, None, None])[2]
    except Exception:
        return None


# ── ROS node ──────────────────────────────────────────────────────────────────

class GraspTestNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("wooden_block_grasp_test")
        self._args = args

        self.declare_parameter("hardware_type", "isaac")
        self._arm = MotionBackend(self, ARM_GROUP, defer_exotica_init=False)
        self._gripper = MotionBackend(self, GRIPPER_GROUP, defer_exotica_init=True)

        self._joint_cmd_pub = self.create_publisher(JointState, "/isaac_joint_commands", 10)

        self._isaac = IsaacClient(ISAAC_HOST, ISAAC_PORT)
        self._isaac.connect(retries=5)
        self._scene = SceneManager(self._isaac, seed=args.seed)

        self.get_logger().info("GraspTestNode ready")

    # ── low-level motion ─────────────────────────────────────────────────────

    def _set_slider(self, pos_m: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [SLIDER_JOINT]
        msg.position = [pos_m]
        self._joint_cmd_pub.publish(msg)
        time.sleep(1.5)

    def _home(self) -> bool:
        return self._arm.move_to_joint_positions(ARM_HOME_JOINTS, velocity=0.3)

    def _open_gripper(self) -> bool:
        return self._gripper.move_to_joint_positions({GRIPPER_JOINT: GRIPPER_OPEN_RAD}, velocity=1.0)

    def _close_gripper(self) -> bool:
        return self._gripper.move_to_joint_positions({GRIPPER_JOINT: GRIPPER_CLOSED_RAD}, velocity=1.0)

    def _move_to_world_xyz(
        self,
        wx: float,
        wy: float,
        wz: float,
        yaw_rad: float = 0.0,
        velocity: float = 0.15,
    ) -> bool:
        arm_x = wx - SLIDER_HOME_M
        arm_y = wy
        arm_z = wz
        q = self._arm._rpy_to_quaternion(math.pi, 0.0, yaw_rad)
        return self._arm.move_to_pose_exotica(
            arm_x, arm_y, arm_z,
            q_dict={"qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w},
            velocity=velocity,
        )

    # ── grasp test for one block ──────────────────────────────────────────────

    def _test_block(self, block_idx: int) -> bool:
        """Execute and evaluate a full grasp sequence for block `block_idx` (0-based).

        Returns True if the block was successfully lifted above the table.
        """
        log = self.get_logger()
        prim_path = BLOCK_PRIM_PATHS[block_idx]
        label = f"block_{block_idx + 1}"

        log.info(_sep(f"Testing grasp: {label}  ({prim_path})"))

        # ── 1. Read current block pose ───────────────────────────────────────
        pose = self._scene.get_block_pose(block_idx)
        z_before = _block_z_from_isaac(self._isaac, prim_path)

        log.info(
            f"  Pose   : x={pose.x:.3f}  y={pose.y:.3f}  z={pose.z:.3f}  "
            f"yaw={math.degrees(pose.yaw_rad):+.1f}°"
        )
        log.info(f"  Z (raw): {z_before:.4f} m  (table surface ≈ {WS_TABLE_Z:.3f} m)")

        gripper_yaw = pose.yaw_rad

        # ── 2. Open gripper and move to pregrasp ────────────────────────────
        self._open_gripper()

        pre_z = pose.z + PREGRASP_Z_OFFSET
        log.info(f"  [1/5] Pre-grasp  ({pose.x:.3f}, {pose.y:.3f}, {pre_z:.3f})  yaw={math.degrees(gripper_yaw):+.1f}°")
        if not self._move_to_world_xyz(pose.x, pose.y, pre_z, gripper_yaw, velocity=0.2):
            log.error(f"  FAIL: could not reach pre-grasp for {label}")
            return False
        log.info("  [1/5] Pre-grasp  ✓")

        # ── 3. Descend to grasp ──────────────────────────────────────────────
        grasp_z = pose.z - BLOCK_HEIGHT_M * 0.5 + 0.005
        log.info(f"  [2/5] Grasp      ({pose.x:.3f}, {pose.y:.3f}, {grasp_z:.3f})")
        if not self._move_to_world_xyz(pose.x, pose.y, grasp_z, gripper_yaw, velocity=0.08):
            log.error(f"  FAIL: could not descend to grasp for {label}")
            return False
        log.info("  [2/5] Grasp      ✓")

        # ── 4. Close gripper ─────────────────────────────────────────────────
        log.info(f"  [3/5] Close gripper  (target={GRIPPER_CLOSED_RAD:.3f} rad)")
        closed_ok = self._close_gripper()
        log.info(f"  [3/5] Close gripper  {'✓' if closed_ok else '⚠ returned failure (continuing)'}")

        if self._args.no_lift:
            log.info("  --no-lift: stopping here for visual inspection. Press Ctrl-C to abort.")
            try:
                while True:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
            self._open_gripper()
            self._home()
            return True

        # ── 5. Lift ──────────────────────────────────────────────────────────
        lift_z = WS_TABLE_Z + LIFT_Z
        log.info(f"  [4/5] Lift       z={lift_z:.3f}")
        if not self._move_to_world_xyz(pose.x, pose.y, lift_z, gripper_yaw, velocity=0.1):
            log.error(f"  FAIL: lift motion failed for {label}")
            self._open_gripper()
            return False
        log.info("  [4/5] Lift       ✓")

        # Allow Isaac physics to update
        time.sleep(0.5)

        # ── 6. Verify block moved ────────────────────────────────────────────
        z_after = _block_z_from_isaac(self._isaac, prim_path)
        log.info(f"  [5/5] Verify     Z before={z_before:.4f}  Z after={z_after:.4f}")

        lifted_threshold = 0.02   # block must have risen at least 2 cm
        success = z_after is not None and (z_after - z_before) >= lifted_threshold

        if success:
            rise_mm = (z_after - z_before) * 1000
            log.info(f"  ✓ PASS — {label} lifted {rise_mm:.0f} mm off the table")
        else:
            if z_after is None:
                log.error(f"  ✗ FAIL — could not read Z after lift")
            else:
                rise_mm = (z_after - z_before) * 1000
                log.error(
                    f"  ✗ FAIL — {label} only moved {rise_mm:.0f} mm "
                    f"(need ≥ {lifted_threshold * 1000:.0f} mm). "
                    "Gripper may have missed — check GRIPPER_CLOSED_RAD in config.py."
                )

        # ── 7. Return home ───────────────────────────────────────────────────
        self._open_gripper()
        time.sleep(0.3)
        self._home()

        return success

    # ── main test loop ────────────────────────────────────────────────────────

    def run(self) -> int:
        log = self.get_logger()

        # Slider to home
        log.info(f"Setting slider to {SLIDER_HOME_M:.3f} m…")
        self._set_slider(SLIDER_HOME_M)

        # Optionally randomise block poses
        if self._args.randomise:
            log.info("Randomising block poses…")
            poses = self._scene.randomize_block_poses()
            for i, p in enumerate(poses):
                log.info(f"  block_{i + 1}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  yaw={math.degrees(p.yaw_rad):+.1f}°")
            time.sleep(1.0)

        # Determine which blocks to test
        if self._args.all:
            block_indices = [0, 1, 2]
        else:
            block_indices = [self._args.block - 1]   # --block is 1-based

        # Home arm before starting
        log.info("Homing arm…")
        self._open_gripper()
        if not self._home():
            log.error("Home failed — aborting")
            return 1

        results: dict[int, bool] = {}
        for idx in block_indices:
            ok = self._test_block(idx)
            results[idx] = ok

        # ── Summary ─────────────────────────────────────────────────────────
        log.info(_sep("Results"))
        passed = sum(results.values())
        for idx, ok in results.items():
            status = "PASS ✓" if ok else "FAIL ✗"
            log.info(f"  block_{idx + 1} : {status}")
        log.info(f"\n  {passed}/{len(results)} blocks grasped successfully")

        if passed < len(results):
            log.info(
                "\nTroubleshooting hints:"
                "\n  • Gripper closed too loosely → increase GRIPPER_CLOSED_RAD in config.py"
                "\n  • Wrong grasp height         → adjust PREGRASP_Z_OFFSET / BLOCK_HEIGHT_M"
                "\n  • IK/planning failure        → check MoveIt / EXOTica logs above"
                "\n  • Block not at expected pose → re-run with --randomise or check USD stage"
            )

        return 0 if passed == len(results) else 1

    def destroy_node(self) -> None:
        self._isaac.disconnect()
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test xArm5 grasp on wooden blocks in Isaac Sim.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--block", type=int, default=1, choices=[1, 2, 3],
        metavar="{1,2,3}",
        help="Which block to test (default: 1)",
    )
    group.add_argument(
        "--all", action="store_true",
        help="Test all three blocks in sequence",
    )
    parser.add_argument(
        "--randomise", action="store_true",
        help="Randomise block poses before testing",
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="RNG seed for randomisation (optional)",
    )
    parser.add_argument(
        "--no-lift", action="store_true",
        dest="no_lift",
        help="Grasp only — hold at grasp height for visual inspection, don't lift",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    try:
        node = GraspTestNode(args)
    except Exception as exc:
        print(f"Failed to initialise: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    ret = 0
    try:
        ret = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return ret


if __name__ == "__main__":
    raise SystemExit(main())
