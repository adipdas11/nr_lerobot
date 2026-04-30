#!/usr/bin/env python3
"""Verify block randomisation by moving blocks to random poses in Isaac Sim.

Run this BEFORE full data collection to confirm:
  - Blocks land inside the workspace rectangle
  - Minimum separation is respected
  - Yaw randomisation looks correct
  - Block positions are reachable (visual check in Isaac Sim viewport)

Usage:
  python3 verify_randomisation.py [--steps N] [--delay S] [--seed SEED]

Isaac Sim must be open with the scene loaded and MCP extension enabled.
No ROS or MoveIt needed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from isaac_data_collection.config import (
    BLOCK_MIN_SEP,
    BLOCK_PRIM_PATHS,
    ISAAC_HOST,
    ISAAC_PORT,
    WS_TABLE_Z,
    WS_X_MAX,
    WS_X_MIN,
    WS_Y_MAX,
    WS_Y_MIN,
)
from isaac_data_collection.isaac_client import IsaacClient
from isaac_data_collection.scene_manager import SceneManager


# ── visual workspace marker ───────────────────────────────────────────────────

MARKER_SCRIPT = f"""
import omni.usd
from pxr import UsdGeom, Gf, Usd
stage = omni.usd.get_context().get_stage()

corners = [
    ({WS_X_MIN:.3f}, {WS_Y_MIN:.3f}, {WS_TABLE_Z:.3f}),
    ({WS_X_MAX:.3f}, {WS_Y_MIN:.3f}, {WS_TABLE_Z:.3f}),
    ({WS_X_MAX:.3f}, {WS_Y_MAX:.3f}, {WS_TABLE_Z:.3f}),
    ({WS_X_MIN:.3f}, {WS_Y_MAX:.3f}, {WS_TABLE_Z:.3f}),
]

# Remove old markers
for i in range(4):
    p = stage.GetPrimAtPath(f"/VerifyMarker_{{i}}")
    if p:
        stage.RemovePrim(f"/VerifyMarker_{{i}}")

# Place small spheres at the four corners
from pxr import UsdShade, Sdf
for i, (cx, cy, cz) in enumerate(corners):
    path = f"/VerifyMarker_{{i}}"
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.GetRadiusAttr().Set(0.015)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz + 0.03))
    # Colour them yellow
    mat = UsdShade.Material.Define(stage, path + "/Mat")
    shader = UsdShade.Shader.Define(stage, path + "/Mat/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.9, 0.0))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(sphere).Bind(mat)
print("Workspace corner markers placed")
"""

CLEAR_MARKER_SCRIPT = """
import omni.usd
stage = omni.usd.get_context().get_stage()
for i in range(4):
    p = stage.GetPrimAtPath(f"/VerifyMarker_{i}")
    if p:
        stage.RemovePrim(f"/VerifyMarker_{i}")
"""


def _in_bounds(x: float, y: float) -> bool:
    return WS_X_MIN <= x <= WS_X_MAX and WS_Y_MIN <= y <= WS_Y_MAX


def _separation_ok(poses: list[tuple]) -> bool:
    for i, (x1, y1, *_) in enumerate(poses):
        for x2, y2, *_ in poses[i + 1:]:
            if math.hypot(x1 - x2, y1 - y2) < BLOCK_MIN_SEP:
                return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify block randomisation in Isaac Sim.")
    parser.add_argument("--steps",   type=int,   default=10,  help="Number of randomisation steps")
    parser.add_argument("--delay",   type=float, default=2.0, help="Seconds between each step")
    parser.add_argument("--seed",    type=int,   default=None)
    parser.add_argument("--no-markers", action="store_true", help="Skip placing corner-sphere markers")
    args = parser.parse_args()

    print(f"Connecting to Isaac Sim MCP at {ISAAC_HOST}:{ISAAC_PORT}…")
    client = IsaacClient(ISAAC_HOST, ISAAC_PORT)
    try:
        client.connect(retries=5, delay=1.0)
    except ConnectionError as exc:
        print(f"ERROR: {exc}")
        print("  Make sure Isaac Sim is open and the MCP extension is enabled.")
        return 1
    print("Connected.\n")

    # Place corner markers so you can see the workspace boundary in the viewport
    if not args.no_markers:
        print("Placing yellow corner markers at workspace boundary…")
        client.execute_script(MARKER_SCRIPT)
        print(f"  X: [{WS_X_MIN:.2f}, {WS_X_MAX:.2f}]  "
              f"Y: [{WS_Y_MIN:.2f}, {WS_Y_MAX:.2f}]  "
              f"Z (table): {WS_TABLE_Z:.3f}\n")

    scene = SceneManager(client, seed=args.seed)

    pass_count = 0
    fail_count = 0

    try:
        for step in range(1, args.steps + 1):
            print(f"── Step {step}/{args.steps} {'─'*40}")
            poses = scene.randomize_block_poses()

            all_ok = True
            for idx, pose in enumerate(poses):
                in_b = _in_bounds(pose.x, pose.y)
                status = "✓" if in_b else "✗ OUT-OF-BOUNDS"
                print(f"  block_{idx+1}: "
                      f"x={pose.x:.3f}  y={pose.y:.3f}  z={pose.z:.3f}  "
                      f"yaw={math.degrees(pose.yaw_rad):+.1f}°  {status}")
                if not in_b:
                    all_ok = False

            sep_ok = _separation_ok(poses)
            print(f"  separation ≥ {BLOCK_MIN_SEP:.3f} m: {'✓' if sep_ok else '✗ VIOLATION'}")
            if not sep_ok:
                all_ok = False

            if all_ok:
                pass_count += 1
            else:
                fail_count += 1

            if step < args.steps:
                print(f"  (waiting {args.delay:.1f}s — inspect in viewport)")
                time.sleep(args.delay)

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        if not args.no_markers:
            client.execute_script(CLEAR_MARKER_SCRIPT)
        client.disconnect()

    total = pass_count + fail_count
    print(f"\n{'═'*50}")
    print(f"Result: {pass_count}/{total} steps passed  ({fail_count} failures)")
    if fail_count == 0:
        print("✓ Workspace bounds and separation constraints are satisfied.")
        print("  Visually confirm in the Isaac Sim viewport that blocks landed")
        print("  on the table surface and are within the yellow-marker rectangle.")
    else:
        print("✗ Some poses violated bounds or separation — adjust config.py.")
    print(f"  WS_X: [{WS_X_MIN}, {WS_X_MAX}]  WS_Y: [{WS_Y_MIN}, {WS_Y_MAX}]")
    print(f"  BLOCK_MIN_SEP: {BLOCK_MIN_SEP}")

    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
