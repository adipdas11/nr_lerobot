"""Block-pose randomisation and domain randomisation via IsaacClient."""

from __future__ import annotations

import math
import random
from typing import NamedTuple

from .config import (
    BLOCK_COLOR_PALETTE,
    BLOCK_MIN_SEP,
    BLOCK_PLACE_Z,
    BLOCK_PRIM_PATHS,
    BLOCK_UPRIGHT_TOLERANCE_DEG,
    LIGHT_INTENSITY_RANGE,
    MAX_PLACE_RETRIES,
    WS_X_MAX,
    WS_X_MIN,
    WS_Y_MAX,
    WS_Y_MIN,
)
from .isaac_client import IsaacClient


class BlockPose(NamedTuple):
    x: float
    y: float
    z: float
    yaw_rad: float


def _min_separation_ok(poses: list[BlockPose], candidate: BlockPose) -> bool:
    for p in poses:
        dist = math.hypot(p.x - candidate.x, p.y - candidate.y)
        if dist < BLOCK_MIN_SEP:
            return False
    return True


def _sample_candidate(rng: random.Random, placed: list[BlockPose]) -> BlockPose:
    """Draw a random (x, y, yaw) that satisfies the separation constraint."""
    for _ in range(500):
        x = rng.uniform(WS_X_MIN, WS_X_MAX)
        y = rng.uniform(WS_Y_MIN, WS_Y_MAX)
        yaw = rng.uniform(-math.pi, math.pi)
        candidate = BlockPose(x, y, BLOCK_PLACE_Z, yaw)
        if _min_separation_ok(placed, candidate):
            return candidate
    raise RuntimeError("Could not sample a non-overlapping block position after 500 tries")


class SceneManager:
    """Manages per-episode scene state: block poses and domain randomisation."""

    def __init__(self, client: IsaacClient, seed: int | None = None):
        self._client = client
        self._rng = random.Random(seed)

    # ── physics helpers ──────────────────────────────────────────────────────

    def _step_physics(self, steps: int = 20) -> None:
        """Advance the Isaac Sim physics engine by `steps` frames to let blocks settle."""
        self._client.execute_script(
            f"import omni.kit.app\n"
            f"app = omni.kit.app.get_app()\n"
            f"[app.update() for _ in range({steps})]"
        )

    def _get_block_euler_deg(self, prim_path: str) -> tuple[float, float, float]:
        """Return (roll_deg, pitch_deg, yaw_deg) of the block after physics settling.

        Uses GfRotation.Decompose(Z, Y, X) so the first returned angle is yaw,
        second is pitch, third is roll — matching the Isaac Sim XYZ-Euler convention.
        """
        script = f"""
import omni.usd
from pxr import UsdGeom, Gf
import json

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("{prim_path}")
xf = UsdGeom.Xformable(prim)
ops = xf.GetOrderedXformOps()
rot_op = None
for op in ops:
    if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
        rot_op = op
        break
if rot_op is not None:
    v = rot_op.Get()
    roll, pitch, yaw = float(v[0]), float(v[1]), float(v[2])
else:
    # Fallback: decompose from the local-to-world matrix
    xform = UsdGeom.Xform(prim)
    mat = xform.GetLocalTransformation()
    rot = Gf.Matrix3d(mat).ExtractRotation()
    angles = rot.Decompose(Gf.Vec3d(0,0,1), Gf.Vec3d(0,1,0), Gf.Vec3d(1,0,0))
    yaw, pitch, roll = float(angles[0]), float(angles[1]), float(angles[2])
print(json.dumps([roll, pitch, yaw]))
"""
        result = self._client.execute_script(script)
        import json as _json
        try:
            roll, pitch, yaw = _json.loads(result.strip().splitlines()[-1])
        except Exception:
            roll, pitch, yaw = 0.0, 0.0, 0.0
        return roll, pitch, yaw

    # ── pose randomisation ───────────────────────────────────────────────────

    def randomize_block_poses(self) -> list[BlockPose]:
        """Randomise all three blocks with upright verification.

        For each block: place → step physics → check roll+pitch ≤ tolerance.
        If the block tipped, draw a new random pose (respecting separation from
        already-settled blocks) and retry up to MAX_PLACE_RETRIES times.
        Returns the final settled poses.
        """
        settled: list[BlockPose] = []

        for block_idx, prim_path in enumerate(BLOCK_PRIM_PATHS):
            pose = _sample_candidate(self._rng, settled)

            for attempt in range(MAX_PLACE_RETRIES):
                self._client.transform_object(
                    prim_path,
                    position=[pose.x, pose.y, pose.z],
                    rotation_deg=[0.0, 0.0, math.degrees(pose.yaw_rad)],
                )
                self._step_physics(steps=20)

                roll_deg, pitch_deg, _ = self._get_block_euler_deg(prim_path)
                if abs(roll_deg) <= BLOCK_UPRIGHT_TOLERANCE_DEG and abs(pitch_deg) <= BLOCK_UPRIGHT_TOLERANCE_DEG:
                    break

                # Block tipped — try a new random position
                pose = _sample_candidate(self._rng, settled)
            else:
                # All retries exhausted; use last pose and warn
                print(
                    f"WARNING: block {block_idx + 1} still tipped after {MAX_PLACE_RETRIES} retries "
                    f"(roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°) — keeping last placement"
                )

            settled.append(pose)

        return settled

    def get_block_pose(self, block_idx: int) -> BlockPose:
        """Read the current pose of block_idx (0-based) from the USD stage."""
        prim_path = BLOCK_PRIM_PATHS[block_idx]
        pos, yaw = self._client.get_block_pose(prim_path)
        return BlockPose(x=pos[0], y=pos[1], z=pos[2], yaw_rad=yaw)

    # ── domain randomisation ─────────────────────────────────────────────────

    def apply_domain_randomization(self) -> None:
        """Randomise block colours and scene lighting."""
        self._randomize_block_colors()
        self._randomize_lighting()

    def _randomize_block_colors(self) -> None:
        color_cmds: list[str] = []
        for idx, prim_path in enumerate(BLOCK_PRIM_PATHS):
            r, g, b = self._rng.choice(BLOCK_COLOR_PALETTE)
            # Add noise so the same palette entry isn't identical every episode
            r = max(0.0, min(1.0, r + self._rng.uniform(-0.08, 0.08)))
            g = max(0.0, min(1.0, g + self._rng.uniform(-0.08, 0.08)))
            b = max(0.0, min(1.0, b + self._rng.uniform(-0.08, 0.08)))
            mat_path = f"/World/DataCollectionMat{idx}"
            color_cmds.append(
                f"""
stage = omni.usd.get_context().get_stage()
from pxr import UsdShade, Sdf, Gf
mat_path = "{mat_path}"
mat = UsdShade.Material.Define(stage, mat_path)
shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f({r:.3f}, {g:.3f}, {b:.3f}))
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set({self._rng.uniform(0.3, 0.9):.2f})
mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
prim = stage.GetPrimAtPath("{prim_path}")
if prim:
    UsdShade.MaterialBindingAPI(prim).Bind(mat)
"""
            )
        self._client.execute_script("\n".join(color_cmds))

    def _randomize_lighting(self) -> None:
        intensity_scale = self._rng.uniform(*LIGHT_INTENSITY_RANGE)
        # Warm/cool colour tint: interpolate between warm (1,0.85,0.7) and cool (0.85,0.9,1)
        t = self._rng.random()
        lr = 1.0 * (1 - t) + 0.85 * t
        lg = 0.85 * (1 - t) + 0.90 * t
        lb = 0.7 * (1 - t) + 1.0 * t
        script = f"""
stage = omni.usd.get_context().get_stage()
from pxr import UsdLux, Gf
for prim in stage.Traverse():
    type_name = prim.GetTypeName()
    if "Light" in type_name:
        light = UsdLux.BoundableLightBase(prim)
        try:
            attr = prim.GetAttribute("inputs:intensity")
            if attr:
                base = attr.Get() or 1000.0
                attr.Set(float(base) * {intensity_scale:.3f})
            color_attr = prim.GetAttribute("inputs:color")
            if color_attr:
                color_attr.Set(Gf.Vec3f({lr:.3f}, {lg:.3f}, {lb:.3f}))
        except Exception:
            pass
"""
        self._client.execute_script(script)

    def reset_lighting(self) -> None:
        """Restore lighting to neutral (1.0 intensity, white colour)."""
        script = """
stage = omni.usd.get_context().get_stage()
from pxr import UsdLux, Gf
for prim in stage.Traverse():
    if "Light" in prim.GetTypeName():
        try:
            attr = prim.GetAttribute("inputs:intensity")
            if attr:
                attr.Set(1000.0)
            color_attr = prim.GetAttribute("inputs:color")
            if color_attr:
                color_attr.Set(Gf.Vec3f(1.0, 1.0, 1.0))
        except Exception:
            pass
"""
        self._client.execute_script(script)
