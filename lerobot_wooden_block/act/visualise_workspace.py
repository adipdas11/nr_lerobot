#!/usr/bin/env python3
"""Draw the workspace rectangle in the Isaac Sim viewport for fine-tuning.

Creates a semi-transparent green floor plane + corner posts to show exactly
where blocks will be randomised.  Edit WS_X_MIN/MAX and WS_Y_MIN/MAX in
config.py, then re-run this script to see the updated rectangle instantly.

Usage:
  python3 visualise_workspace.py          # draw rectangle
  python3 visualise_workspace.py --clear  # remove all markers

Isaac Sim must be open with MCP extension enabled.  No ROS needed.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from isaac_data_collection.config import (
    BLOCK_MIN_SEP,
    ISAAC_HOST,
    ISAAC_PORT,
    STACK_X,
    STACK_Y,
    WS_TABLE_Z,
    WS_X_MAX,
    WS_X_MIN,
    WS_Y_MAX,
    WS_Y_MIN,
)
from isaac_data_collection.isaac_client import IsaacClient


# ── USD scripts executed inside Isaac Sim ─────────────────────────────────────

def _draw_script() -> str:
    cx = (WS_X_MIN + WS_X_MAX) / 2
    cy = (WS_Y_MIN + WS_Y_MAX) / 2
    sx = WS_X_MAX - WS_X_MIN   # full width
    sy = WS_Y_MAX - WS_Y_MIN   # full depth
    sz = 0.003                  # thin slab height

    return f"""
import omni.usd
from pxr import UsdGeom, UsdShade, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# ── helper: remove then redefine ─────────────────────────────────────────────
def remove(path):
    p = stage.GetPrimAtPath(path)
    if p and p.IsValid():
        stage.RemovePrim(path)

def make_mat(path, r, g, b, opacity=1.0):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, path + "/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
    sh.CreateInput("opacity",      Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

# ── 1. Semi-transparent green floor slab ─────────────────────────────────────
remove("/WS_FloorSlab")
cube = UsdGeom.Cube.Define(stage, "/WS_FloorSlab")
cube.GetSizeAttr().Set(1.0)
xf = UsdGeom.Xformable(cube)
xf.AddTranslateOp().Set(Gf.Vec3d({cx:.4f}, {cy:.4f}, {WS_TABLE_Z:.4f}))
xf.AddScaleOp().Set(Gf.Vec3f({sx:.4f}, {sy:.4f}, {sz:.4f}))
mat = make_mat("/WS_FloorSlab/Mat", 0.1, 0.85, 0.2, 0.25)
UsdShade.MaterialBindingAPI(cube).Bind(mat)

# ── 2. Four edge bars (solid green outline) ────────────────────────────────
edges = [
    # (label,  cx,    cy,    scale_x, scale_y)
    ("N", {cx:.4f}, {WS_Y_MAX:.4f},  {sx:.4f},  0.005),
    ("S", {cx:.4f}, {WS_Y_MIN:.4f},  {sx:.4f},  0.005),
    ("E", {WS_X_MAX:.4f}, {cy:.4f},  0.005, {sy:.4f}),
    ("W", {WS_X_MIN:.4f}, {cy:.4f},  0.005, {sy:.4f}),
]
bar_z = {WS_TABLE_Z:.4f} + 0.006
for label, bx, by, bsx, bsy in edges:
    path = f"/WS_Edge_{{label}}"
    remove(path)
    b = UsdGeom.Cube.Define(stage, path)
    b.GetSizeAttr().Set(1.0)
    xfb = UsdGeom.Xformable(b)
    xfb.AddTranslateOp().Set(Gf.Vec3d(bx, by, bar_z))
    xfb.AddScaleOp().Set(Gf.Vec3f(bsx, bsy, 0.006))
    mat_b = make_mat(path + "/Mat", 0.0, 0.9, 0.0, 1.0)
    UsdShade.MaterialBindingAPI(b).Bind(mat_b)

# ── 3. Corner posts ───────────────────────────────────────────────────────────
corners = [
    ("NE", {WS_X_MAX:.4f}, {WS_Y_MAX:.4f}),
    ("NW", {WS_X_MIN:.4f}, {WS_Y_MAX:.4f}),
    ("SE", {WS_X_MAX:.4f}, {WS_Y_MIN:.4f}),
    ("SW", {WS_X_MIN:.4f}, {WS_Y_MIN:.4f}),
]
post_h = 0.06
for label, px, py in corners:
    path = f"/WS_Post_{{label}}"
    remove(path)
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(0.012)
    cyl.GetHeightAttr().Set(post_h)
    xfc = UsdGeom.Xformable(cyl)
    xfc.AddTranslateOp().Set(Gf.Vec3d(px, py, {WS_TABLE_Z:.4f} + post_h / 2))
    mat_c = make_mat(path + "/Mat", 1.0, 0.85, 0.0, 1.0)   # yellow
    UsdShade.MaterialBindingAPI(cyl).Bind(mat_c)

# ── 4. Stack zone marker (red post) ──────────────────────────────────────────
remove("/WS_StackZone")
sc = UsdGeom.Cylinder.Define(stage, "/WS_StackZone")
sc.GetRadiusAttr().Set(0.025)
sc.GetHeightAttr().Set(0.008)
xfs = UsdGeom.Xformable(sc)
xfs.AddTranslateOp().Set(Gf.Vec3d({STACK_X:.4f}, {STACK_Y:.4f}, {WS_TABLE_Z:.4f} + 0.004))
mat_s = make_mat("/WS_StackZone/Mat", 0.9, 0.1, 0.1, 0.8)   # red
UsdShade.MaterialBindingAPI(sc).Bind(mat_s)

print("Workspace markers placed.")
"""


def _clear_script() -> str:
    return """
import omni.usd
stage = omni.usd.get_context().get_stage()
paths = [
    "/WS_FloorSlab",
    "/WS_Edge_N", "/WS_Edge_S", "/WS_Edge_E", "/WS_Edge_W",
    "/WS_Post_NE", "/WS_Post_NW", "/WS_Post_SE", "/WS_Post_SW",
    "/WS_StackZone",
]
for path in paths:
    p = stage.GetPrimAtPath(path)
    if p and p.IsValid():
        stage.RemovePrim(path)
print("Workspace markers cleared.")
"""


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    parser = argparse.ArgumentParser(description="Visualise workspace rectangle in Isaac Sim.")
    parser.add_argument("--clear", action="store_true", help="Remove markers instead of drawing them")
    args = parser.parse_args()

    print(f"Connecting to Isaac Sim MCP at {ISAAC_HOST}:{ISAAC_PORT}…")
    client = IsaacClient(ISAAC_HOST, ISAAC_PORT)
    try:
        client.connect(retries=5, delay=1.0)
    except ConnectionError as exc:
        print(f"ERROR: {exc}")
        return 1

    if args.clear:
        client.execute_script(_clear_script())
        print("Markers cleared.")
    else:
        client.execute_script(_draw_script())
        print()
        print("  Workspace rectangle drawn in Isaac Sim viewport:")
        print(f"    X : [{WS_X_MIN:.3f}, {WS_X_MAX:.3f}]  width  = {WS_X_MAX - WS_X_MIN:.3f} m")
        print(f"    Y : [{WS_Y_MIN:.3f}, {WS_Y_MAX:.3f}]  depth  = {WS_Y_MAX - WS_Y_MIN:.3f} m")
        print(f"    Z : {WS_TABLE_Z:.3f} (table surface)")
        print(f"    Block min separation : {BLOCK_MIN_SEP:.3f} m")
        print()
        print("  Markers:")
        print("    Green slab      = randomisation area")
        print("    Green edge bars = rectangle boundary")
        print("    Yellow posts    = corners")
        print("    Red disc        = stack zone target")
        print()
        print("  To adjust: edit WS_X_MIN/MAX, WS_Y_MIN/MAX in config.py then re-run.")
        print("  To remove: python3 visualise_workspace.py --clear")

    client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
