"""Configuration constants for wooden-block data collection."""

from __future__ import annotations

# ── Isaac Sim MCP TCP connection ─────────────────────────────────────────────
ISAAC_HOST = "localhost"
ISAAC_PORT = 8766

# ── USD prim paths ────────────────────────────────────────────────────────────
BLOCK_PRIM_PATHS = ["/block_1_base", "/block_2_base", "/block_3_base"]
SCENE_ROOT = "/World/nr_dual_arm_scene"

# ── Joint names ───────────────────────────────────────────────────────────────
SLIDER_JOINT = "uf_slide_joint"
ARM_JOINTS = ["xarm5_joint1", "xarm5_joint2", "xarm5_joint3", "xarm5_joint4", "xarm5_joint5"]
GRIPPER_JOINT = "xarm_gripper_right_drive_joint"
# All joints included in the recorded joint state
STATE_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]

# ── Slider ───────────────────────────────────────────────────────────────────
# Slider moves along world X.  Home = 0.35 m (centre of 0–0.70 m range).
SLIDER_HOME_M = 0.35

# ── Workspace rectangle (world coordinates, metres) ──────────────────────────
# Blocks are randomised inside this rectangle; Z is the nominal table surface.
WS_X_MIN = 0.768   # square 0.315 m side, centre shifted +5 cm forward (X)
WS_X_MAX = 1.083
WS_Y_MIN = 0.668   # square, same centre Y as before
WS_Y_MAX = 0.983
WS_TABLE_Z = 0.856          # estimated table surface (block z-centre − block_h/2)

# Minimum centre-to-centre separation between blocks (metres)
BLOCK_MIN_SEP = 0.12

# ── Block geometry ────────────────────────────────────────────────────────────
# Triangular wooden blocks, flat sides normal to block-local X-axis.
# Height observed from stacked nominal positions: each block is 0.030 m tall.
BLOCK_HEIGHT_M = 0.030
# Z of block centre when resting flat on the table (used for teleport placement)
BLOCK_PLACE_Z = WS_TABLE_Z + BLOCK_HEIGHT_M / 2   # = 0.871 m

# ── Block upright verification ────────────────────────────────────────────────
# After teleporting a block, step physics and check that roll and pitch stayed
# below this threshold.  If not, retry with a new random (x, y, yaw).
BLOCK_UPRIGHT_TOLERANCE_DEG = 5.0
MAX_PLACE_RETRIES = 8

# ── Stack zone (world X, Y where block_1 is placed at the bottom) ────────────
STACK_X = 0.85
STACK_Y = 0.72

# ── Grasp parameters (arm base_link frame) ───────────────────────────────────
PREGRASP_Z_OFFSET = 0.12    # approach height above block centre
GRASP_Z_OFFSET = 0.005      # small extra descent for firm contact
LIFT_Z = 0.15               # lift height above table

# Gripper positions (drive joint, radians)
GRIPPER_OPEN_RAD = 0.0
GRIPPER_CLOSED_RAD = 0.55   # adjust based on block width during validation

# ── MoveIt planning group ─────────────────────────────────────────────────────
ARM_GROUP = "xarm5_arm_no_slide"
GRIPPER_GROUP = "xarm_gripper"

# Arm home joint positions (radians) — upright rest pose
ARM_HOME_JOINTS = {
    "xarm5_joint1": 0.0,
    "xarm5_joint2": -0.5,
    "xarm5_joint3": -1.2,
    "xarm5_joint4": 1.7,
    "xarm5_joint5": 0.0,
}

# ── ROS topics ────────────────────────────────────────────────────────────────
JOINT_STATE_TOPIC = "/isaac_joint_states"
CAMERA1_TOPIC = "/camera1/realsense_camera/color/image_raw/compressed"
CAMERA2_TOPIC = "/camera2/realsense_camera/color/image_raw/compressed"

# ── Data collection ───────────────────────────────────────────────────────────
NUM_EPISODES = 50
ROSBAG_BASE_DIR = "rosbags"

# ── Domain randomisation ──────────────────────────────────────────────────────
# Block diffuse colours sampled from this palette (R,G,B in 0–1)
BLOCK_COLOR_PALETTE = [
    (0.8, 0.2, 0.1),   # red
    (0.1, 0.5, 0.8),   # blue
    (0.2, 0.7, 0.2),   # green
    (0.9, 0.8, 0.1),   # yellow
    (0.6, 0.3, 0.7),   # purple
    (0.9, 0.5, 0.1),   # orange
    (0.8, 0.8, 0.8),   # light grey (near real wood)
    (0.4, 0.25, 0.1),  # dark wood brown
]

# Lighting intensity scale factors relative to nominal
LIGHT_INTENSITY_RANGE = (0.6, 1.5)
# Light colour temperature range (Kelvin) mapped to a warm–cool RGB tint
LIGHT_TEMP_RANGE = (4000, 6500)
