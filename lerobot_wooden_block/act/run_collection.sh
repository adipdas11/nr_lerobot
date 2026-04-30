#!/usr/bin/env bash
# Automated data collection for wooden-block stacking task.
#
# Usage:
#   ./run_collection.sh [--episodes N] [--seed S] [--output-dir DIR]
#
# Prerequisites (run in separate terminals before this script):
#   Terminal 1 — Isaac Sim open with nr_dual_arm_scene_wooden_blocks.usd + MCP extension enabled
#   Terminal 2 — ros2 launch nr_dual_arm_moveit_config demo.launch.py hardware_type:=isaac

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── ROS / workspace setup ─────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source /home/adip/workspace/lerobot_ws/install/setup.bash

# ── Defaults ──────────────────────────────────────────────────────────────────
EPISODES=50
SEED=42
OUTPUT_DIR="$SCRIPT_DIR/rosbags"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --episodes) EPISODES="$2"; shift 2 ;;
    --seed)     SEED="$2";     shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

echo "════════════════════════════════════════════════════════════"
echo "  Wooden Block Data Collection"
echo "  Episodes : $EPISODES"
echo "  Output   : $OUTPUT_DIR"
echo "  Seed     : $SEED"
echo "════════════════════════════════════════════════════════════"

# ── Pre-flight checks ─────────────────────────────────────────────────────────
echo "[check] Waiting for /isaac_joint_states topic…"
timeout 30 ros2 topic echo /isaac_joint_states --once > /dev/null 2>&1 || {
  echo "ERROR: /isaac_joint_states not found. Is Isaac Sim running with the OmniGraph ROS bridge?"
  exit 1
}
echo "[check] /isaac_joint_states OK"

echo "[check] Waiting for /camera1/realsense_camera/color/image_raw/compressed…"
timeout 15 ros2 topic echo /camera1/realsense_camera/color/image_raw/compressed --once > /dev/null 2>&1 || {
  echo "ERROR: camera1 topic not found."
  exit 1
}
echo "[check] Camera topics OK"

# EXOTica initialises via internal topics/process — no single service to ping.
# We check that the nr_dual_arm_moveit_config launch is up by verifying
# /joint_states is being published (move_group republishes it) and that the
# EXOTica IK node process exists.  If neither is available, move_to_pose_exotica
# still falls back to MoveIt's /compute_ik, so we check that as a safety net.
echo "[check] Waiting for MoveIt move_group (/joint_states or /compute_ik)…"
MOVEIT_OK=false
if timeout 10 ros2 topic echo /joint_states --once > /dev/null 2>&1; then
  MOVEIT_OK=true
elif timeout 20 ros2 service list 2>/dev/null | grep -q compute_ik; then
  MOVEIT_OK=true
fi
if [ "$MOVEIT_OK" = false ]; then
  echo "WARNING: MoveIt move_group not detected. EXOTica will use its local planner."
  echo "  For best results launch: ros2 launch nr_dual_arm_moveit_config demo.launch.py hardware_type:=isaac"
fi
echo "[check] Motion planner check done (EXOTica handles fallback internally)"

echo "[check] Pinging Isaac Sim MCP socket (port 8766)…"
python3 -c "
from isaac_data_collection.isaac_client import IsaacClient
c = IsaacClient()
c.connect(retries=3)
print('  Isaac Sim MCP: connected')
c.disconnect()
" || {
  echo "ERROR: Cannot connect to Isaac Sim MCP on port 8766."
  echo "  Enable the Isaac Sim MCP Server extension inside Isaac Sim."
  exit 1
}

# ── Run collection ────────────────────────────────────────────────────────────
echo ""
echo "Starting collection…"
python3 "$SCRIPT_DIR/collect_data.py" \
  --episodes "$EPISODES" \
  --output-dir "$OUTPUT_DIR" \
  --seed "$SEED"

# ── Validate bags ─────────────────────────────────────────────────────────────
echo ""
echo "Validating recorded bags…"
python3 "$SCRIPT_DIR/validate_bags.py" --bag-dir "$OUTPUT_DIR"

echo ""
echo "Done. Convert to LeRobot dataset with:"
echo "  python3 rosbag_to_lerobot_act.py --dir $OUTPUT_DIR"
