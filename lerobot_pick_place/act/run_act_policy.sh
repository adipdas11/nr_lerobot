#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# act/ lives inside lerobot_pick_place/ which lives inside agentic_disassembly/
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_PYTHON_BIN="${ROS_PYTHON_BIN:-python3.10}"
MODEL_PYTHON_BIN="${MODEL_PYTHON_BIN:-$WORKSPACE_ROOT/.venv/bin/python}"
HARDWARE_TYPE="${HARDWARE_TYPE:-real}"
CHECKPOINT_ROOT="${CHECKPOINT_ROOT:-$SCRIPT_DIR/training_output}"
SOCKET_PATH="${SOCKET_PATH:-/tmp/act_policy.sock}"
CHECKPOINT_STEP=""
MODEL_DIR=""
FORWARD_ARGS=()

usage() {
  cat <<EOF
Usage:
  ./run_act_policy.sh [--real | --isaac | --hardware-type <mode>] [--checkpoint STEP|PATH] [args...]

Hardware selection:
  --real               Run against real hardware topics and controllers (default)
  --isaac              Run against Isaac Sim topics and controllers
  --hardware-type MODE Explicit: isaac, twin, real, or fake

Checkpoint selection (default: latest under training_output/):
  --checkpoint STEP    Run step e.g. --checkpoint 050000
                       Searches training_output/checkpoints/STEP/pretrained_model
                       and training_output/STEP/pretrained_model
  --checkpoint PATH    Direct path to a pretrained_model directory

Examples:
  ./run_act_policy.sh --real
  ./run_act_policy.sh --real --checkpoint 050000
  ./run_act_policy.sh --real --dry-run --max-chunks 1
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real)
      HARDWARE_TYPE="real"; shift ;;
    --isaac)
      HARDWARE_TYPE="isaac"; shift ;;
    --hardware-type)
      [[ $# -lt 2 ]] && { echo "--hardware-type requires a value" >&2; usage >&2; exit 1; }
      HARDWARE_TYPE="$2"; shift 2 ;;
    --checkpoint)
      [[ $# -lt 2 ]] && { echo "--checkpoint requires a value" >&2; usage >&2; exit 1; }
      CHECKPOINT_STEP="$2"; shift 2 ;;
    --help|-h)
      usage; exit 0 ;;
    *)
      FORWARD_ARGS+=("$1"); shift ;;
  esac
done

if ! command -v "$ROS_PYTHON_BIN" >/dev/null 2>&1; then
  echo "ROS Python not found: $ROS_PYTHON_BIN" >&2; exit 1
fi
if [[ ! -x "$MODEL_PYTHON_BIN" ]]; then
  echo "Model Python not found: $MODEL_PYTHON_BIN" >&2
  echo "Run: cd $WORKSPACE_ROOT && uv venv --python 3.12 .venv && uv pip install -e ./lerobot torch transformers safetensors opencv-python numpy" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
ROS_WS_ROOT="$(cd "$WORKSPACE_ROOT/../.." && pwd)"
if [[ -f "$ROS_WS_ROOT/install/setup.bash" ]]; then
  source "$ROS_WS_ROOT/install/setup.bash"
fi
set -u

# SCRIPT_DIR on PYTHONPATH so act_ipc.py is importable by both processes.
export PYTHONPATH="$SCRIPT_DIR:$WORKSPACE_ROOT/lerobot/src:$WORKSPACE_ROOT/nr_dual_arm_moveit_config:$WORKSPACE_ROOT${PYTHONPATH:+:$PYTHONPATH}"

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rm -f "$SOCKET_PATH"

# Resolve --checkpoint to a model dir.
if [[ -n "$CHECKPOINT_STEP" ]]; then
  if [[ -d "$CHECKPOINT_STEP" ]]; then
    MODEL_DIR="$CHECKPOINT_STEP"
  else
    CANDIDATE_A="$CHECKPOINT_ROOT/checkpoints/$CHECKPOINT_STEP/pretrained_model"
    CANDIDATE_B="$CHECKPOINT_ROOT/$CHECKPOINT_STEP/pretrained_model"
    if [[ -f "$CANDIDATE_A/config.json" ]]; then
      MODEL_DIR="$CANDIDATE_A"
    elif [[ -f "$CANDIDATE_B/config.json" ]]; then
      MODEL_DIR="$CANDIDATE_B"
    else
      echo "Checkpoint '$CHECKPOINT_STEP' not found." >&2
      echo "  Tried: $CANDIDATE_A" >&2
      echo "  Tried: $CANDIDATE_B" >&2
      exit 1
    fi
  fi
  echo "Starting ACT runtime — hardware: $HARDWARE_TYPE  checkpoint: $MODEL_DIR"
else
  echo "Starting ACT runtime — hardware: $HARDWARE_TYPE  (latest checkpoint)"
fi

SERVER_EXTRA_ARGS=()
[[ -n "$MODEL_DIR" ]] && SERVER_EXTRA_ARGS+=(--model-dir "$MODEL_DIR")

"$MODEL_PYTHON_BIN" "$SCRIPT_DIR/act_policy_server.py" \
  --checkpoint-root "$CHECKPOINT_ROOT" \
  --socket-path "$SOCKET_PATH" \
  "${SERVER_EXTRA_ARGS[@]}" &
SERVER_PID=$!

echo "Waiting for ACT server to become ready..."
for _ in $(seq 1 60); do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    wait "$SERVER_PID"
    echo "ACT policy server exited during startup" >&2
    exit 1
  fi
  if [[ -S "$SOCKET_PATH" ]]; then
    echo "ACT server ready."
    break
  fi
  sleep 1
done

if [[ ! -S "$SOCKET_PATH" ]]; then
  echo "ACT policy server did not become ready at $SOCKET_PATH" >&2
  exit 1
fi

"$ROS_PYTHON_BIN" "$SCRIPT_DIR/act_ros_client.py" \
  --hardware-type "$HARDWARE_TYPE" \
  --socket-path "$SOCKET_PATH" \
  "${FORWARD_ARGS[@]}"
