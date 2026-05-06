#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# smolvla/ lives inside lerobot_pick_place/ which lives inside agentic_disassembly/
# WORKSPACE_ROOT is agentic_disassembly/ — two levels up from SCRIPT_DIR
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_PYTHON_BIN="${ROS_PYTHON_BIN:-python3.10}"
MODEL_PYTHON_BIN="${MODEL_PYTHON_BIN:-$WORKSPACE_ROOT/.venv/bin/python}"
HARDWARE_TYPE="${HARDWARE_TYPE:-isaac}"
# Support both 'smolvla_training_output' and the more generic 'training_output'
if [[ -z "${CHECKPOINT_ROOT:-}" ]]; then
  if [[ -d "$SCRIPT_DIR/smolvla_training_output" ]]; then
    CHECKPOINT_ROOT="$SCRIPT_DIR/smolvla_training_output"
  else
    CHECKPOINT_ROOT="$SCRIPT_DIR/training_output"
  fi
fi

SOCKET_PATH="${SOCKET_PATH:-/tmp/smolvla_policy.sock}"
CHECKPOINT_STEP=""   # specific step number, e.g. 010000
MODEL_DIR=""         # or a fully-qualified path to a pretrained_model dir
VLM_MODEL_DIR="${VLM_MODEL_DIR:-}"
FORWARD_ARGS=()

usage() {
  cat <<EOF
Usage:
  ./run_smolvla_policy.sh [--isaac | --real | --hardware-type <mode>] [--checkpoint STEP|PATH] [--vlm-model-dir PATH] [smolvla args...]

Hardware selection:
  --isaac              Run against Isaac Sim topics and controllers
  --real               Run against real hardware topics and controllers
  --hardware-type MODE Explicit hardware type: isaac, twin, real, or fake

Checkpoint selection (default: latest checkpoint under CHECKPOINT_ROOT):
  --checkpoint STEP    Run a specific training step, e.g. --checkpoint 010000
                       Looks under CHECKPOINT_ROOT/checkpoints/STEP/pretrained_model
                       and CHECKPOINT_ROOT/STEP/pretrained_model
  --checkpoint PATH    Run a specific pretrained_model directory by full path

Base VLM assets for offline inference:
  --vlm-model-dir PATH Local directory containing SmolVLM processor/tokenizer/config files.
                       Required for offline inference if the Hugging Face cache is empty and the
                       checkpoint does not already bundle these assets.

Examples:
  ./run_smolvla_policy.sh --real
  ./run_smolvla_policy.sh --real --checkpoint 010000
  ./run_smolvla_policy.sh --real --checkpoint /abs/path/to/pretrained_model
  ./run_smolvla_policy.sh --real --checkpoint 050000 --vlm-model-dir /abs/path/to/SmolVLM2-500M-Video-Instruct
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --isaac)
      HARDWARE_TYPE="isaac"
      shift
      ;;
    --real)
      HARDWARE_TYPE="real"
      shift
      ;;
    --hardware-type)
      if [[ $# -lt 2 ]]; then
        echo "--hardware-type requires a value" >&2
        usage >&2
        exit 1
      fi
      HARDWARE_TYPE="$2"
      shift 2
      ;;
    --checkpoint)
      if [[ $# -lt 2 ]]; then
        echo "--checkpoint requires a value (step number or path)" >&2
        usage >&2
        exit 1
      fi
      CHECKPOINT_STEP="$2"
      shift 2
      ;;
    --vlm-model-dir)
      if [[ $# -lt 2 ]]; then
        echo "--vlm-model-dir requires a path" >&2
        usage >&2
        exit 1
      fi
      VLM_MODEL_DIR="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      FORWARD_ARGS+=("$1")
      shift
      ;;
  esac
done

if ! command -v "$ROS_PYTHON_BIN" >/dev/null 2>&1; then
  echo "ROS Python executable not found: $ROS_PYTHON_BIN" >&2
  exit 1
fi
if [[ ! -x "$MODEL_PYTHON_BIN" ]]; then
  echo "Model Python executable not found: $MODEL_PYTHON_BIN" >&2
  exit 1
fi

# ROS setup scripts are not consistently nounset-safe.
set +u
source /opt/ros/humble/setup.bash
# ROS workspace install is two levels above smolvla/ (at disassembly_ws level)
ROS_WS_ROOT="$(cd "$WORKSPACE_ROOT/../.." && pwd)"
if [[ -f "$ROS_WS_ROOT/install/setup.bash" ]]; then
  source "$ROS_WS_ROOT/install/setup.bash"
elif [[ -f "$WORKSPACE_ROOT/../../install/setup.bash" ]]; then
  source "$WORKSPACE_ROOT/../../install/setup.bash"
fi
set -u

# SCRIPT_DIR added so smolvla_ipc.py is importable by both server and client
export PYTHONPATH="$SCRIPT_DIR:$WORKSPACE_ROOT/lerobot/src:$WORKSPACE_ROOT/nr_dual_arm_moveit_config:$WORKSPACE_ROOT${PYTHONPATH:+:$PYTHONPATH}"

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rm -f "$SOCKET_PATH"

# Resolve --checkpoint argument to a --model-dir path for the server.
if [[ -n "$CHECKPOINT_STEP" ]]; then
  if [[ -d "$CHECKPOINT_STEP" ]]; then
    # Treat as a direct path to a pretrained_model directory.
    MODEL_DIR="$CHECKPOINT_STEP"
  else
    # Treat as a step number; try both checkpoint layouts.
    CANDIDATE_A="$CHECKPOINT_ROOT/checkpoints/$CHECKPOINT_STEP/pretrained_model"
    CANDIDATE_B="$CHECKPOINT_ROOT/$CHECKPOINT_STEP/pretrained_model"
    if [[ -f "$CANDIDATE_A/config.json" ]]; then
      MODEL_DIR="$CANDIDATE_A"
    elif [[ -f "$CANDIDATE_B/config.json" ]]; then
      MODEL_DIR="$CANDIDATE_B"
    else
      echo "Checkpoint step '$CHECKPOINT_STEP' not found." >&2
      echo "Tried:" >&2
      echo "  $CANDIDATE_A" >&2
      echo "  $CANDIDATE_B" >&2
      exit 1
    fi
  fi
  echo "Starting SmolVLA runtime with hardware type: $HARDWARE_TYPE, checkpoint: $MODEL_DIR"
else
  echo "Starting SmolVLA runtime with hardware type: $HARDWARE_TYPE (latest checkpoint)"
fi

SERVER_EXTRA_ARGS=()
if [[ -n "$MODEL_DIR" ]]; then
  SERVER_EXTRA_ARGS+=(--model-dir "$MODEL_DIR")
fi
if [[ -n "$VLM_MODEL_DIR" ]]; then
  SERVER_EXTRA_ARGS+=(--vlm-model-dir "$VLM_MODEL_DIR")
fi

"$MODEL_PYTHON_BIN" "$SCRIPT_DIR/smolvla_policy_server.py" \
  --checkpoint-root "$CHECKPOINT_ROOT" \
  --socket-path "$SOCKET_PATH" \
  "${SERVER_EXTRA_ARGS[@]}" \
  "${FORWARD_ARGS[@]}" &
SERVER_PID=$!

for _ in $(seq 1 60); do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    wait "$SERVER_PID"
    echo "SmolVLA policy server exited during startup" >&2
    exit 1
  fi
  if [[ -S "$SOCKET_PATH" ]]; then
    break
  fi
  sleep 1
done

if [[ ! -S "$SOCKET_PATH" ]]; then
  echo "SmolVLA policy server did not become ready at $SOCKET_PATH" >&2
  exit 1
fi

"$ROS_PYTHON_BIN" "$SCRIPT_DIR/smolvla_ros_client.py" \
  --hardware-type "$HARDWARE_TYPE" \
  --socket-path "$SOCKET_PATH" \
  "${FORWARD_ARGS[@]}"
