#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_PYTHON_BIN="${ROS_PYTHON_BIN:-python3.10}"
MODEL_PYTHON_BIN="${MODEL_PYTHON_BIN:-$REPO_ROOT/.venv/bin/python}"
HARDWARE_TYPE="${HARDWARE_TYPE:-isaac}"
CHECKPOINT_ROOT="${CHECKPOINT_ROOT:-$SCRIPT_DIR/smolvla_training_output}"
SOCKET_PATH="${SOCKET_PATH:-/tmp/smolvla_policy.sock}"

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
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  source "$REPO_ROOT/install/setup.bash"
fi
set -u

# The SmolVLA base processor/config are now cached locally. Force offline mode
# so transformers/huggingface_hub does not attempt network HEAD requests during
# runtime, which would otherwise fail in restricted environments.
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export PYTHONPATH="$REPO_ROOT/lerobot/src:$REPO_ROOT/nr_dual_arm_moveit_config:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rm -f "$SOCKET_PATH"

"$MODEL_PYTHON_BIN" "$SCRIPT_DIR/smolvla_policy_server.py" \
  --checkpoint-root "$CHECKPOINT_ROOT" \
  --socket-path "$SOCKET_PATH" \
  "$@" &
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
  "$@"
