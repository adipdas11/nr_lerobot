#!/usr/bin/env bash
set -euo pipefail

CLEAN_BUILD=0

for arg in "$@"; do
  case "${arg}" in
    --clean)
      CLEAN_BUILD=1
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      echo "Usage: $0 [--clean]" >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if docker compose version >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker compose)
  DOCKER_COMPOSE_ENV=()
elif command -v docker-compose >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker-compose)
  DOCKER_COMPOSE_ENV=(env PYTHONNOUSERSITE=1 PYTHONPATH=)
else
  echo "Docker Compose is required but was not found." >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is required for GUI forwarding but was not found." >&2
  exit 1
fi

xhost +local:root >/dev/null

export HOST_UID
export HOST_GID
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

# A clean workspace build also refreshes the image. This is where the pinned
# Humble micro-ROS Agent is built, so it cannot be deleted with /ws/install.
if [[ "${CLEAN_BUILD}" == "1" ]]; then
  echo "[run_teleop] Building the Docker image, including micro-ROS for Humble..."
  "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" build teleop
fi

if ! docker image inspect agentic-disassembly-teleop:humble >/dev/null 2>&1; then
  echo "[run_teleop] Docker image is missing; building it now..."
  "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" build teleop
fi

# Bypass the image entrypoint while compiling. The entrypoint normally sources
# /ws/install, which must not be used as an underlay while rebuilding itself.
"${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" run --rm -T --entrypoint /bin/bash -e CLEAN_BUILD="${CLEAN_BUILD}" teleop -s <<'CONTAINER_BUILD_SCRIPT'
set -eo pipefail

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

WORKSPACE_ROOT="${HOME}"

if [[ -z "${WORKSPACE_ROOT}" || "${WORKSPACE_ROOT}" == "/" ]]; then
  echo "[run_teleop] Invalid workspace root: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

if [[ ! -d "${WORKSPACE_ROOT}/src" ]]; then
  echo "[run_teleop] Source directory not found: ${WORKSPACE_ROOT}/src" >&2
  exit 1
fi

cd "${WORKSPACE_ROOT}"
source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup /opt/micro_ros_humble/setup.bash

if [[ "${CLEAN_BUILD}" == "1" ]]; then
  echo "[run_teleop] Removing workspace build, install and log directories..."
  rm -rf "${WORKSPACE_ROOT}/build" "${WORKSPACE_ROOT}/install" "${WORKSPACE_ROOT}/log"
fi

# LeRobot is a regular Python project, not a ROS package.
LEROBOT_DIRECTORY="${WORKSPACE_ROOT}/src/nr_lerobot/lerobot"
if [[ -d "${LEROBOT_DIRECTORY}" ]]; then
  touch "${LEROBOT_DIRECTORY}/COLCON_IGNORE"
fi

BUILD_PACKAGES=(
  ros_tcp_endpoint
  exotica_core
  exotica_collision_scene_fcl_latest
  exotica_core_task_maps
  exotica_ik_solver
  exotica_ompl_solver
  exotica_python
  hand_teleoperation_interfaces
  hand_teleoperation
  nr_dual_arm_description
  nr_dual_arm_moveit_config
  arm_teleop
)

echo "[run_teleop] Building the mounted ROS workspace..."
colcon build --symlink-install --executor sequential --event-handlers console_direct+ --packages-select "${BUILD_PACKAGES[@]}"

source_ros_setup "${WORKSPACE_ROOT}/install/setup.bash"

echo "[run_teleop] Validating installed packages and executables..."
VALIDATE_PACKAGES=(
  micro_ros_msgs
  micro_ros_agent
  hand_teleoperation_interfaces
  hand_teleoperation
  arm_teleop
)

for package in "${VALIDATE_PACKAGES[@]}"; do
  prefix="$(ros2 pkg prefix "${package}")"
  echo "[run_teleop] Found ${package}: ${prefix}"
done

HAND_PREFIX="$(ros2 pkg prefix hand_teleoperation)"
test -f "${HAND_PREFIX}/share/hand_teleoperation/launch/node.launch.py"
test -f "${HAND_PREFIX}/share/hand_teleoperation/launch/node_launch.py"
ros2 pkg executables hand_teleoperation | grep -Fq "hand_teleoperation filter_node"
ros2 pkg executables micro_ros_agent | grep -Fq "micro_ros_agent micro_ros_agent"

echo "[run_teleop] Build and validation completed successfully."
CONTAINER_BUILD_SCRIPT

echo "[run_teleop] Opening a fresh teleoperation container..."
exec "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" run --rm teleop bash
