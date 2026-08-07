#!/usr/bin/env bash
set -euo pipefail

CLEAN_BUILD=0
for arg in "$@"; do
  if [[ "$arg" == "--clean" ]]; then
    CLEAN_BUILD=1
  fi
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if docker compose version >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker compose)
  DOCKER_COMPOSE_ENV=()
elif command -v docker-compose >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker-compose)
  # Legacy docker-compose is a Python app and can break if user-site packages
  # override the distro-pinned docker SDK dependencies.
  DOCKER_COMPOSE_ENV=(env PYTHONNOUSERSITE=1 PYTHONPATH=)
else
  echo "Docker Compose is required but neither 'docker compose' nor 'docker-compose' was found in PATH." >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is required for GUI forwarding but was not found in PATH." >&2
  exit 1
fi

xhost +local:root >/dev/null

# Pass host UID/GID into compose so files created in the container are owned
# by the current user rather than root.
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

exec "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" run --rm teleop bash -lc "
  set -e
  cd /ws

  CLEAN_BUILD=${CLEAN_BUILD}

  if [ \"\${CLEAN_BUILD}\" = \"1\" ]; then
    echo '[run_teleop] --clean: removing build/, install/, log/ for a fresh colcon build.'
    rm -rf /ws/build /ws/install /ws/log
  fi

  # Sentinels: older host installs may contain arm_teleop but predate the Vuer
  # bridge. The host install is mounted over the copy baked into the image.
  BUILT_MARKER=/ws/install/arm_teleop/share/arm_teleop/package.sh
  VUER_BRIDGE_MARKER=/ws/install/arm_teleop/lib/arm_teleop/vuer_quest_bridge

  if [ -f \"\${BUILT_MARKER}\" ] && [ -x \"\${VUER_BRIDGE_MARKER}\" ]; then
    echo '[run_teleop] Workspace already built — skipping colcon build.'
    source /ws/install/setup.bash
  elif [ -f \"\${BUILT_MARKER}\" ]; then
    echo '[run_teleop] Existing workspace predates the Vuer bridge — rebuilding arm_teleop.'
    source /ws/install/setup.bash
    colcon build --packages-select arm_teleop --executor sequential
    source /ws/install/setup.bash
  else
    echo '[run_teleop] Building workspace...'
    colcon build --packages-select ros_tcp_endpoint --executor sequential
    source /ws/install/setup.bash
    colcon build --packages-select \
      exotica_core \
      exotica_collision_scene_fcl_latest \
      exotica_core_task_maps \
      exotica_ik_solver \
      exotica_ompl_solver \
      exotica_python \
      --executor sequential
    source /ws/install/setup.bash
    colcon build --packages-select nr_dual_arm_description nr_dual_arm_moveit_config arm_teleop --executor sequential
    source /ws/install/setup.bash
  fi

  exec bash
"
