#!/usr/bin/env bash
# ROS 2's generated setup scripts reference optional variables without default
# values, so this entrypoint cannot enable Bash nounset (`set -u`).
set -eo pipefail

stage_vuer_tls() {
  local cert_file="${VUER_TLS_CERT_SOURCE:-${VUER_CERT_FILE:-}}"
  local key_file="${VUER_TLS_KEY_SOURCE:-${VUER_KEY_FILE:-}}"
  local runtime_dir="/run/vuer-tls"

  # Vuer is optional, so an absent or incomplete certificate pair must not
  # prevent the container from being used for the other teleoperation modes.
  if [[ -z "${cert_file}" || -z "${key_file}" ]]; then
    return
  fi
  if [[ ! -f "${cert_file}" || ! -f "${key_file}" ]]; then
    return
  fi

  install -d -m 0700 -o "${HOST_UID}" -g "${HOST_GID}" "${runtime_dir}"
  if ! install -m 0600 -o "${HOST_UID}" -g "${HOST_GID}" -- \
    "${cert_file}" "${runtime_dir}/cert.pem"; then
    echo "[teleop-entrypoint] Warning: could not stage the Vuer certificate from ${cert_file}." >&2
    return
  fi
  if ! install -m 0600 -o "${HOST_UID}" -g "${HOST_GID}" -- \
    "${key_file}" "${runtime_dir}/key.pem"; then
    echo "[teleop-entrypoint] Warning: could not stage the Vuer private key from ${key_file}." >&2
    rm -f "${runtime_dir}/cert.pem"
    return
  fi

  export VUER_CERT_FILE="${runtime_dir}/cert.pem"
  export VUER_KEY_FILE="${runtime_dir}/key.pem"
  echo "[teleop-entrypoint] Vuer TLS files staged in ${runtime_dir}."
}

drop_to_host_user() {
  local gid
  local supplementary_gids=""
  local -a current_gids=()
  local -a group_option=()

  read -r -a current_gids <<< "$(id -G)"
  for gid in "${current_gids[@]}"; do
    if [[ "${gid}" == "0" || "${gid}" == "${HOST_GID}" ]]; then
      continue
    fi
    supplementary_gids+="${supplementary_gids:+,}${gid}"
  done

  if [[ -n "${supplementary_gids}" ]]; then
    group_option=(--groups="${supplementary_gids}")
  else
    group_option=(--clear-groups)
  fi

  exec setpriv \
    --reuid="${HOST_UID}" \
    --regid="${HOST_GID}" \
    "${group_option[@]}" \
    /ros_entrypoint_local.sh "$@"
}

if [[ "$(id -u)" == "0" ]]; then
  HOST_UID="${HOST_UID:-0}"
  HOST_GID="${HOST_GID:-0}"
  if [[ ! "${HOST_UID}" =~ ^[0-9]+$ || ! "${HOST_GID}" =~ ^[0-9]+$ ]]; then
    echo "[teleop-entrypoint] HOST_UID and HOST_GID must be numeric." >&2
    exit 1
  fi

  stage_vuer_tls

  # Only the certificate staging step runs as root. Re-enter this script as
  # the host user before sourcing the bind-mounted workspace or running ROS.
  if [[ "${HOST_UID}" != "0" ]]; then
    drop_to_host_user "$@"
  fi
fi

source /opt/ros/humble/setup.bash

if [[ -f /ws/install/setup.bash ]]; then
  source /ws/install/setup.bash
fi

exec "$@"
