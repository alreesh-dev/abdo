#!/usr/bin/env bash
# Source ROS 2 + workspace overlays using ABDO_WS / ABDO_EXTRA_WS.
#
# Usage:
#   cp my_factory/config/abdo.env.example my_factory/config/abdo.env
#   # edit paths in abdo.env
#   source env/setup_abdo.bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${ABDO_ENV_FILE:-${SRC_ROOT}/my_factory/config/abdo.env}"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

: "${ABDO_WS:=${SRC_ROOT}/..}"
: "${ABDO_EXTRA_WS:=}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -n "${ABDO_EXTRA_WS}" && -f "${ABDO_EXTRA_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ABDO_EXTRA_WS}/install/setup.bash"
fi

if [[ -f "${ABDO_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ABDO_WS}/install/setup.bash"
else
  echo "warning: ABDO_WS install not found at ${ABDO_WS}/install/setup.bash" >&2
  echo "         build with: cd \"${ABDO_WS}\" && colcon build --symlink-install" >&2
fi

echo "ABDO environment loaded:"
echo "  ABDO_WS=${ABDO_WS}"
echo "  ABDO_EXTRA_WS=${ABDO_EXTRA_WS:-<unset>}"
