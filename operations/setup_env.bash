#!/usr/bin/env bash
# Source this file to use the HL_KU ROS workspace from any directory.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Run this file with: source /path/to/HL_KU/operations/setup_env.bash"
  exit 2
fi

_hlku_ops_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export HLKU_OPS="${_hlku_ops_dir}"
export HLKU_ROOT="$(cd -- "${HLKU_OPS}/.." && pwd)"
export HLKU_WS="${HLKU_ROOT}/ros2_ws"
export HLKU_PACKAGE="${HLKU_WS}/src/hl_ku_core"
export HLKU_NODES="${HLKU_PACKAGE}/hl_ku_core"
export HLKU_LAUNCH="${HLKU_PACKAGE}/launch"
export HLKU_CONFIG="${HLKU_PACKAGE}/config"
export HLKU_ROUTES="${HLKU_PACKAGE}/routes"
export HLKU_RVIZ="${HLKU_PACKAGE}/rviz"
_hlku_default_data="$(cd -- "${HLKU_ROOT}/.." && pwd)/HL_KU_RTK"
if [[ ! -d "${_hlku_default_data}" ]]; then
  _hlku_default_data="${HLKU_ROOT}/data"
fi
export HLKU_DATA="${HLKU_DATA:-${_hlku_default_data}}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble environment not found: /opt/ros/humble/setup.bash" >&2
  unset _hlku_default_data _hlku_ops_dir
  return 2
fi
source /opt/ros/humble/setup.bash

if [[ ! -f "${HLKU_WS}/install/setup.bash" ]]; then
  echo "HL_KU workspace is not built." >&2
  echo "Build it with: cd \"${HLKU_WS}\" && colcon build --symlink-install" >&2
  unset _hlku_default_data _hlku_ops_dir
  return 2
fi
source "${HLKU_WS}/install/setup.bash"
unset _hlku_default_data _hlku_ops_dir

echo "HL_KU environment ready"
echo "  operations : ${HLKU_OPS}"
echo "  workspace  : ${HLKU_WS}"
echo "  routes     : ${HLKU_ROUTES}"
echo "  RTK data   : ${HLKU_DATA}"
