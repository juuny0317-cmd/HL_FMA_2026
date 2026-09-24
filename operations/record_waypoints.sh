#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ${HLKU_SCRIPT_DIR}/record_waypoints.sh COURSE_NAME"
  echo "Example: ${HLKU_SCRIPT_DIR}/record_waypoints.sh course_07"
  exit 0
fi
if [[ $# -ne 1 ]]; then
  echo "Usage: ${HLKU_SCRIPT_DIR}/record_waypoints.sh COURSE_NAME" >&2
  exit 2
fi
if [[ ! "${1}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "COURSE_NAME may contain only letters, digits, underscore, and hyphen." >&2
  exit 2
fi

source "${HLKU_SCRIPT_DIR}/setup_env.bash"
exec ros2 run hl_ku_core rtk_waypoint_recorder --ros-args \
  --params-file "${HLKU_CONFIG}/rtk_field_test.yaml" \
  -p "output_prefix:=${HLKU_DATA}/${1}"
