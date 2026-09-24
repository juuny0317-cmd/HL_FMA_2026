#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${HLKU_SCRIPT_DIR}/setup_env.bash"

HLKU_ROUTE="${HLKU_WS}/src/hl_ku_core/routes/course_07_vehicle.csv"
HLKU_REFERENCE="${HLKU_WS}/src/hl_ku_core/routes/course_07_parking_overlay/course_07_p1_t1_f1_fair_fsm_preview.csv"

exec ros2 run hl_ku_core mission_tagging \
  "${HLKU_ROUTE}" \
  "${HLKU_REFERENCE}" \
  "${HLKU_ROUTE}" \
  "$@"
