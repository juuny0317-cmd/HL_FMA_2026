#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 "${HLKU_SCRIPT_DIR}/build_course_07_csv_parking.py" --check
python3 "${HLKU_SCRIPT_DIR}/build_course_07_mission_csv.py" --check
python3 "${HLKU_SCRIPT_DIR}/generate_mando_competition_routes.py" --full-only
exec "${HLKU_SCRIPT_DIR}/run_mando_tui.sh" \
  --config "${HLKU_SCRIPT_DIR}/mando_competition_scenarios.yaml" "$@"
