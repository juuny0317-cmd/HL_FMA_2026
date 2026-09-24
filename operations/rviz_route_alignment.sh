#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${HLKU_SCRIPT_DIR}/setup_env.bash"
exec rviz2 -d "${HLKU_RVIZ}/gps_route_alignment.rviz"
