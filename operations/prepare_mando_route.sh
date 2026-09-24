#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${HLKU_SCRIPT_DIR}/setup_env.bash"
exec ros2 run hl_ku_core mando_route_slice "$@"
