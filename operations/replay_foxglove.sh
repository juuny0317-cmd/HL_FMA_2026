#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${HLKU_SCRIPT_DIR}/setup_env.bash"

default_bag="${HLKU_DATA}/bags/fma_full_20260905_152721"
bag_path="${1:-${default_bag}}"
rate="${2:-1.0}"
recorded_route="${HLKU_RECORDED_ROUTE:-${HLKU_DATA}/course_07_route.csv}"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ${HLKU_SCRIPT_DIR}/replay_foxglove.sh [BAG_PATH] [RATE]"
  echo "Default bag: ${default_bag}"
  echo "Data root override: export HLKU_DATA=/absolute/path/to/HL_KU_RTK"
  echo "Recorded route override: export HLKU_RECORDED_ROUTE=/absolute/path/to/route.csv"
  exit 0
fi
if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "ROS bag metadata not found: ${bag_path}/metadata.yaml" >&2
  exit 2
fi
if [[ ! "${rate}" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "${rate}" == "0" ]] || [[ "${rate}" == "0.0" ]]; then
  echo "RATE must be a positive number: ${rate}" >&2
  exit 2
fi
if ! ros2 pkg prefix hl_ku_foxglove >/dev/null 2>&1; then
  echo "hl_ku_foxglove is not built. Run colcon build in ${HLKU_WS}." >&2
  exit 2
fi

launch_arguments=()
if [[ -f "${recorded_route}" ]]; then
  launch_arguments+=("recorded_route_file:=${recorded_route}")
else
  echo "Recorded route CSV not found; continuing without recorded-route overlay: ${recorded_route}" >&2
fi

launch_pid=""
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ "${launch_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  exit "${result}"
}
trap cleanup EXIT INT TERM

echo "Starting read-only Foxglove replay"
echo "  bag  : ${bag_path}"
echo "  rate : ${rate}x"
echo "  safety: recorded actuator-command topics are excluded"

setsid ros2 launch hl_ku_foxglove course_07_replay.launch.py "${launch_arguments[@]}" &
launch_pid=$!

for _ in {1..50}; do
  ros2 node list 2>/dev/null | grep -qx '/foxglove_bridge' && break
  sleep 0.1
done

ros2 bag play "${bag_path}" \
  --clock 50 \
  --rate "${rate}" \
  --topics \
  /camera/image_raw \
  /camera/camera_info \
  /perception/yolo_overlay \
  /gnss/fix \
  /gnss/status \
  /planning/reference_path \
  /mission/status \
  /vehicle/feedback \
  /safety/status
