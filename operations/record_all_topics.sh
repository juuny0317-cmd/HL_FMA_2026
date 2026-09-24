#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${HLKU_SCRIPT_DIR}/setup_env.bash"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ${HLKU_SCRIPT_DIR}/record_all_topics.sh [BAG_NAME]"
  exit 0
fi

bag_name="${1:-fma_full_$(date +%Y%m%d_%H%M%S)}"
if [[ ! "${bag_name}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "Bag name may contain only letters, digits, dot, underscore, and hyphen." >&2
  exit 2
fi
bag_dir="${HLKU_DATA}/bags"
mkdir -p "${bag_dir}"

# -a keeps rosbag discovery active, so /scan and the LiDAR perception/planning
# topics are recorded even when their publishers start after B is pressed.
echo "Recording all ROS topics as MCAP (including /scan and LiDAR outputs) to ${bag_dir}/${bag_name}"
exec ros2 bag record -a \
  --storage mcap \
  -o "${bag_dir}/${bag_name}" \
  --max-bag-size 4294967296
