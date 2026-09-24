#!/usr/bin/env bash
# Stop every HL_KU field-test process. Safe to run repeatedly.
set +e

HLKU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash 2>/dev/null
source "${HLKU_ROOT}/ros2_ws/install/setup.bash" 2>/dev/null

# Request a brake/fault command before tearing down the ROS graph.
timeout 2 ros2 service call /mission/fault std_srvs/srv/Trigger '{}' \
  >/dev/null 2>&1
sleep 0.3

PATTERNS=(
  '[m]ando_tui'
  '[r]un_mando_tui.sh'
  'ros2 launch hl_ku_'
  '/install/hl_ku_core/lib/hl_ku_core/'
  '/install/rplidar_ros/lib/rplidar_ros/'
  '/opt/ros/humble/lib/rviz2/rviz2'
  '/opt/ros/humble/lib/foxglove_bridge/foxglove_bridge'
  '/opt/ros/humble/lib/usb_cam/usb_cam_node_exe'
  '[n]ucleo_serial_mux'
  'ros2 bag record'
  '[f]oxglove-studio'
)

for pattern in "${PATTERNS[@]}"; do
  pkill -INT -f "${pattern}" 2>/dev/null
done
sleep 2
for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null
done

rm -f /tmp/nucleo-control /tmp/nucleo-lidar
echo "HL_KU TUI, ROS, LiDAR, RViz, Foxglove, NUCLEO mux stopped."
