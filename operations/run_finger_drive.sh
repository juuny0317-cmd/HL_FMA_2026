#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: ${HLKU_SCRIPT_DIR}/run_finger_drive.sh [--max-pwm 1..100] [--initial-pwm 0..MAX] [--step-pwm 1..MAX]"
  echo "Starts the full-range NUCLEO bridge and incremental keyboard teleop in one terminal."
  exit 0
fi

max_pwm=100
initial_pwm=10
step_pwm=10
while (($#)); do
  case "$1" in
    --max-pwm)
      max_pwm="${2:-}"
      shift 2
      ;;
    --initial-pwm)
      initial_pwm="${2:-}"
      shift 2
      ;;
    --step-pwm)
      step_pwm="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done
if [[ ! "${max_pwm}" =~ ^[0-9]+$ ]] || ((max_pwm < 1 || max_pwm > 100)); then
  echo "--max-pwm must be an integer from 1 to 100" >&2
  exit 2
fi
if [[ ! "${initial_pwm}" =~ ^[0-9]+$ ]] || ((initial_pwm < 0 || initial_pwm > max_pwm)); then
  echo "--initial-pwm must be an integer from 0 to --max-pwm" >&2
  exit 2
fi
if [[ ! "${step_pwm}" =~ ^[0-9]+$ ]] || ((step_pwm < 1 || step_pwm > max_pwm)); then
  echo "--step-pwm must be an integer from 1 to --max-pwm" >&2
  exit 2
fi
max_duty="$(awk -v pwm="${max_pwm}" 'BEGIN { printf "%.2f", pwm / 100.0 }')"
initial_duty="$(awk -v pwm="${initial_pwm}" 'BEGIN { printf "%.2f", pwm / 100.0 }')"
step_duty="$(awk -v pwm="${step_pwm}" 'BEGIN { printf "%.2f", pwm / 100.0 }')"

source "${HLKU_SCRIPT_DIR}/setup_env.bash"

finger_log_dir="${HLKU_ROOT}/logs/finger_drive"
mkdir -p "${finger_log_dir}"
finger_log_file="${finger_log_dir}/$(date +%Y%m%d_%H%M%S)_finger_drive.log"

launch_pid=""
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ "${launch_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    fi
    wait "${launch_pid}" 2>/dev/null || true
  fi
  exit "${result}"
}
trap cleanup EXIT INT TERM

echo "W/S: each key changes drive PWM by ${step_pwm} and keeps it; A/D: each key changes steering by 0.096 rad (5.5 deg) and keeps it"
echo "Drive starts at 0 PWM: W=+${step_pwm}, repeated W increases; S decreases through 0 into reverse; SPACE=brake; C=center; X=fault; Q=quit"
echo "PWM range: +/-${max_pwm}/100; first/step PWM: ${initial_pwm}/100, ${step_pwm}/100"
echo "ROS launch log: ${finger_log_file}"
setsid ros2 launch hl_ku_core finger_drive.launch.py \
  drive_enabled:=true \
  maximum_drive_duty:="${max_duty}" \
  maximum_drive_command:="${max_pwm}" \
  </dev/null >"${finger_log_file}" 2>&1 &
launch_pid=$!

ros2 run hl_ku_core keyboard_teleop --ros-args \
  --params-file "${HLKU_CONFIG}/system.yaml" \
  --params-file "${HLKU_CONFIG}/route_test.yaml" \
  --params-file "${HLKU_CONFIG}/finger_drive.yaml" \
  -p maximum_drive_duty:="${max_duty}" \
  -p initial_drive_duty:="${initial_duty}" \
  -p drive_duty_step:="${step_duty}"
