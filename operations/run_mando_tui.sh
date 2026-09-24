#!/usr/bin/env bash
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<EOF
Usage: ${HLKU_SCRIPT_DIR}/run_mando_tui.sh [mando_tui options]

Options are passed to the ROS 2 mando_tui executable. Common examples:
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh --no-auto-rtk
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh --check-config
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh --print-commands
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh --no-open-foxglove
  ${HLKU_SCRIPT_DIR}/run_mando_tui.sh --no-start-mux
EOF
  exit 0
fi

source "${HLKU_SCRIPT_DIR}/setup_env.bash"

HLKU_OPEN_FOXGLOVE=true
HLKU_NEEDS_HARDWARE=true
HLKU_TUI_ARGS=()
for HLKU_ARG in "$@"; do
  if [[ "${HLKU_ARG}" == "--no-open-foxglove" ]]; then
    HLKU_OPEN_FOXGLOVE=false
  elif [[ "${HLKU_ARG}" == "--no-start-mux" ]]; then
    HLKU_NEEDS_HARDWARE=false
  else
    HLKU_TUI_ARGS+=("${HLKU_ARG}")
  fi
  if [[ "${HLKU_ARG}" == "--check-config" || "${HLKU_ARG}" == "--print-commands" ]]; then
    HLKU_OPEN_FOXGLOVE=false
    HLKU_NEEDS_HARDWARE=false
  fi
done

HLKU_MUX_PID=""
HLKU_NUCLEO_PORT="${HLKU_NUCLEO_PORT:-/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_004700233434511834313937-if02}"
HLKU_MUX_LOG="${TMPDIR:-/tmp}/hl_ku_nucleo_mux.log"

cleanup_mux() {
  if [[ -n "${HLKU_MUX_PID}" ]] && kill -0 "${HLKU_MUX_PID}" 2>/dev/null; then
    kill -INT "${HLKU_MUX_PID}" 2>/dev/null || true
    wait "${HLKU_MUX_PID}" 2>/dev/null || true
  fi
}
trap cleanup_mux EXIT INT TERM

if [[ "${HLKU_NEEDS_HARDWARE}" == true ]]; then
  if pgrep -f '[n]ucleo_serial_mux' >/dev/null 2>&1 \
      && [[ -e /tmp/nucleo-control && -e /tmp/nucleo-lidar ]]; then
    echo "NUCLEO mux: 기존 프로세스 사용 (/tmp/nucleo-control, /tmp/nucleo-lidar)"
  else
    if [[ ! -e "${HLKU_NUCLEO_PORT}" ]]; then
      echo "NUCLEO mux: [대기] 포트 미연결 (${HLKU_NUCLEO_PORT}); TUI는 계속 엽니다." >&2
      echo "보드 연결 후 TUI를 다시 실행하세요. 다른 PC는 HLKU_NUCLEO_PORT로 포트를 지정할 수 있습니다." >&2
    fi
    if [[ -e "${HLKU_NUCLEO_PORT}" ]]; then
      ros2 run hl_ku_core nucleo_serial_mux \
        --port "${HLKU_NUCLEO_PORT}" \
        --control-link /tmp/nucleo-control \
        --lidar-link /tmp/nucleo-lidar \
        >"${HLKU_MUX_LOG}" 2>&1 &
      HLKU_MUX_PID=$!
      for _ in $(seq 1 40); do
        if [[ -e /tmp/nucleo-control && -e /tmp/nucleo-lidar ]]; then
          break
        fi
        if ! kill -0 "${HLKU_MUX_PID}" 2>/dev/null; then
          break
        fi
        sleep 0.1
      done
      if [[ -e /tmp/nucleo-control && -e /tmp/nucleo-lidar ]]; then
        echo "NUCLEO mux: [✓] control=/tmp/nucleo-control  lidar=/tmp/nucleo-lidar"
      else
        echo "NUCLEO mux: [대기] 시작 실패; 로그: ${HLKU_MUX_LOG}. TUI는 계속 엽니다." >&2
        if kill -0 "${HLKU_MUX_PID}" 2>/dev/null; then
          kill -INT "${HLKU_MUX_PID}" 2>/dev/null || true
        fi
        wait "${HLKU_MUX_PID}" 2>/dev/null || true
        HLKU_MUX_PID=""
      fi
    fi
  fi
fi

if [[ "${HLKU_OPEN_FOXGLOVE}" == true && -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  HLKU_FOXGLOVE_URL='foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765/'
  if command -v foxglove-studio >/dev/null 2>&1; then
    nohup foxglove-studio "${HLKU_FOXGLOVE_URL}" \
      >"${TMPDIR:-/tmp}/hl_ku_foxglove.log" 2>&1 </dev/null &
    disown || true
  elif command -v xdg-open >/dev/null 2>&1; then
    nohup xdg-open "${HLKU_FOXGLOVE_URL}" \
      >"${TMPDIR:-/tmp}/hl_ku_foxglove.log" 2>&1 </dev/null &
    disown || true
  fi
fi

set +e
ros2 run hl_ku_core mando_tui \
  --config "${HLKU_SCRIPT_DIR}/mando_scenarios.yaml" "${HLKU_TUI_ARGS[@]}"
HLKU_TUI_STATUS=$?
set -e
exit "${HLKU_TUI_STATUS}"
