#!/usr/bin/env bash
# Read-only software, configuration, and device check for the Mando workstation.
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HLKU_ROOT="$(cd -- "${HLKU_SCRIPT_DIR}/.." && pwd)"
HLKU_WS="${HLKU_ROOT}/ros2_ws"
failures=0
waiting=0

ok() { printf '[✓] %s\n' "$*"; }
fail() { printf '[실패] %s\n' "$*"; failures=$((failures + 1)); }
wait_for() { printf '[대기] %s\n' "$*"; waiting=$((waiting + 1)); }

extract_yaml_scalar() {
  local file="$1" section="$2" key="$3"
  awk -v section="${section}:" -v key="${key}:" '
    $1 == section {inside=1; next}
    inside && /^[^[:space:]#]/ {inside=0}
    inside && $1 == key {sub(/^[^:]+:[[:space:]]*/, ""); print; exit}
  ' "${file}" | sed -e 's/^['"'"'"]//' -e 's/['"'"'"]$//'
}

echo "HL MANDO PC CHECK"
echo "repo: ${HLKU_ROOT}"
echo

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    ok "OS: ${PRETTY_NAME}"
  else
    wait_for "OS: Ubuntu 22.04 권장, 현재 ${PRETTY_NAME:-unknown}"
  fi
else
  fail "OS 정보를 읽을 수 없음"
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  ok "ROS 2 Humble"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  fail "ROS 2 Humble 없음: ./operations/setup_new_pc.sh 실행"
fi

for command_name in python3 colcon rosdep; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    ok "명령: ${command_name}"
  else
    fail "명령 없음: ${command_name}"
  fi
done

if python3 -c 'import cv2, serial, yaml' >/dev/null 2>&1; then
  ok "Python: cv2 / serial / yaml"
else
  fail "Python 모듈 누락: cv2, serial 또는 yaml"
fi

if [[ -f "${HLKU_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${HLKU_WS}/install/setup.bash"
  missing_packages=()
  for package_name in hl_ku_interfaces hl_ku_core hl_ku_foxglove \
    foxglove_bridge usb_cam rosbag2_storage_mcap rplidar_ros; do
    ros2 pkg prefix "${package_name}" >/dev/null 2>&1 || \
      missing_packages+=("${package_name}")
  done
  if ((${#missing_packages[@]} == 0)); then
    ok "워크스페이스와 ROS 패키지"
  else
    fail "ROS 패키지 누락: ${missing_packages[*]}"
  fi
else
  fail "워크스페이스 미빌드: ./operations/setup_new_pc.sh 실행"
fi

required_files=(
  "${HLKU_SCRIPT_DIR}/mando_scenarios.yaml"
  "${HLKU_WS}/src/hl_ku_core/routes/course_07_vehicle.csv"
  "${HLKU_WS}/src/hl_ku_core/routes/mando/konkuk_straight.csv"
  "${HLKU_WS}/src/hl_ku_core/routes/mando/konkuk_curve.csv"
  "${HLKU_WS}/src/hl_ku_foxglove/config/HL_KU_FMA_Replay.json"
  "${HLKU_WS}/src/hl_ku_foxglove/foxglove-extension/hlku.hl-ku-fma-dashboard-0.1.2.foxe"
)
missing_files=()
for required_file in "${required_files[@]}"; do
  [[ -f "${required_file}" ]] || missing_files+=("${required_file#${HLKU_ROOT}/}")
done
if ((${#missing_files[@]} == 0)); then
  ok "TUI 경로 / Foxglove 레이아웃 / 확장"
else
  fail "저장소 파일 누락: ${missing_files[*]}"
fi

ntrip_private="${HLKU_WS}/src/hl_ku_core/config/ntrip_private.yaml"
if [[ ! -f "${ntrip_private}" ]]; then
  wait_for "NTRIP 개인 설정 없음: ntrip_private.example.yaml을 복사해 입력"
elif python3 - "${ntrip_private}" <<'PY' >/dev/null 2>&1
import sys, yaml
data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
p = data.get("ntrip_client", {}).get("ros__parameters", {})
bad = {"", "caster.example.com", "MOUNTPOINT", "USERNAME", "PASSWORD"}
raise SystemExit(0 if all(str(p.get(k, "")).strip() not in bad for k in
                          ("host", "mountpoint", "username", "password")) else 1)
PY
then
  ok "NTRIP 개인 설정 입력됨 (값은 표시하지 않음)"
else
  wait_for "NTRIP 예시값 상태: ntrip_private.yaml 수정 필요"
fi

gps_config="${HLKU_WS}/src/hl_ku_core/config/gps_only_route.yaml"
gnss_device="$(extract_yaml_scalar "${gps_config}" um982_serial port)"
nucleo_device="${HLKU_NUCLEO_PORT:-/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_004700233434511834313937-if02}"
camera_device="$(extract_yaml_scalar "${HLKU_SCRIPT_DIR}/mando_scenarios.yaml" common camera_device)"

for label_and_path in \
  "GNSS|${gnss_device}" \
  "NUCLEO|${nucleo_device}" \
  "카메라|${camera_device}"; do
  label="${label_and_path%%|*}"
  device_path="${label_and_path#*|}"
  if [[ -n "${device_path}" && -e "${device_path}" ]]; then
    ok "${label}: ${device_path}"
  else
    wait_for "${label} 장치 미연결 또는 식별자 불일치: ${device_path:-설정 없음}"
  fi
done

if command -v foxglove-studio >/dev/null 2>&1; then
  ok "Foxglove Desktop"
else
  wait_for "Foxglove Desktop 미설치: https://foxglove.dev/download"
fi

extension_dir="${HOME}/.foxglove-studio/extensions/hlku.hl-ku-fma-dashboard-0.1.2"
if [[ -f "${extension_dir}/dist/extension.js" ]]; then
  ok "Foxglove HL KU 확장 설치됨"
else
  wait_for "Foxglove 확장 미설치: ./operations/setup_new_pc.sh 실행"
fi

echo
if ((failures > 0)); then
  printf '결과: 실패 %d개, 연결/입력 대기 %d개\n' "${failures}" "${waiting}"
  exit 1
fi
printf '결과: 소프트웨어 준비 완료, 연결/입력 대기 %d개\n' "${waiting}"
