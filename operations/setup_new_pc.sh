#!/usr/bin/env bash
# Prepare a fresh Ubuntu 22.04 PC to run the HL Mando ROS 2 workspace.
set -eo pipefail

HLKU_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HLKU_ROOT="$(cd -- "${HLKU_SCRIPT_DIR}/.." && pwd)"
HLKU_WS="${HLKU_ROOT}/ros2_ws"
INSTALL_SYSTEM_PACKAGES=1
UPDATE_ROSDEP=1

usage() {
  cat <<EOF
Usage: ${HLKU_SCRIPT_DIR}/setup_new_pc.sh [options]

Options:
  --check-only             Do not install or build; run the environment checker.
  --skip-system-packages   Keep the current OS packages and build the workspace.
  --skip-rosdep-update     Skip the online rosdep database update.
  -h, --help               Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --check-only)
      exec "${HLKU_SCRIPT_DIR}/check_mando_pc.sh"
      ;;
    --skip-system-packages)
      INSTALL_SYSTEM_PACKAGES=0
      ;;
    --skip-rosdep-update)
      UPDATE_ROSDEP=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  elif command -v sudo >/dev/null 2>&1; then
    sudo "$@"
  else
    echo "sudo is required to install system packages." >&2
    exit 2
  fi
}

if [[ "${INSTALL_SYSTEM_PACKAGES}" -eq 1 ]]; then
  if [[ ! -r /etc/os-release ]]; then
    echo "Cannot identify this operating system." >&2
    exit 2
  fi
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    echo "This setup is tested on Ubuntu 22.04; detected ${PRETTY_NAME:-unknown}." >&2
    exit 2
  fi

  echo "[1/6] Installing base package tools"
  run_as_root apt-get update
  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ca-certificates curl gnupg locales lsb-release software-properties-common

  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[2/6] Configuring the official ROS 2 Humble apt repository"
    run_as_root add-apt-repository -y universe
    ros_key="$(mktemp)"
    ros_source="$(mktemp)"
    trap 'rm -f "${ros_key:-}" "${ros_source:-}"' EXIT
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o "${ros_key}"
    run_as_root install -m 0644 "${ros_key}" \
      /usr/share/keyrings/ros-archive-keyring.gpg
    architecture="$(dpkg --print-architecture)"
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
      "${architecture}" "${UBUNTU_CODENAME}" >"${ros_source}"
    run_as_root install -m 0644 "${ros_source}" \
      /etc/apt/sources.list.d/ros2.list
    rm -f "${ros_key}" "${ros_source}"
    trap - EXIT
  else
    echo "[2/6] ROS 2 Humble apt repository already available"
  fi

  echo "[3/6] Installing ROS, camera, MCAP, and Python dependencies"
  run_as_root apt-get update
  mapfile -t apt_packages < <(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' \
    "${HLKU_SCRIPT_DIR}/apt-packages.txt")
  run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    "${apt_packages[@]}"

  if ! command -v foxglove-studio >/dev/null 2>&1; then
    if apt-cache show foxglove-studio >/dev/null 2>&1; then
      run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y foxglove-studio
    else
      echo "[안내] Foxglove Desktop은 공식 다운로드 페이지에서 별도 설치하세요."
      echo "       https://foxglove.dev/download"
      echo "       저장소의 .foxe 확장은 지금 미리 설치합니다."
    fi
  fi
else
  echo "[1/6] System package installation skipped"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is missing: /opt/ros/humble/setup.bash" >&2
  exit 2
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "[4/6] Resolving ROS dependencies"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  run_as_root rosdep init
fi
if [[ "${UPDATE_ROSDEP}" -eq 1 ]]; then
  rosdep update
fi
rosdep install --from-paths "${HLKU_WS}/src" --ignore-src -r -y \
  --rosdistro humble

echo "[5/6] Building the HL_KU workspace"
cd "${HLKU_WS}"
colcon build --symlink-install

ntrip_example="${HLKU_WS}/src/hl_ku_core/config/ntrip_private.example.yaml"
ntrip_private="${HLKU_WS}/src/hl_ku_core/config/ntrip_private.yaml"
if [[ ! -f "${ntrip_private}" ]]; then
  umask 077
  cp "${ntrip_example}" "${ntrip_private}"
  echo "[설정 필요] ${ntrip_private}에 NTRIP 계정을 입력하세요."
fi

data_root="${HLKU_ROOT}/data"
mkdir -p "${data_root}/bags" "${HLKU_ROOT}/logs/mando_tui"

foxe="${HLKU_WS}/src/hl_ku_foxglove/foxglove-extension/hlku.hl-ku-fma-dashboard-0.1.2.foxe"
extension_root="${HOME}/.foxglove-studio/extensions"
extension_dir="${extension_root}/hlku.hl-ku-fma-dashboard-0.1.2"
if [[ -f "${foxe}" ]]; then
  temporary_extension="$(mktemp -d)"
  unzip -oq "${foxe}" -d "${temporary_extension}"
  mkdir -p "${extension_root}"
  rm -rf "${extension_root}"/hlku.hl-ku-fma-dashboard-*
  mv "${temporary_extension}" "${extension_dir}"
fi

login_user="${SUDO_USER:-${USER}}"
groups_to_add=()
getent group dialout >/dev/null 2>&1 && groups_to_add+=(dialout)
getent group video >/dev/null 2>&1 && groups_to_add+=(video)
if ((${#groups_to_add[@]})); then
  group_csv="$(IFS=,; echo "${groups_to_add[*]}")"
  run_as_root usermod -aG "${group_csv}" "${login_user}"
fi

echo "[6/6] Checking the completed setup"
"${HLKU_SCRIPT_DIR}/check_mando_pc.sh" || true

cat <<EOF

Setup finished.
1. Edit NTRIP credentials: ${ntrip_private}
2. Reconnect the GNSS, NUCLEO, and camera, then run:
   ${HLKU_SCRIPT_DIR}/check_mando_pc.sh
3. If group membership changed, log out and back in once.
4. Start the field console:
   ${HLKU_SCRIPT_DIR}/run_mando_tui.sh
EOF
