#!/usr/bin/env bash
# Compatibility entry point. New commands live in ../operations.
set -eo pipefail

HLKU_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${HLKU_WS_DIR}/../operations/run_finger_drive.sh" "$@"
