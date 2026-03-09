#!/usr/bin/env bash
set -euo pipefail

PORT="/dev/cu.usbmodem101"
FLASH_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--port)
      PORT="${2:-}"
      shift 2
      ;;
    --flash-only)
      FLASH_ONLY=1
      shift
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./run_motor_test.sh [options]

Options:
  -p, --port <port>   Serial port (default: /dev/cu.usbmodem101)
  --flash-only        Only build/flash firmware, do not open terminal console
  -h, --help          Show this help

Examples:
  ./run_motor_test.sh
  ./run_motor_test.sh --port /dev/cu.usbmodem101
  ./run_motor_test.sh --flash-only
EOF
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Run with --help for usage." >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IDF_EXPORT="${HOME}/esp/esp-idf/export.sh"

if [[ ! -f "${IDF_EXPORT}" ]]; then
  echo "ESP-IDF export script not found at: ${IDF_EXPORT}" >&2
  exit 1
fi

echo "Loading ESP-IDF environment..."
# shellcheck disable=SC1090
source "${IDF_EXPORT}" >/dev/null 2>&1

echo "Flashing firmware to ${PORT}..."
idf.py -C "${SCRIPT_DIR}" -p "${PORT}" flash

if [[ "${FLASH_ONLY}" -eq 1 ]]; then
  echo "Flash complete."
  exit 0
fi

echo "Starting motor terminal console..."
python3 "${SCRIPT_DIR}/motor_terminal.py" --port "${PORT}"
