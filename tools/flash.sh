#!/usr/bin/env bash
# Flash the CANBed M4 over USB.
#
#   ./bazelw run //src:flash            # or: tools/flash.sh path/to/firmware.uf2 path/to/firmware.bin
#
# Strategy:
#   1. If the UF2 bootloader drive (volume "SAME51G19A") is mounted, copy the .uf2 onto it.
#   2. Else, if a /dev/cu.usbmodem* port exists, do the 1200-baud touch so an
#      Arduino-style app reboots into the bootloader, then use the drive or bossac.
#   3. Otherwise wait for the user to double-tap RESET (up to FLASH_WAIT seconds).
set -euo pipefail

uf2="${1:?usage: flash.sh <firmware.uf2> [firmware.bin]}"
bin="${2:-${uf2%.uf2}.bin}"
APP_OFFSET="${APP_OFFSET:-0x4000}"
FLASH_WAIT="${FLASH_WAIT:-120}"

find_uf2_drive() {
  for v in /Volumes/*; do
    [ -f "$v/INFO_UF2.TXT" ] && { echo "$v"; return 0; }
  done
  return 1
}

find_port() { ls /dev/cu.usbmodem* 2>/dev/null | head -n1 || true; }

copy_uf2() {
  echo "UF2 bootloader drive at $1:"
  sed -n '1,3p' "$1/INFO_UF2.TXT"
  cp "$uf2" "$1/"
  echo "copied $(basename "$uf2") -> $1 (board resets into the new firmware)"
}

if drive="$(find_uf2_drive)"; then copy_uf2 "$drive"; exit 0; fi

port="$(find_port)"
if [ -n "$port" ]; then
  echo "No UF2 drive; sending 1200-baud touch on $port ..."
  python3 - "$port" <<'PY' || true
import sys, termios, os
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attr = termios.tcgetattr(fd)
attr[4] = attr[5] = termios.B1200
termios.tcsetattr(fd, termios.TCSANOW, attr)
os.close(fd)
PY
  sleep 3
  if drive="$(find_uf2_drive)"; then copy_uf2 "$drive"; exit 0; fi
  port="$(find_port)"
  if [ -n "$port" ] && command -v bossac >/dev/null; then
    exec bossac --port="$port" -U --offset="$APP_OFFSET" -e -w -v -R "$bin"
  fi
fi

echo "No bootloader drive or serial port found."
echo "Double-tap the RESET button on the CANBed M4; waiting up to ${FLASH_WAIT}s for the SAME51G19A drive ..."
for ((i = 0; i < FLASH_WAIT; i++)); do
  if drive="$(find_uf2_drive)"; then copy_uf2 "$drive"; exit 0; fi
  sleep 1
done
echo "timed out waiting for the bootloader drive" >&2
exit 1
