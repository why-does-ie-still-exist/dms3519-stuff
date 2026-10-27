#!/usr/bin/env bash
# Open a screen session on the CANBed's USB serial console, whatever the
# device node is called today.
#
#   tools/console.sh            # find the board, open screen at 115200
#   tools/console.sh --wait     # wait for the board to appear first (e.g. after a flash)
#   tools/console.sh /dev/ttyACM1   # use this port instead of searching
#
# Search order: the stable by-id symlink that names the board (Linux), then
# macOS cu.usbmodem*, then generic ttyACM* / ttyUSB*.  If several candidates
# remain, the board's USB product string decides.  Quit screen with Ctrl-A K.
set -euo pipefail

BAUD="${BAUD:-115200}"
wait_for=0
port=""
for arg in "$@"; do
  case "$arg" in
    --wait) wait_for=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) port="$arg" ;;
  esac
done

# Print the board's port, or nothing.
find_port() {
  local p
  # Linux: udev's by-id link embeds the USB product string, so it is stable
  # across ttyACM numbering and even across machines.
  for p in /dev/serial/by-id/*CANBed* /dev/serial/by-id/*Longan*; do
    [ -e "$p" ] && { readlink -f "$p"; return 0; }
  done
  # macOS, and Linux without the by-id link.
  local candidates=()
  for p in /dev/cu.usbmodem* /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$p" ] && candidates+=("$p")
  done
  [ ${#candidates[@]} -eq 0 ] && return 1
  [ ${#candidates[@]} -eq 1 ] && { echo "${candidates[0]}"; return 0; }
  # Several serial devices: pick the one whose USB product string says CANBed.
  for p in "${candidates[@]}"; do
    if command -v udevadm >/dev/null 2>&1; then
      udevadm info -q property -n "$p" 2>/dev/null | grep -qi 'canbed' && { echo "$p"; return 0; }
    elif [ "$(uname -s)" = Darwin ]; then
      ioreg -r -c IOUSBHostDevice -l 2>/dev/null | grep -qi 'canbed' && { echo "$p"; return 0; }
    fi
  done
  echo "console.sh: several serial ports, none identified as the CANBed:" >&2
  printf '  %s\n' "${candidates[@]}" >&2
  echo "pass the right one explicitly" >&2
  return 2
}

if [ -z "$port" ]; then
  if [ "$wait_for" = 1 ]; then
    echo "waiting for the CANBed ..." >&2
    until port="$(find_port 2>/dev/null)"; do sleep 0.5; done
    sleep 1  # let the CDC port finish enumerating
  else
    port="$(find_port)" || {
      [ $? -eq 2 ] && exit 2
      echo "console.sh: no CANBed found. Is it plugged in? (--wait waits for it)" >&2
      exit 1
    }
  fi
fi

# A stale detached screen keeps the port busy and looks like a dead board.
if command -v fuser >/dev/null 2>&1 && fuser -s "$port" 2>/dev/null; then
  echo "console.sh: $port is already open by:" >&2
  fuser -v "$port" 2>&1 | sed 's/^/  /' >&2
  echo "close it first (screen -ls; screen -X -S <id> quit)" >&2
  exit 1
fi
if [ ! -w "$port" ]; then
  echo "console.sh: no write access to $port; add yourself to the '$(stat -c %G "$port" 2>/dev/null || echo dialout)' group" >&2
  exit 1
fi

command -v screen >/dev/null 2>&1 || {
  echo "console.sh: 'screen' is not installed (apt install screen / brew install screen)" >&2
  exit 1
}
echo "screen $port $BAUD   (quit: Ctrl-A K)" >&2
exec screen "$port" "$BAUD"
