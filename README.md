# CANBed M4 + DaMiao DM-S3519 (Zephyr)

Zephyr 4.4 firmware for the Longan Labs CANBed M4 (ATSAME51G19A, CAN FD) that
drives a DaMiao DM-S3519-1EC gearmotor (DM3520-1EC driver) over CAN. A USB
serial shell exposes velocity, torque/current, position and register commands.

## Setup

```sh
uv sync                                     # west + Zephyr python deps (pyproject.toml)
uv run west update --narrow -o=--depth=1    # clones zephyr/ and modules/
uv run west patch apply                     # adds SOC_SAME51G19A to Zephyr
brew install dtc bossa                      # optional: devicetree checks, bossac
```

Compiler: Arm GNU Toolchain in `~/.local/opt/arm-gnu-toolchain-*-arm-none-eabi`
(set via `west config build.cmake-args`; see `.west/config`). No Zephyr SDK needed.

## Build and flash

```sh
uv run west build -d build/zephyr app
tools/flash.sh build/zephyr/zephyr/zephyr.uf2
```

`flash.sh` reboots a running Zephyr image into the UF2 bootloader (1200-baud
touch) and copies the file. If no serial port exists, double-tap RESET so the
`SAME51G19A` drive appears. Close any terminal on the port first; a detached
`screen` still blocks it.

## Shell

```sh
screen /dev/cu.usbmodem* 115200             # quit: Ctrl-A K
uv run tools/console.py --seconds 30        # or: capture-only
```

### Velocity control

```
dm init 1 0        # ESC_ID 1, MST_ID 0; reads PMAX/VMAX/TMAX from the driver
dm velmode         # CTRL_MODE=3, enable, stream setpoint at 100 Hz (add "save" to persist mode)
dm vel 0.5         # rad/s at the output shaft, ramped at 2 rad/s^2
dm status
dm stop
dm disable
```

### Torque / current control

```
dm init 1 0
dm torquemode      # MIT mode with Kp=Kd=0, streaming the torque term
dm current 0.5     # A, via Kt 0.4537 N.m/A (or: dm torque <N.m>)
dm speedcap 10     # cut torque above 10 rad/s (default 20); dm torqueramp <N.m/s>
dm stop
dm disable
```

### Other

`dm info` (registers), `dm read/write/save <rid>`, `dm mit <pos> <vel> <kp> <kd> <tor>`,
`dm pos <pos> <vel>`, `dm bus normal|loopback [fd]`, `dm fb`, `can ...` (Zephyr CAN shell).

## Notes

- Bus: 1 Mbit/s CAN 2.0. Driver register CAN_BR (RID 35) > 4 switches it to CAN FD
  with BRS; then `dm bus normal fd` and match `bitrate-data` in the board dts.
- Modes (RID 10): 1 MIT (ID 0x000+ESC_ID), 2 position-velocity (0x100+), 3 velocity
  (0x200+), 4 force-position (0x300+). Feedback and register replies arrive on MST_ID.
- Feedback position is the rotor (19.2x the output); velocity is output rad/s with
  0.1 rad/s resolution at VMAX 200. Lower VMAX (RID 22) for finer readout.
- The driver drops to comm-loss if commands stop; the shell streams setpoints for you.
- The board has no crystal: fine for CAN 2.0 and USB, add clock recovery before
  trusting high FD data rates.
- Board pins: LED PA07, CAN0 PA22/PA23 (MCP2542FD, STBY grounded), Serial1
  SERCOM3 PA16/PA17, SWD on J13. 120 ohm termination via SW1.

## Layout

```
app/boards/longan/canbed_m4/   board definition
app/zephyr/patches/            SOC_SAME51G19A patch (west patch apply)
app/include/same51g19a.h       CMSIS header for the 48-pin part (not in hal_atmel)
app/src/dm/                    DaMiao protocol driver + shell (app/tests/host/run.sh tests it)
tools/                         flash.sh, console.py, uf2conv.py
ext/                           (git-ignored) clones: modm, DaMiao docs/SDK
```

Earlier modm + Bazel iteration (`src/`, `bazel/`, `bazelw`, `MODULE.bazel`,
`project.xml`) still builds a blink with `./bazelw build //src:uf2`.
