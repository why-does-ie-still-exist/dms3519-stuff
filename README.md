# CANBed M4 firmware (Zephyr)

Firmware for the Longan Labs **CANBed M4** (Microchip **ATSAME51G19A**, Cortex-M4F,
512 KiB flash, 192 KiB RAM, CAN FD via the on-chip Bosch M_CAN) on **Zephyr RTOS 4.4**.
The application runs a CAN loopback self-test (classic 2.0A and 64-byte CAN FD with
bit-rate switching) at boot, reports on a USB CDC console, and leaves the Zephyr
shell with the `can` commands available.

The earlier modm + Bazel setup is still in the tree (`src/`, `modm/`, `project.xml`,
`bazelw`, `MODULE.bazel`); see "Alternatives" at the end.

## Layout

```
app/                         Zephyr application and west manifest repository
  west.yml                   Zephyr v4.4.0 + hal_atmel + cmsis only
  CMakeLists.txt, prj.conf   app build; prj.conf enables CAN, CAN FD, shell, logging
  src/main.c                 loopback self-test, `loopback` shell command, 1200-baud reboot
  boards/longan/canbed_m4/   board definition (dts, pinctrl, defconfig, board.cmake)
  include/same51g19a.h       CMSIS device header for the 48-pin part (not in hal_atmel)
  zephyr/patches/            adds SOC_SAME51G19A to Zephyr (applied with `west patch apply`)
  dts/bindings/vendor-prefixes.txt   "longan" vendor prefix
zephyr/, modules/, .west/    west workspace (git-ignored, created by `west update`)
.venv/                       Python venv with west and Zephyr's requirements
build/zephyr/                build output (zephyr.elf, zephyr.uf2, zephyr.bin)
tools/flash.sh               copy a UF2 to the bootloader drive (waits for it)
tools/console.py             capture the USB CDC console
```

## Setup (once)

```sh
uv sync                                    # .venv with west, pyserial and Zephyr's requirements (pyproject.toml)
uv run west update --narrow -o=--depth=1   # clones zephyr/ and modules/ (~700 MB)
uv run west patch apply                    # SOC_SAME51G19A patch from app/zephyr/patches
brew install dtc bossa                     # dtc for devicetree checks, bossac optional
```

Every `west` or tool invocation below is `uv run ...`; no venv activation needed.

The workspace config already points at the Arm GNU Toolchain 15.3 in
`~/.local/opt` (`west config build.cmake-args`) and defaults the board to
`canbed_m4`; the Zephyr SDK is not required.

## Build, flash, test

```sh
uv run west build -d build/zephyr app      # add -p always for a clean build
tools/flash.sh build/zephyr/zephyr/zephyr.uf2   # 1200-baud touch, then copy to the UF2 drive
uv run tools/console.py --seconds 30       # watch the self-test output
```

Interactive shell: `screen /dev/cu.usbmodem* 115200` (quit with Ctrl-A K; a merely
detached screen keeps the port open and blocks flashing).

`tools/flash.sh` is the tested path: if the Zephyr firmware is running it opens the
CDC port at 1200 baud, Zephyr's `bossa.c` reboots the board into the UF2
bootloader, and the file is copied to the `SAME51G19A` drive. No button press
needed. If some other firmware is running (no CDC port), **double-tap RESET** to
get the drive and the script picks it up. `west flash -d build/zephyr` (uf2
runner) works when the drive is already mounted; `west flash -r bossac` should
also work but has not been exercised here.

Expected console output:

```
CANBed M4 (ATSAME51G19A) Zephyr 4.4.0, board canbed_m4
[00:00:01.234,000] <inf> loopback: can@42000000 capabilities: ... (loopback yes, FD yes)
[00:00:01.235,000] <inf> loopback: classic 2.0A: OK (id 0x123, 8 bytes)
[00:00:01.236,000] <inf> loopback: CAN FD 64B: OK (id 0x1abcdef, 64 bytes, FD+BRS)
*** CAN LOOPBACK SELF-TEST PASSED ***
uart:~$
```

Then in the shell: `loopback` re-runs the test; `can mode can@42000000 normal`,
`can start can@42000000`, `can send can@42000000 123 de ad be ef`,
`can recv can@42000000` drive the bus for real (needs a second node or termination).

## DaMiao DM-S3519-1EC motor

The target actuator is the DaMiao **DM-S3519-1EC** gearmotor with its separate
**DM3520-1EC** driver (BLDC, 19.2:1 planetary, 3.5 N.m rated / 7.8 N.m peak,
395 rpm rated, 24 V nominal, Kt 0.454 N.m/A at the output, single incremental
encoder so position is relative to power-up). DaMiao's material is cloned under
`ext/damiao/` (the top-level repo is only a shell of git submodules; the useful
ones were cloned over HTTPS: `SDK/电机SDK`, `SDK/电机控制例程`,
`电机产品/分立系列/DM-S3519-1EC`, `固件/电机固件`, `工具和上位机/电机调试上位机`).
Manual: `ext/damiao/电机产品/分立系列/DM-S3519-1EC/说明书/*.pdf` (V1.1, 2024-11).

### Driver in this repo

`app/src/dm/` is a small, portable C implementation of the DaMiao CAN protocol on
Zephyr's CAN API, ported from DaMiao's STM32 examples
(`ext/damiao/SDK/电机控制例程/stm32例程/dm_ctrl(h7 fdcan) v1.1 裸机/User` and
`dm_ctrl(DM3519 一拖四)/User`):

- `dm_motor_pack.c`: float<->uint mapping, MIT frame packing, feedback unpacking,
  DM3519 one-to-four current frames. No Zephyr dependency; `app/tests/host/run.sh`
  unit-tests it on the host.
- `dm_motor.c`: enable/disable/zero/clear-error, MIT / position-velocity /
  velocity / force-position frames, register read/write/save on 0x7FF with reply
  matching, feedback RX filters, `dm_motor_sync_limits()` to read PMAX/VMAX/TMAX
  from the driver, CAN FD (BRS) framing when the driver is set to >1 Mbit/s.
- `dm_shell.c`: `dm ...` shell commands over the USB console.

### Bus and IDs

- Default 1 Mbit/s CAN 2.0 (`bitrate = <1000000>` in the board dts). Register
  `CAN_BR` (RID 35) codes 0..9 = 125k, 200k, 250k, 500k, 1M, 2M, 2.5M, 3.2M, 4M,
  5M; a code above 1M switches the driver to CAN FD with bit-rate switching, with
  the arbitration phase staying at 1 Mbit/s. Then use `dm bus normal fd` and set
  `bitrate-data` in the dts to match.
- `ESC_ID` (RID 8) is the command ID; `MST_ID` (RID 7) is the feedback ID
  (factory default 0). Feedback and register replies both arrive on `MST_ID`; the
  driver only compares the low 8 bits of a command ID, the high 3 bits carry the
  mode offset.
- Frames are always <= 8 bytes, even in FD mode.

### Control modes (register CTRL_MODE, RID 10)

| Mode | CMODE | Command ID | Payload |
|---|---|---|---|
| MIT | 1 | `0x000 + ESC_ID` | pos u16 over [-PMAX, PMAX], vel u12 over [-VMAX, VMAX], Kp u12 over [0, 500], Kd u12 over [0, 5], torque u12 over [-TMAX, TMAX] |
| Position-velocity | 2 | `0x100 + ESC_ID` | float pos (rad), float max vel (rad/s); ACC/DEC registers shape the move |
| Velocity | 3 | `0x200 + ESC_ID` | float vel (rad/s), 4-byte frame |
| Force-position hybrid | 4 | `0x300 + ESC_ID` | float pos, u16 vel x100, u16 current per-unit x10000 (documented for the J series; the S3519 manual lists CMODE range 0..4 but only describes 1-3) |

MIT mode is the general one: Kp = Kd = 0 with a torque value gives pure torque
control; Kp/Kd with a position gives an impedance/PD joint; the manual warns Kd
must not be 0 when Kp is used. Units are rad, rad/s and N.m at the output shaft.
Feedback (8 bytes on `MST_ID`, sent in reply to every command; there is no
periodic broadcast): `id|state`, pos u16, vel u12, torque u12, T_mos, T_coil.
State nibble: 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A
over-current, B MOS over-temp, C coil over-temp, D comm loss, E overload.

Special 8-byte commands on the mode's ID: enable `FF..FF FC`, disable `FF..FF
FD`, set zero `FF..FF FE`, clear error `FF..FF FB`. Registers: read `0x7FF: idL idH
33 rid`, write `0x7FF: idL idH 55 rid v0..v3` (applied immediately, volatile),
save `0x7FF: idL idH AA 01` (only while disabled, ~30 ms, limited flash cycles).

Mapping ranges: the official 2026 selection table gives the S3519 defaults
**PMAX 12.5 rad, VMAX 200 rad/s, TMAX 10 N.m**; DaMiao's own SDK tables carry a
different, self-flagged "check" entry, so the driver reads RIDs 21-23 from the
motor at `dm init` and uses those when it answers.

A separate "one-to-four" firmware exists for the DM3519 driver that replaces the
protocol above with a DJI C620-style current-only scheme (command 0x200/0x1FF
with four int16 currents, 16384 = 20.5 A; broadcast feedback on 0x200+n). The
driver supports it (`dm cur3519`, `fb`) but the stock firmware is the MIT one.

### Velocity control (the intended use)

The board boots into normal CAN 2.0 mode at 1 Mbit/s after its loopback
self-test, so with the motor wired up:

```
dm init 1 0                # ESC_ID 1, MST_ID 0 (check with dm info); reads PMAX/VMAX/TMAX
dm velmode                 # disable, CTRL_MODE=3, enable on 0x201, stream 0 rad/s at 100 Hz
dm velmode save            # same, but also persist CTRL_MODE=3 in the driver's flash
dm vel 6.28                # 1 rev/s at the output shaft (rad/s); streamed continuously
dm stream 100 10           # 100 Hz with a 10 rad/s^2 ramp toward each new setpoint
dm status                  # setpoint, frames sent, last feedback (pos/vel/torque/temps/state)
dm stop                    # setpoint 0 (motor holds at zero speed, still enabled)
dm disable                 # motor free
```

Measured on the real motor (2026-09-17): `dm vel 0.1` and `dm vel 0.5` turn the
output shaft at 0.10 and 0.51 rad/s. The feedback **velocity** is in output-shaft
units, but the feedback **position** is the rotor (it equals register 80 `p_m`;
register 81 `x_out` reads 0 on this single-encoder motor), so it advances 19.2x
faster than the output and wraps at +-PMAX. The 0xCC "request feedback" command
did not produce a frame on this firmware (SW_VER string "6720"); feedback only
comes in reply to control frames.

Why the streamer: the driver watches its TIMEOUT register and drops to the
comm-loss error state (D) when commands stop arriving, so a velocity setpoint
has to be re-sent continuously. Each command also triggers one feedback frame,
so at 100 Hz you get 100 Hz feedback. The velocity frame is a 4-byte float in
rad/s at the output shaft on ID `0x200 + ESC_ID`; the speed loop gains live in
KP_ASR/KI_ASR (RIDs 25/26) and the limit in MAX_SPD (RID 6). Feedback velocity
is quantised over +-VMAX (12 bits), so with VMAX 200 the resolution is about
0.1 rad/s; lower VMAX in the driver (RID 22, then `dm save`) for finer readout
of slow motion. The S3519 no-load speed of 435 rpm is about 45 rad/s.

### Other modes from the shell

```
dm write 10 1              # CTRL_MODE = MIT (volatile until "dm save" while disabled)
dm mode mit                # enable/disable frames on the 0x000 offset
dm enable                  # sends FF..FC three times
dm mit 0 0 0 0 0.5         # 0.5 N.m torque, no position/velocity term
dm mit 1.57 0 10 0.5 0     # hold 90 deg with Kp 10, Kd 0.5
dm fb                      # last feedback
dm disable
```

Caveats: wire the CAN bus with 120 ohm termination at both ends (slide switch
SW1 on the CANBed enables the on-board resistor); the S3519 position is zero at
power-up (single encoder); and see the clock note above before running at
FD data rates.

## Board definition notes

- SoC: Zephyr 4.4 only lists the 64/100-pin SAME51 parts, so
  `app/zephyr/patches/same51g19a-soc.patch` adds `same51g19a` to `soc.yml`,
  `Kconfig.soc` and `soc.h`. hal_atmel lacks `same51g19a.h`; the copy in
  `app/include/` is the unmodified Microchip CMSIS header (Apache-2.0, same
  1.1 generation as the HAL's `same51j19a.h`).
- Clocks: no crystal, so `CONFIG_SOC_ATMEL_SAMD5X_OSCULP32K_AS_MAIN=y`. Zephyr
  runs the CPU at 120 MHz from DPLL0 (referenced to OSCULP32K) and USB/CAN from
  the 48 MHz DFLL in open-loop mode. That is fine for loopback and USB in
  practice, but the DFLL is only factory-trimmed, so for a real bus at high
  bitrates consider adding USB clock recovery (DFLL `USBCRM`) or an external
  clock; the bit timing tolerance of CAN is about 1.5 %.
- CAN0: PA22 TX / PA23 RX (function I), MCP2542FD with STBY grounded. GCLK7 from
  the DFLL feeds the controller (48 MHz). Default 500 kbit/s nominal, 2 Mbit/s data.
- USB: legacy `USB_DEVICE_STACK` (deprecated in 4.4 but still the one with
  the BOSSA 1200-baud hook). Enumerates as `03EB:802B` "CANBed M4 (Zephyr)".
- Flash layout: 16 KiB bootloader partition, code at 0x4000, last 16 KiB left
  as a storage partition. `CONFIG_BUILD_OUTPUT_UF2` writes `zephyr.uf2` with
  the SAMD51 family ID.

## Board facts (from the schematic and the Longan Arduino variant)

| Item | Detail |
|---|---|
| Bootloader | Adafruit/Microsoft `uf2-samdx1` v3.10, 16 KiB at 0x0000; app at **0x4000**. Also speaks SAM-BA (bossac). USB `03EB:00CD`, MSC volume `SAME51G19A`. |
| App USB ID | `03EB:802B` (Arduino-convention PID from the Longan core; not a registered Microchip PID) |
| Clock | No crystal. DFLL48M open loop after reset (48 MHz); the Arduino core uses USB clock recovery. |
| User LED | D5 blue, **PA07**, active high |
| CAN | **CAN0**: PA22 TX, PA23 RX (peripheral function I). MCP2542FD transceiver, STBY grounded. 120 Ω termination on slide switch SW1. Red TX/RX LEDs on the same lines. |
| UART | SERCOM3: PA16 RX, PA17 TX (Grove + 2x9 header) |
| I2C / SPI | SERCOM2 PA12 SDA / PA13 SCL;  SERCOM1 PA00 MOSI / PA01 SCK / PB23 MISO |
| SWD | J13: 3V3, SWDIO (PA31), SWCLK (PA30), GND |
| Power | 7-28 V in via TPS54302 buck → 5 V → AMS1117 3V3 |

Sources: wiki <https://docs.longan-labs.cc/1030013/>, schematic
<https://github.com/Longan-Labs/Hardware_CANBed_Series>, board package
<https://github.com/Longan-Labs/LONGAN-SAME-TOOLS> (`variants/CANBED_M4/`),
CAN library <https://github.com/Longan-Labs/CANBed_M4_Arduino_Lib> (useful as a
register-level reference for CAN0 setup).

## Debugging over SWD

lbuild generates `modm/openocd.cfg` (target `atsame5x`) and `modm/gdbinit*`.  With a
CMSIS-DAP probe on header J13 (`-f interface/cmsis-dap.cfg`; J-Link: `interface/jlink.cfg`):

```sh
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -f modm/openocd.cfg \
        -c "program bazel-bin/src/firmware.elf verify reset exit"
openocd -f interface/cmsis-dap.cfg -c "transport select swd" -f modm/openocd.cfg &
arm-none-eabi-gdb -x modm/gdbinit_openocd bazel-bin/src/firmware.elf
```

The image is linked at 0x4000 (`linkerscript.flash_offset` in `project.xml`), so
SWD programming leaves the bootloader intact.

## Alternatives (researched 2026-09-16)

Ranked for a plain, IDE-free build on macOS with `arm-none-eabi-gcc`:

1. **Bare-metal + vendored Microchip DFP + CMSIS core.** The DFP is Apache-2.0,
   current (3.9.267, Nov 2025), and ships exactly the four files a GCC build
   needs plus an SVD. This was the first version of this repo; it was replaced
   by modm (below) for the typed GPIO/clock/peripheral API and startup code.
2. **Zephyr 4.4 (this repo).** Has the SAM0 tree, a Bosch M_CAN driver
   (`can_sam0`) with SAME51 clock handling, USB device and `west flash` runners
   for bossac, uf2 and OpenOCD. There was no `SOC_SAME51G19A` in-tree and no
   board for this module; both are added here.
3. **modm.** Supports `same51g19a`, has a clean C++ HAL for the SAM D5x/E5x
   (GPIO, GCLK, SERCOM UART, USB via TinyUSB, SysTick, fibers), and was the
   second iteration of this repo (`./bazelw build //src:uf2` still builds the
   blink). Its SAM MCAN driver excludes every D5x/E5x part, so CAN would be
   hand-written, which is why the project moved to Zephyr.
4. **Pigweed (Bazel).** Good Cortex-M4 toolchain, but no SAM target and not on
   the BCR; heavy for a small project.
5. **Microchip ASF4 / Atmel START** is deprecated; **MPLAB Harmony CSP** is
   maintained but assumes MPLAB X and MCC.
6. **Rust** `atsamd-hal` (`same51g` feature) + `mcan` crate is the strongest
   non-C option, including CAN-FD.

Flashing/debug tools on macOS: `bossac` (Homebrew formula `bossa` 1.9.1; always
pass `--offset=0x4000`), `uf2conv.py`, OpenOCD 0.12 (`target/atsame5x.cfg`), and
pyOCD (`pyocd pack install atsame51g19a` pulls the same DFP, whose 3.9.267
release added the CMSIS flash algorithms).
