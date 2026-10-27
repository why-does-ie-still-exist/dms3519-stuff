# CANBed M4 + DaMiao DM-S3519 (Zephyr)

Zephyr 4.4 firmware for the Longan Labs CANBed M4 (ATSAME51G19A, CAN FD) that
drives a DaMiao DM-S3519-1EC gearmotor (DM3520-1EC driver) over CAN. A USB
serial shell exposes velocity, torque/current, position and register commands.

## Setup

```sh
uv sync                                     # west + Zephyr python deps (pyproject.toml)
uv run west init -l app                     # creates .west/ (not in git)
uv run west update --narrow -o=--depth=1    # clones zephyr/ and modules/
uv run west patch apply                     # adds SOC_SAME51G19A to Zephyr
brew install dtc                            # optional: devicetree checks
```

Compiler: Arm GNU Toolchain in `~/.local/opt/arm-gnu-toolchain-*-arm-none-eabi`
(download the `darwin-arm64-arm-none-eabi` tarball and untar it there). No
Zephyr SDK needed. `.west/config` is git-ignored, so a fresh clone has to set
the board and the toolchain itself — without the first line `west build` fails
with `BOARD is not being defined`:

```sh
uv run west config build.board canbed_m4
uv run west config build.cmake-args -- \
  "-DZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb -DGNUARMEMB_TOOLCHAIN_PATH=$HOME/.local/opt/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi"
```

## Build and flash

```sh
uv run west build -d build/zephyr app
tools/flash.sh build/zephyr/zephyr/zephyr.uf2
```

`flash.sh` reboots a running Zephyr image into the UF2 bootloader (1200-baud
touch) and copies the file. If no serial port exists, double-tap RESET so the
`SAME51G19A` drive appears. Close any terminal on the port first; a detached
`screen` still blocks it.

The 1200-baud touch only works once a Zephyr image is running; the stock
Longan firmware ignores it, so the first flash needs a manual double-tap.

Do not reach for `bossac`: this bootloader does **not** speak SAM-BA. Homebrew's
bossa 1.9.1 reports `Device unsupported`, and the Adafruit fork gets as far as
`SAM-BA operation failed` and then wedges the bootloader until the board is
unplugged. UF2 is the only USB path; SWD on J13 is the alternative.

### If the UF2 drive will not mount

Managed Macs often refuse to mount the bootloader volume: `diskutil mount` fails
with `SUIS premount dissented` and Disk Utility greys out the Mount button. This
is an MDM policy denying external volumes, not a permission you can approve —
confirm it with:

```sh
plutil -p /Library/Managed\ Preferences/com.apple.systemuiserver.plist | grep -A4 mount-controls
```

`"harddisk-external" = (deny)` means no amount of clicking will mount it. Write
to the raw device instead: the bootloader parses incoming blocks by UF2 magic
number and ignores the LBA they land on, so a plain `dd` works with no volume
mounted.

```sh
diskutil list external                      # find the SAME51G19A disk
sudo dd if=build/zephyr/zephyr/zephyr.uf2 of=/dev/rdiskN bs=512
```

Use `/dev/rdiskN` (raw), not `/dev/diskN`, and keep `bs=512` — UF2 is a stream of
512-byte blocks. The device node is `root:operator`, so this needs `sudo` even if
your account is an admin. Each block carries its own `targetAddr` of `0x4000`, so
this cannot overwrite the bootloader however the file is written. The board
resets itself and re-enumerates as `CANBed M4 (Zephyr)`.

To get into the bootloader without the drive mounted, send the 1200-baud touch
directly; the port disappearing mid-call is the reboot, not a failure:

```sh
uv run python -c "import serial; s=serial.Serial('/dev/cu.usbmodem*',1200); s.dtr=False; s.close()"
```

## Shell

```sh
tools/console.sh                            # finds the board's port, opens screen; quit: Ctrl-A K
tools/console.sh --wait                     # wait for the board to enumerate (after a flash)
uv run tools/console.py --seconds 30        # or: capture-only
```

`console.sh` looks for the `/dev/serial/by-id/*CANBed*` link on Linux and
`/dev/cu.usbmodem*` on macOS, so it works whatever number the tty gets, and
it refuses to start if a stale `screen` still holds the port.

### Several motors at once

**Single-motor lockout (default on).** While the motors share one thin supply
lead, only one may be energised at a time: `dm enable`, the mode commands and
the one-shot drive commands are refused while another slot is enabled or
streaming, naming the motor in the way. `dm disable`, `dm stop` + `dm stream 0`
or `dm release` on that motor clears it. `dm lockout off` lifts the rule once
the wiring can carry the current; `dm lockout` shows the state.


Each motor gets its own slot with its own control loop, so `dm init` is really
"select": the first `dm init 3` sets the motor up, a later `dm init 3` just
switches the shell's commands back to it and leaves it (and every other motor)
running. `dm motors` shows what each slot is doing; `dm release [esc]` stops,
disables and forgets one. Loop settings such as `torquecap`, `velgains`,
`watchdog` and `speedcap` are per motor, so select the motor before tuning it.

```
dm init 1          # select/set up motor 1
dm safevelmode
dm vel 0.5
dm init 2          # motor 1 keeps going
dm safevelmode
dm vel -0.5
dm motors          # * marks the selected one
dm init 1          # back to motor 1, still at 0.5 rad/s
dm stop
```

`dm init` warns when the driver's TIMEOUT register (rid 9) is 0: that motor
holds its last command for ever if its frames stop, so the shell never leaves a
motor's stream running unattended on purpose. `dm scan` and `set_can_id.py`
refuse to run while any motor is streaming.

### Velocity control

```
dm init 1          # ESC_ID 1, MST_ID 0x11 (esc + 0x10); 'dm init 1 0' to give MST_ID explicitly
dm velmode         # CTRL_MODE=3, enable, stream setpoint at 100 Hz (add "save" to persist mode)
dm vel 0.5         # rad/s at the output shaft, ramped at 2 rad/s^2
dm status
dm stop
dm disable
```

### Safe velocity control (torque-capped)

Velocity control that cannot pull more than a set torque. Unlike `dm speedcap`
in torque mode, hitting the cap is **not** a trip: a stalled motor pushes at
exactly the cap and holds there, and goes back to tracking the setpoint by
itself once the load is removed. The loop runs on the MCU at 1 kHz.

```
dm init 1
dm torquecap 1.5   # N.m at the output shaft (default 0.5, fine unloaded); 0 means no torque
dm safevelmode     # MIT mode, MCU-side velocity loop at 1 kHz; starts idle
dm vel 0.5         # rad/s at the output shaft, ramped by 'dm stream <hz> <ramp>'
dm status          # shows the torque being used and "[AT CAP]" when saturated
dm stop            # idle: 0 N.m until the next 'dm vel'
dm disable
```

`dm stop` idles the loop at zero torque rather than regulating to 0 rad/s. Those
are different things: a zero-speed setpoint leaves the loop fighting feedback
noise at standstill. `dm vel 0` still asks for an actively braked zero.

#### How the loop deals with friction (and why it does not lurch)

An unloaded DM-S3519 needs about 0.34 N.m to break away but only 0.14 N.m to
keep turning. A plain PI loop winds its integrator up to the breakaway torque,
then carries that excess into the move and overshoots by 70% or more while it
unwinds (measured with `tools/vel_step.py`). The loop therefore has:

- a kinetic-friction feed-forward (default 0.14 N.m) in the direction of the
  command, so the integrator only carries the residual;
- a breakaway kick (default 0.34 N.m) applied once the shaft has read
  "not moving" for 20 ms while a move is commanded, dropped the moment the
  shaft is clearly moving, with the integrator frozen while it is applied;
- an integrator clamp (default 0.15 N.m) well below the torque cap;
- an independent runaway guard on the driver's own velocity: if it exceeds
  1.5x the setpoint plus 1 rad/s the loop idles at 0 N.m and says so, since
  that means the estimator below has lost the plot;
- velocity derived from the **rotor position** rather than the driver's own
  velocity estimate. On this driver the feedback position is the rotor angle
  while the velocity is at the output shaft; differencing the rotor angle over
  10 ms and dividing by the gear ratio (register 20, 19.2) gives a velocity
  that reads 0 +- 0.004 rad/s at rest, against +-0.15 rad/s of jitter in the
  driver's estimate at any VMAX. The window is a time, so the estimate is
  marked invalid (loop open, 0 N.m) whenever feedback is missing from it;
  `dm safevelmode` refuses rates below 200 Hz for the same reason.

Measured result at 0.5 rad/s, unloaded: breakaway in 0.08 s with the
integrator at 0.008 N.m, overshoot about +10% on a 50 ms average, settled in
0.4 s, steady error under 0.01 rad/s. The remaining +-0.1 rad/s raw ripple is
real shaft motion: cogging at about 40 Hz. Below about 0.3 rad/s the unloaded
motor stick-slips; a load damps that.

```
dm velgains 0.3 2.0        # kp (N.m per rad/s), ki (N.m per rad)
dm velff 0.14 0.34 0.15    # kinetic ff, breakaway kick, integrator clamp (N.m)
dm velfilt 3 20 0.2 0.05   # velocity filter ms, ms stuck before kick, ff taper rad/s, stuck threshold rad/s
dm velsrc pos 10           # velocity from rotor position over 10 ms ('fb' = driver estimate)
dm veldeadband 0.01        # velocity error ignored
```

#### Measuring a step response

```sh
uv run tools/vel_step.py 1 --vel 0.5                 # step motor 1 to 0.5 rad/s, print metrics
uv run tools/vel_step.py 1 --vel 0.5 --ff 0.1 --brk 0 --csv step.csv
uv run tools/vel_step.py 1 --vel -0.5 --velsrc fb     # compare with the driver's velocity
```

The firmware captures the loop to RAM at full rate (`dm trace <n>`, then
`dm trace dump` for CSV: time, ramped command, velocity used, driver velocity,
torque, integrator, state, rotor position), so the measurement never competes
with the loop for time. The script reports breakaway time and torque, overshoot
and settling on a 50 ms average, steady-state error and the driver's own
velocity noise for comparison.

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

- The SAM E5x CAN controller reaches its message RAM through 16-bit offsets, so
  the driver's message-RAM buffer must stay in the first 64 KiB of SRAM. Big
  application buffers go in the `.dm_trace` section (`app/linker/dm_trace.ld`),
  which is placed after it; a large ordinary static array silently kills CAN
  (frames never leave the controller, error counters stay at 0).
- Both motors have TIMEOUT (register 9) saved as 2000. The unit is 50 us
  (measured: 200 tripped between 10 and 20 ms of frame spacing), so 2000 is
  100 ms: the driver itself disables a motor whose command frames stop, and it
  tolerates streams down to about 20 Hz. The trip is a latched fault (state
  0xD, "comm lost") that the mode commands clear before enabling. With the
  factory value 0 a motor holds its last torque for ever.

- Bus: 1 Mbit/s CAN 2.0. Driver register CAN_BR (RID 35) > 4 switches it to CAN FD
  with BRS; then `dm bus normal fd` and match `bitrate-data` in the board dts.
- Modes (RID 10): 1 MIT (ID 0x000+ESC_ID), 2 position-velocity (0x100+), 3 velocity
  (0x200+), 4 force-position (0x300+). Feedback and register replies arrive on MST_ID.
- Feedback position is the rotor (19.2x the output); velocity is output rad/s with
  0.1 rad/s resolution at VMAX 200. Lower VMAX (RID 22) for finer readout.
- The driver drops to comm-loss if commands stop; the shell streams setpoints for you.
- If the motor stops responding, check the feedback `state`: anything but 1 means the
  driver has latched out and is ignoring commands, however healthy the stream looks.
  `dm clearerr` recovers it. Zero the setpoint first (`dm stop`) — a loop that has been
  commanding into a dead driver is wound up, and re-enabling hands the motor that
  torque all at once.
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
