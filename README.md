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

### Safe velocity control (torque-capped)

Velocity control that cannot pull more than a set torque. Unlike `dm speedcap`
in torque mode, hitting the cap is **not** a trip: a stalled motor pushes at
exactly the cap and holds there, and goes back to tracking the setpoint by
itself once the load is removed.

```
dm init 1 0
dm torquecap 1.5   # N.m at the output shaft (default 2.0); 0 means no torque
dm velgains 0.4 1.2  # optional: kp (N.m per rad/s), ki (N.m per rad)
dm safevelmode     # MIT mode, MCU-side velocity loop, 100 Hz; starts idle
dm vel 0.5         # rad/s, ramped by 'dm stream <hz> <ramp>'
dm status          # shows the torque being used and "[AT CAP]" when saturated
dm stop            # idle: 0 N.m until the next 'dm vel'
dm disable
```

`dm stop` idles the loop at zero torque rather than regulating to 0 rad/s. Those
are different things: a zero-speed setpoint leaves the loop fighting feedback
quantisation (below), which shows up as jitter at standstill. `dm vel 0` still
asks for an actively braked zero if that is what you want.

The velocity loop runs on the MCU rather than in the driver, because the
driver's own velocity mode (`dm velmode`, CTRL_MODE 3) will spend up to TMAX to
hold a setpoint. Here a PI controller on velocity error produces a torque that
is clamped to the cap and sent as a MIT frame with Kp = Kd = 0. Integration is
suspended while clamped, so a long stall cannot wind the integrator up and
lurch when the load clears.

Feedback cannot express zero velocity. The 12-bit field maps +-VMAX onto
0..4095, so zero falls at code 2047.5 and a stationary motor reports
+-VMAX/4095 (0.049 rad/s at VMAX 200) for ever. Left alone that constant
apparent error ratchets the integrator up until it breaks stiction and the
motor jitters, so error within `dm veldeadband` (default 1.5 counts, 0.147
rad/s at VMAX 200) is ignored by both terms:

```
dm veldeadband       # show, in rad/s and in feedback counts
dm veldeadband 0.05  # override; warns below one count
```

The cost is up to one deadband of steady-state speed error. Lowering VMAX
(RID 22) shrinks the quantisation step and the deadband together and is the
real fix if you care about slow speeds; it also rescales the MIT velocity
field, so re-run `dm init` afterwards.

Tuning notes: the default gains are deliberately soft and assume the output
shaft, so expect to raise `kp` for stiffer tracking. If feedback stops for
100 ms the loop opens and commands 0 N.m.

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
