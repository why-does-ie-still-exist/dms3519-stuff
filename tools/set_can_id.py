#!/usr/bin/env python3
"""Give the one DaMiao motor on the bus a new CAN ID.

Talks to the CANBed M4 firmware's shell over USB serial and:

  1. runs `dm scan`, which probes every ESC_ID (0x000..0x7FE) with a 0x7FF
     register read and lists each motor that answers, with its ESC_ID and
     the MST_ID its replies arrived on;
  2. refuses to continue unless exactly one motor answered.  Two motors that
     share both IDs answer the probe with identical frames, which the bus
     merges into one, so the single survivor is then asked for its shaft
     position (register 80) several times: two motors answer that with
     different data at the same instant, which corrupts the frames and makes
     the CAN error counters climb.  Rising counters or failed reads mean
     duplicates.  (Register 15 "SN" is the same on every DM3519, so it is no
     help here.)
  3. writes MST_ID (register 7) and ESC_ID (register 8), re-addresses the
     motor under the new IDs, reads both registers back, saves to flash
     (motor disabled) and re-scans to confirm exactly one motor at the new IDs.

    tools/set_can_id.py 3                 # ESC_ID 3, MST_ID 0x13 (ESC + 0x10)
    tools/set_can_id.py 3 --mst 0x23      # choose the feedback ID yourself
    tools/set_can_id.py --scan-only       # just list what is on the bus
    tools/set_can_id.py 3 -y              # no confirmation prompt

Exit codes: 0 done, 1 more than one motor (or duplicates suspected),
2 no motor found, 3 the change could not be verified, 4 serial/shell trouble.
"""
import argparse
import glob
import re
import sys
import time

import serial

PROMPT = "uart:~$ "
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def die(msg, code):
    print(msg, file=sys.stderr)
    sys.exit(code)


class Shell:
    def __init__(self, port, verbose=False):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.ser.dtr = True
        self.verbose = verbose
        # A freshly flashed board waits for DTR, then prints its boot banner and
        # self-test; let all of that drain before talking to the shell.
        self._drain(quiet=0.7, limit=20.0)
        for _ in range(4):
            self.ser.write(b"\r")
            if PROMPT.strip() in ANSI.sub("", self._drain(quiet=0.3, limit=3.0)):
                break
        else:
            die(f"no shell prompt on {port}", 4)

    def _drain(self, quiet, limit):
        """Read until nothing arrives for `quiet` seconds (or `limit` in total)."""
        buf = b""
        start = last = time.time()
        while time.time() - last < quiet and time.time() - start < limit:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                last = time.time()
        return buf.decode("utf-8", "replace")

    def run(self, cmd, timeout=3.0):
        """Send one shell line; return its output lines (echo and prompt removed)."""
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r").encode())
        # The reply is complete when a prompt follows the echoed command and
        # nothing more arrives for a moment: a log line printed mid-command
        # makes the shell redraw its prompt early, so a prompt alone is not
        # proof that the command has finished.
        buf = ""
        deadline = time.time() + timeout
        prompt_at = None
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk.decode("utf-8", "replace")
                prompt_at = None
                clean = ANSI.sub("", buf)
                echo = clean.find(cmd)
                if echo >= 0 and clean[echo + len(cmd):].rstrip(" ").endswith(PROMPT.rstrip(" ")):
                    prompt_at = time.time()
            elif prompt_at is not None and time.time() - prompt_at > 0.15:
                break
        out = ANSI.sub("", buf)
        lines = []
        for raw in out.replace("\r", "\n").split("\n"):
            line = raw.strip()
            if not line or line.endswith(PROMPT.strip()) or line == cmd:
                continue
            if line.startswith(PROMPT.strip()):
                line = line[len(PROMPT.strip()):].strip()
                if not line or line == cmd:
                    continue
            lines.append(line)
        if self.verbose:
            print(f"    > {cmd}")
            for line in lines:
                print(f"      {line}")
        return lines


def find_port():
    for pattern in ("/dev/serial/by-id/*CANBed*", "/dev/ttyACM*", "/dev/cu.usbmodem*"):
        ports = sorted(glob.glob(pattern))
        if ports:
            return ports[0]
    return None


def quiesce(sh):
    """Stop anything a previous shell session left running.

    `dm scan` refuses to run while the streamer is active, and a motor whose
    TIMEOUT register is 0 holds its last command forever, so zero the setpoint
    before stopping the stream and disable the motor.  Each command just
    complains if no motor is initialised; that is fine.
    """
    for cmd in ("dm stop", "dm stream 0", "dm disable"):
        sh.run(cmd)


def scan(sh):
    quiesce(sh)
    lines = sh.run("dm scan", timeout=20.0)
    for line in lines:
        if line.startswith("scan: stop the streamer"):
            die("the board's streamer is still running and could not be stopped", 4)
    found = []
    warnings = []
    for line in lines:
        m = re.match(r"scan: found esc 0x([0-9a-f]+) mst 0x([0-9a-f]+)", line, re.I)
        if m:
            found.append((int(m.group(1), 16), int(m.group(2), 16)))
        elif line.startswith("scan:") and ("errors rose" in line or "aborted" in line
                                            or "truncated" in line or "failed" in line):
            warnings.append(line)
    if not any(line.startswith("scan:") for line in lines):
        die("the firmware did not answer `dm scan`; flash a build that has it", 4)
    return found, warnings


def read_reg(sh, rid):
    for line in sh.run(f"dm read {rid}"):
        m = re.match(rf"rid {rid} = (\d+) \(0x([0-9a-f]+)\)", line, re.I)
        if m:
            return int(m.group(1))
    return None


def read_float(sh, rid):
    for line in sh.run(f"dm read {rid}"):
        m = re.match(rf"rid {rid} = (\S+) \(raw 0x([0-9a-f]+)\)", line, re.I)
        if m:
            return int(m.group(2), 16)  # raw bits: exact comparison, no float noise
    return None


def init(sh, esc, mst):
    sh.run(f"dm init {esc:#x} {mst:#x}")


def bus_errors(sh):
    """(rx errors, tx errors) from `can show`; None if the shell lacks it."""
    rx = tx = None
    for line in sh.run("can show can@42000000"):
        m = re.match(r"(rx|tx) errors:\s+(\d+)", line)
        if m:
            if m.group(1) == "rx":
                rx = int(m.group(2))
            else:
                tx = int(m.group(2))
    return None if rx is None or tx is None else (rx, tx)


def ids_match(sh, esc, mst):
    init(sh, esc, mst)
    return read_reg(sh, 8) == esc and read_reg(sh, 7) == mst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("esc", nargs="?", type=lambda s: int(s, 0), help="new ESC_ID (motor CAN ID), 0..0x7FE")
    ap.add_argument("--mst", type=lambda s: int(s, 0), help="new MST_ID (feedback CAN ID); default ESC_ID + 0x10")
    ap.add_argument("--port", help="serial port (default: first CANBed / ttyACM / cu.usbmodem)")
    ap.add_argument("--scan-only", action="store_true", help="list the motors on the bus and exit")
    ap.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")
    ap.add_argument("--no-rescan", action="store_true", help="skip the final confirmation scan")
    ap.add_argument("-v", "--verbose", action="store_true", help="show every shell exchange")
    args = ap.parse_args()

    if not args.scan_only and args.esc is None:
        ap.error("give the new ESC_ID, or --scan-only")
    if args.esc is not None and not 0 <= args.esc <= 0x7FE:
        ap.error("ESC_ID must be 0..0x7FE (0x7FF is the register channel)")
    new_mst = args.mst if args.mst is not None else (args.esc + 0x10 if args.esc is not None else None)
    if new_mst is not None and not 0 <= new_mst <= 0x7FF:
        ap.error("MST_ID must be 0..0x7FF")
    if args.esc is not None and new_mst == args.esc:
        ap.error("MST_ID must differ from ESC_ID, otherwise the motor hears its own feedback")

    port = args.port or find_port()
    if port is None:
        die("no serial port found; plug in the CANBed or pass --port", 4)
    try:
        sh = Shell(port, verbose=args.verbose)
    except serial.SerialException as e:
        die(f"cannot open {port}: {e}", 4)
    print(f"shell on {port}")

    # 1. who is on the bus?
    print("scanning ESC_IDs 0x000..0x7FE (a few seconds) ...")
    found, warnings = scan(sh)
    for esc, mst in found:
        print(f"  motor: esc 0x{esc:03x} mst 0x{mst:03x}")
    for w in warnings:
        print(f"  {w}")
    if args.scan_only:
        return 0 if len(found) == 1 else (2 if not found else 1)
    if not found:
        die("no motor answered: check power, CAN wiring/termination and bitrate", 2)
    if len(found) > 1:
        die(f"{len(found)} motors on the bus; connect exactly one to set its ID", 1)
    if any("errors rose" in w for w in warnings):
        die("bus errors during the scan: two motors with identical IDs? connect one", 1)

    # 2. identify the single motor, and make sure it really is single.
    old_esc, old_mst = found[0]
    init(sh, old_esc, old_mst)
    errs0 = bus_errors(sh)
    positions = [read_float(sh, 80) for _ in range(6)]
    errs1 = bus_errors(sh)
    model = read_reg(sh, 15)
    sw = read_reg(sh, 14)
    # The position itself is only the stimulus: a shaft that drifts a count
    # between reads is normal, but two motors answering at once corrupt the
    # frames, which shows up as missing replies and rising error counters.
    got = {p for p in positions if p is not None}
    if not got:
        die("motor answers the scan but not register reads; try again", 3)
    dup = []
    if None in positions:
        dup.append(f"{positions.count(None)} of {len(positions)} position reads got no reply")
    if errs0 and errs1 and errs1 > errs0:
        dup.append(f"bus error counters rose rx/tx {errs0} -> {errs1}")
    if dup:
        die("two motors seem to share esc 0x%03x mst 0x%03x: %s. Connect exactly one"
            % (old_esc, old_mst, "; ".join(dup)), 1)
    print(f"one motor: esc 0x{old_esc:03x} mst 0x{old_mst:03x} model {model:#x} sw {sw:#x}")
    if (old_esc, old_mst) == (args.esc, new_mst):
        print("already has these IDs; nothing to do")
        return 0
    print(f"new IDs:   esc 0x{args.esc:03x} mst 0x{new_mst:03x}")
    if not args.yes:
        if input("write and save? [y/N] ").strip().lower() not in ("y", "yes"):
            print("aborted")
            return 0

    # 3. write, re-address, verify, save, verify again.
    sh.run("dm disable")
    # The reply to an ID write may already use the new ID, which the firmware
    # is not listening on yet, so the write can report a timeout even when it
    # worked.  The read-back below is what counts.
    if old_mst != new_mst:
        sh.run(f"dm write 7 {new_mst:#x}")
    if old_esc != args.esc:
        sh.run(f"dm write 8 {args.esc:#x}")
    if not ids_match(sh, args.esc, new_mst):
        # Tell the user where the motor ended up, if we can find it.
        for esc, mst in ((args.esc, old_mst), (old_esc, new_mst), (old_esc, old_mst)):
            if ids_match(sh, esc, mst):
                die(f"ID write did not fully apply; motor still answers at esc 0x{esc:03x} "
                    f"mst 0x{mst:03x} (not saved)", 3)
        die("motor no longer answers at the old or new IDs; power-cycle it and re-scan", 3)
    save = sh.run("dm save")
    if not any(line.startswith("save: 0") for line in save):
        die(f"save to flash failed: {' | '.join(save) or 'no reply'}", 3)
    if read_reg(sh, 8) != args.esc or read_reg(sh, 7) != new_mst:
        die("IDs changed after save; re-scan to find the motor", 3)
    if not args.no_rescan:
        found, _ = scan(sh)
        if found != [(args.esc, new_mst)]:
            die(f"after save the scan shows {found or 'nothing'}, expected "
                f"[(0x{args.esc:03x}, 0x{new_mst:03x})]", 3)
    print(f"done: motor is now esc 0x{args.esc:03x} mst 0x{new_mst:03x}, saved to flash")
    if new_mst == args.esc + 0x10:
        print(f"use:  dm init {args.esc:#x}")
    else:
        print(f"use:  dm init {args.esc:#x} {new_mst:#x}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
