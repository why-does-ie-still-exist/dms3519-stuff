#!/usr/bin/env python3
"""Step the safe-velocity loop and measure how it responds.

Drives the board's shell: selects the motor, enters safevelmode at the given
rate, arms a RAM trace, commands a velocity step, waits, stops, then pulls the
trace back as CSV and reports breakaway, overshoot and settling.

    uv run tools/vel_step.py 1 --vel 0.5                    # motor 1, step to 0.5 rad/s
    uv run tools/vel_step.py 1 --vel 0.5 --ff 0.1 --imax 0.3  # with friction feed-forward
    uv run tools/vel_step.py 1 --vel 0.5 --kp 0.4 --ki 1.5 --ramp 0
    uv run tools/vel_step.py 1 --vel 0.5 --csv step.csv     # keep the samples

The loop's 'cmd' column is the ramped setpoint actually being tracked, so
overshoot is measured against the final setpoint once the ramp has arrived.
"""
import argparse
import csv
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from set_can_id import Shell, die, find_port, quiesce  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("esc", type=lambda s: int(s, 0), help="motor ESC_ID")
    ap.add_argument("--mst", type=lambda s: int(s, 0), help="MST_ID if not ESC_ID + 0x10")
    ap.add_argument("--vel", type=float, default=0.5, help="step target, rad/s (default 0.5)")
    ap.add_argument("--hz", type=int, default=1000, help="loop rate (default 1000)")
    ap.add_argument("--samples", type=int, default=2000, help="trace length in ticks (max 2500)")
    ap.add_argument("--ramp", type=float, help="setpoint ramp rad/s^2 (0 = step); default: board's")
    ap.add_argument("--kp", type=float)
    ap.add_argument("--ki", type=float)
    ap.add_argument("--ff", type=float, help="kinetic friction feed-forward, N.m")
    ap.add_argument("--brk", type=float, help="breakaway kick, N.m (0 = off)")
    ap.add_argument("--imax", type=float, help="integrator clamp, N.m")
    ap.add_argument("--tau", type=float, help="velocity filter, ms (0 = off)")
    ap.add_argument("--stick", type=float, help="ms at rest before the kick")
    ap.add_argument("--taper", type=float, help="feed-forward taper, rad/s")
    ap.add_argument("--db", type=float, help="velocity deadband, rad/s")
    ap.add_argument("--velsrc", choices=["pos", "fb"], help="loop velocity: rotor position difference, or the driver's estimate")
    ap.add_argument("--win", type=int, help="position-difference window, ticks")
    ap.add_argument("--cap", type=float, help="torque cap, N.m")
    ap.add_argument("--csv", help="write the samples here")
    ap.add_argument("--port")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    port = a.port or find_port()
    if port is None:
        die("no serial port found", 4)
    sh = Shell(port, verbose=a.verbose)
    quiesce(sh)

    init = f"dm init {a.esc:#x}" + (f" {a.mst:#x}" if a.mst is not None else "")
    out = sh.run(init)
    if not any("limits read" in l or "selected motor" in l for l in out):
        die(f"motor did not answer: {' | '.join(out)}", 2)
    if a.cap is not None:
        sh.run(f"dm torquecap {a.cap}")
    if a.kp is not None or a.ki is not None:
        cur = sh.run("dm velgains")
        m = re.search(r"kp ([\d.]+) N.m per rad/s, ki ([\d.]+)", " ".join(cur))
        kp = a.kp if a.kp is not None else float(m.group(1))
        ki = a.ki if a.ki is not None else float(m.group(2))
        sh.run(f"dm velgains {kp} {ki}")
    if a.ff is not None or a.brk is not None or a.imax is not None:
        cur = " ".join(sh.run("dm velff"))
        m = re.search(r"kinetic ([\d.]+) N.m, breakaway kick ([\d.]+) N.m, integrator clamp ([\d.]+)", cur)
        ff = a.ff if a.ff is not None else float(m.group(1))
        brk = a.brk if a.brk is not None else float(m.group(2))
        imax = a.imax if a.imax is not None else float(m.group(3))
        sh.run(f"dm velff {ff} {brk} {imax}")
    if a.tau is not None or a.stick is not None or a.taper is not None:
        cur = " ".join(sh.run("dm velfilt"))
        m = re.search(r"low-pass ([\d.]+) ms, kick after ([\d.]+) ms stuck, feed-forward tapered over ([\d.]+)", cur)
        tau = a.tau if a.tau is not None else float(m.group(1))
        stick = a.stick if a.stick is not None else float(m.group(2))
        taper = a.taper if a.taper is not None else float(m.group(3))
        sh.run(f"dm velfilt {tau} {stick} {taper}")
    if a.velsrc is not None or a.win is not None:
        sh.run("dm velsrc " + (a.velsrc or "") + (f" {a.win}" if a.win is not None else ""))
    if a.db is not None:
        sh.run(f"dm veldeadband {a.db}")
    if a.ramp is not None:
        sh.run(f"dm stream 0 {a.ramp}")  # sets the ramp; the mode command restarts the stream
    mode = sh.run(f"dm safevelmode {a.hz}", timeout=3.0)
    for line in mode:
        if "streaming at" in line or "torque cap" in line or "friction ff" in line:
            print(line)
    if not any("streaming at" in l for l in mode):
        die(f"safevelmode failed: {' | '.join(mode)}", 3)
    period = 1.0 / a.hz

    sh.run(f"dm trace {a.samples}")
    t_step = time.time()
    sh.run(f"dm vel {a.vel}")
    time.sleep(a.samples * period + 0.3)
    sh.run("dm stop")
    time.sleep(0.3)
    sh.run("dm disable")
    sh.run("dm stream 0")

    lines = sh.run("dm trace dump", timeout=60.0)
    rows = []
    for line in lines:
        m = re.match(r"(\d+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(\d+),(-?[\d.]+),(\d+)$", line)
        if m:
            rows.append((int(m.group(1)) / 1e6, float(m.group(2)), float(m.group(3)),
                         float(m.group(5)), float(m.group(6)), int(m.group(7)) & 0x7F,
                         bool(int(m.group(7)) & 0x80), float(m.group(4)), float(m.group(8)),
                         int(m.group(9))))
    if len(rows) < 10:
        die(f"trace came back with {len(rows)} samples: {' | '.join(lines[:3])}", 3)
    t0 = rows[0][0]
    rows = [(t - t0,) + tuple(r[1:]) for (t, *_) , r in zip(rows, rows)]

    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "cmd", "meas", "tor", "integ", "state", "kick", "fbv", "pos", "fb_cyc"])
            w.writerows(rows)

    # --- metrics -----------------------------------------------------------
    target = a.vel
    sgn = 1.0 if target >= 0 else -1.0
    v = [r[2] * sgn for r in rows]
    tq = [r[3] * sgn for r in rows]
    integ = [r[4] * sgn for r in rows]
    t = [r[0] for r in rows]
    cmd = [r[1] * sgn for r in rows]
    tgt = abs(target)
    dts = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    dt_mean = sum(dts) / len(dts)
    dt_max = max(dts)

    vmax = 200.0
    m = re.search(r"rid 22 = ([\d.]+)", " ".join(sh.run("dm read 22")))
    if m:
        vmax = float(m.group(1))
    lsb = 2 * vmax / 4095  # feedback resolution at this VMAX
    # breakaway: first sample where the motor is clearly moving the right way
    # The shaft has a real ~40 Hz cogging ripple, so judge the transient on a
    # 50 ms moving average; breakaway on the raw signal.
    def moving_avg(x, n=50):
        out, acc = [], 0.0
        for i, val in enumerate(x):
            acc += val
            if i >= n:
                acc -= x[i - n]
            out.append(acc / min(i + 1, n))
        return out
    va = moving_avg(v)
    k_break = next((i for i, x in enumerate(v) if x > 0.1), None)
    k_ramp_done = next((i for i, c in enumerate(cmd) if c >= tgt - 1e-6), None)
    k_peak = max(range(len(va)), key=lambda i: va[i])
    k_90 = next((i for i, x in enumerate(va) if x >= 0.9 * tgt), None)
    band = 0.10 * tgt
    k_settle = None
    for i in range(len(va)):
        if all(abs(x - tgt) <= band for x in va[i:]):
            k_settle = i
            break
    n_tail = max(10, len(v) // 4)
    tail_v = v[-n_tail:]
    tail_tq = tq[-n_tail:]
    mean_v = sum(tail_v) / n_tail
    sd_v = math.sqrt(sum((x - mean_v) ** 2 for x in tail_v) / n_tail)
    mean_tq = sum(tail_tq) / n_tail

    print()
    print(f"loop: {len(rows)} samples, dt mean {dt_mean*1e3:.3f} ms, max {dt_max*1e3:.3f} ms "
          f"(asked {period*1e3:.1f} ms)")
    print(f"step: 0 -> {target:+.3f} rad/s" + (f", ramp arrives at {t[k_ramp_done]:.3f} s" if k_ramp_done is not None else ", ramp never reached target"))
    if k_break is None:
        print("motor never moved (|vel| stayed under 2 counts)")
    else:
        print(f"breakaway: t {t[k_break]:.3f} s, torque {tq[k_break]:.3f} N.m "
              f"(integrator {integ[k_break]:.3f}, feed-forward+P {tq[k_break]-integ[k_break]:.3f}), "
              f"cmd then {cmd[k_break]:.3f} rad/s")
    if k_90 is not None:
        print(f"90% of target at t {t[k_90]:.3f} s")
    print(f"peak (50 ms avg): {va[k_peak]:.3f} rad/s at t {t[k_peak]:.3f} s = overshoot "
          f"{100*(va[k_peak]-tgt)/tgt:+.1f}% (integrator at peak {integ[k_peak]:.3f} N.m); "
          f"raw max {max(v):.3f}")
    print(f"settled within +-10%: " + (f"t {t[k_settle]:.3f} s" if k_settle is not None else "never"))
    tail_a = va[-n_tail:]
    sd_a = math.sqrt(sum((x - mean_v) ** 2 for x in tail_a) / n_tail)
    print(f"steady (last {n_tail} samples): vel {mean_v:.3f} +- {sd_v:.3f} rad/s raw (+- {sd_a:.3f} on 50 ms avg), "
          f"torque {mean_tq:.3f} N.m, error {mean_v - tgt:+.3f} rad/s")
    fbv = [r[7] * sgn for r in rows][-n_tail:]
    mfb = sum(fbv) / n_tail
    sdfb = math.sqrt(sum((x - mfb) ** 2 for x in fbv) / n_tail)
    print(f"driver's own velocity over the same tail: {mfb:.3f} +- {sdfb:.3f} rad/s "
          f"(its resolution {lsb:.4f} rad/s at VMAX {vmax:g})")
    cyc = [r[9] for r in rows]
    updates = sum(1 for i in range(1, len(cyc)) if cyc[i] != cyc[i - 1])
    print(f"feedback frames with new content: {updates} in {len(cyc)} ticks "
          f"({100.0 * updates / max(1, len(cyc) - 1):.0f}% of loop ticks saw fresh feedback)")
    kicks = [i for i, r in enumerate(rows) if r[6]]
    if kicks:
        print(f"breakaway kick: {len(kicks)} ticks, first at t {t[kicks[0]]:.3f} s, last at t {t[kicks[-1]]:.3f} s")
    states = sorted({r[5] for r in rows})
    if states != [1]:
        print(f"driver states seen: {states} (1 = enabled)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
