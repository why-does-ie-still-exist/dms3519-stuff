#!/usr/bin/env python3
"""Capture the board's USB CDC console.

Waits for a /dev/cu.usbmodem* port to appear, opens it (asserting DTR so the
firmware knows the host is listening), and echoes everything to stdout and
an optional log file for a fixed duration.

    tools/console.py [--seconds N] [--log FILE] [--send "text"]
"""
import argparse, glob, sys, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=60)
ap.add_argument("--wait", type=float, default=3600, help="seconds to wait for the port")
ap.add_argument("--log")
ap.add_argument("--send", action="append", default=[], help="line to send after open (repeatable)")
args = ap.parse_args()

deadline = time.time() + args.wait
port = None
while time.time() < deadline:
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if ports:
        port = ports[0]
        break
    time.sleep(0.5)
if port is None:
    sys.exit("no /dev/cu.usbmodem* port appeared")

time.sleep(0.5)
ser = serial.Serial(port, 115200, timeout=0.2)
ser.dtr = True
print(f"--- opened {port} ---", flush=True)
log = open(args.log, "ab") if args.log else None
end = time.time() + args.seconds
sent = False
try:
    while time.time() < end:
        data = ser.read(4096)
        if data:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
            if log:
                log.write(data); log.flush()
        if not sent and time.time() > end - args.seconds + 8:
            for line in args.send:
                ser.write((line + "\r\n").encode())
                time.sleep(0.5)
            sent = True
except serial.SerialException as e:
    print(f"--- port closed: {e} ---")
print("--- done ---", flush=True)
