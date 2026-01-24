#!/usr/bin/env python3
"""Send FWD then STOP to ESP32 and wait for DONE each time.

This script sends:
  1) FWD,400,1000,90\n then waits for a line exactly equal to "DONE"
  2) STOP,400,1000,90\n then waits for a line exactly equal to "DONE"

Notes:
- Expects the firmware to output DONE as a standalone line ("DONE\n").
- Ignores other incoming lines (SEN stream, # messages, etc.).

Usage:
  python3 fwd.py --port /dev/ttyUSB0 --baud 3000000
"""

from __future__ import annotations

import argparse
import sys
import time

import serial  # pyserial


def wait_done(ser: serial.Serial, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    buf = []

    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue

        # Decode line safely
        line = raw.decode("ascii", errors="replace").rstrip("\r\n")

        # strip embedded NULs / control chars just in case
        if "\x00" in line:
            line = line.replace("\x00", "")
        line = "".join(ch for ch in line if (ch >= " " or ch == "\t"))

        line = line.strip()
        if not line:
            continue

        # Ignore sensor spam
        if line.startswith("SEN,"):
            continue

        # Accept DONE
        if line == "DONE":
            return

        # Keep recent non-SEN lines to help diagnose timeouts
        buf.append(line)

    tail = "\n".join(buf[-50:])
    raise TimeoutError(f"Timed out waiting for DONE (last lines):\n{tail}\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=3000000, help="Baudrate (default: 3000000)")
    ap.add_argument("--timeout", type=float, default=5.0, help="DONE wait timeout seconds (default: 5.0)")
    args = ap.parse_args()

    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        def send(line: str) -> None:
            # Give the device a moment and clear any queued output so we wait for DONE *after* this command
            ser.write(line.encode("ascii", errors="ignore"))
            print(f"TX: {line.strip()}")

        for _ in range(4):
            send("RDST\n")
            send("RANG\n")
            send("FWD,400,2000,90\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (FWD)")

            send("STOP,400,2000,90\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (STOP)")

            # send("RANG\n")
            send("TURN,-1.53\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (TURN)")

            time.sleep(0.1)

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
