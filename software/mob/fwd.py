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

FWD_SPEED = 250
FWD_ACC = 1000


def wait_done(ser: serial.Serial, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    buf = []

    while time.time() < deadline:
        time.sleep(0.01)  # small delay to allow response
        try:
            raw = ser.readline()
        except serial.SerialException as e:
            # USB device temporary glitch - wait and retry
            print(f"#Serial error: {e}, retrying...")
            time.sleep(0.1)
            try:
                ser.reset_input_buffer()
            except:
                pass
            continue
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

        # Display messages starting with #
        if line.startswith("#"):
            print(line)

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
            # Clear input buffer before sending to avoid stale DONE messages
            try:
                ser.reset_input_buffer()
            except:
                pass
            time.sleep(0.01)  # Small delay to ensure buffer is clear
            ser.write(line.encode("ascii", errors="ignore"))
            print(f"TX: {line.strip()}")

        for _ in range(1):
            send("GCAL\n")
            time.sleep(1)
            send("RDST\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (RDST)")
            
            send("RANG\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (RANG)")
            
            send(f"FWD,{FWD_SPEED},{FWD_ACC},90\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (FWD)")

            send(f"FWD,{FWD_SPEED},{FWD_ACC},180\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (FWD)")

            send(f"STOP,{FWD_SPEED},{FWD_ACC},90\n")
            wait_done(ser, args.timeout)
            print("RX: DONE (STOP)")

        # Request sensor data to display current distance
        send("SEN\n")
        time.sleep(0.05)  # Wait a bit for response
        
        # Read SEN response
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                raw = ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            
            line = raw.decode("ascii", errors="replace").rstrip("\r\n").strip()
            if line.startswith("SEN,"):
                # Parse SEN response: SEN,gyro,vbatt,lf,ls,rs,rf,enc_r,enc_l,odo_dist,odo_ang
                parts = line.split(",")
                if len(parts) >= 10:
                    try:
                        odo_dist = float(parts[9])
                        odo_ang = float(parts[10]) if len(parts) > 10 else 0.0
                        print(f"\n=== Final Position ===")
                        print(f"Distance: {odo_dist:.2f} mm")
                        print(f"Angle: {odo_ang:.4f} rad ({odo_ang * 180 / 3.14159:.2f} deg)")
                    except ValueError:
                        print(f"#Failed to parse SEN data: {line}")
                break

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
