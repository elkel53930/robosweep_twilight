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

FWD_SPEED = 350
FWD_ACC = 1500
TURN_R = -1.53  # approx 90 degrees
TURN_L = 1.53  # approx -90 degrees

class RobotController:
    """ロボット制御用シリアル通信クラス"""
    
    def __init__(self, ser: serial.Serial, timeout: float = 5.0):
        self.ser = ser
        self.timeout = timeout
    
    def _send(self, line: str) -> None:
        """コマンドを送信"""
        # Clear input buffer before sending to avoid stale DONE messages
        try:
            self.ser.reset_input_buffer()
        except:
            pass
        time.sleep(0.01)  # Small delay to ensure buffer is clear
        self.ser.write(line.encode("ascii", errors="ignore"))
        print(f"TX: {line.strip()}")
    
    def _wait_done(self) -> None:
        """DONE応答を待機"""
        deadline = time.time() + self.timeout
        buf = []

        while time.time() < deadline:
            time.sleep(0.01)  # small delay to allow response
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                # USB device temporary glitch - wait and retry
                print(f"#Serial error: {e}, retrying...")
                time.sleep(0.1)
                try:
                    self.ser.reset_input_buffer()
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
    
    def gcal(self) -> None:
        """ジャイロキャリブレーション"""
        self._send("GCAL\n")
        time.sleep(1)  # キャリブレーション待機
    
    def rdst(self) -> None:
        """距離リセット"""
        self._send("RDST\n")
        self._wait_done()
        print("RX: DONE (RDST)")
    
    def rang(self) -> None:
        """角度リセット"""
        self._send("RANG\n")
        self._wait_done()
        print("RX: DONE (RANG)")
    
    def fwd(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """前進コマンド"""
        self._send(f"FWD,{speed_mmps},{accel_mmps2},{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (FWD)")
    
    def stop(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """停止コマンド"""
        self._send(f"STOP,{speed_mmps},{accel_mmps2},{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (STOP)")
    
    def turn(self, angle_rad: float) -> None:
        """旋回コマンド"""
        self._send(f"TURN,{angle_rad}\n")
        self._wait_done()
        print("RX: DONE (TURN)")
    
    def sen(self) -> dict | None:
        """センサーデータ取得"""
        self._send("SEN\n")
        time.sleep(0.05)  # Wait a bit for response
        
        # Read SEN response
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                raw = self.ser.readline()
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
                        return {
                            'gyro': float(parts[1]),
                            'vbatt': float(parts[2]),
                            'lf': int(parts[3]),
                            'ls': int(parts[4]),
                            'rs': int(parts[5]),
                            'rf': int(parts[6]),
                            'enc_r': int(parts[7]),
                            'enc_l': int(parts[8]),
                            'odo_dist': float(parts[9]),
                            'odo_ang': float(parts[10]) if len(parts) > 10 else 0.0,
                        }
                    except (ValueError, IndexError) as e:
                        print(f"#Failed to parse SEN data: {line} ({e})")
                        break
        return None


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

        robot = RobotController(ser, timeout=args.timeout)

        for _ in range(1):
            robot.gcal()
            robot.rdst()
            robot.rang()
            
            robot.fwd(FWD_SPEED, FWD_ACC, 90)
            robot.fwd(FWD_SPEED, FWD_ACC, 180)
            robot.stop(FWD_SPEED, FWD_ACC, 90)
            
            robot.turn(TURN_R)
            
            robot.fwd(FWD_SPEED, FWD_ACC, 90)
            robot.stop(FWD_SPEED, FWD_ACC, 90)

            robot.turn(TURN_L)

            robot.fwd(FWD_SPEED, FWD_ACC, 90)
            robot.stop(FWD_SPEED, FWD_ACC, 90)

            robot.turn(TURN_L)

            robot.fwd(FWD_SPEED, FWD_ACC, 90)
            robot.stop(FWD_SPEED, FWD_ACC, 90)

            robot.turn(TURN_R*2)
        # Request sensor data to display current distance
        sensor_data = robot.sen()
        if sensor_data:
            print(f"\n=== Final Position ===")
            print(f"Distance: {sensor_data['odo_dist']:.2f} mm")
            print(f"Angle: {sensor_data['odo_ang']:.4f} rad ({sensor_data['odo_ang'] * 180 / 3.14159:.2f} deg)")

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
