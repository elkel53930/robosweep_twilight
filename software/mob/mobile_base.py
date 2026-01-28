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
from math import pi
import time

import serial  # pyserial

FWD_SPEED = 300
FWD_ACC = 1200
TURN_R = -pi/2  # approx 90 degrees
TURN_L = pi/2  # approx -90 degrees

# LFとRFの基準値
LF_BASE = 200
RF_BASE = 222

class MobileBase:
    """ロボット制御用シリアル通信クラス"""
    
    def __init__(self, ser: serial.Serial, timeout: float = 5.0):
        self.ser = ser
        self.timeout = timeout
        self.turn_dir = 1  # -1:右回り、1:左回り
    
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
    
    def cmd_gcal(self) -> None:
        """ジャイロキャリブレーション"""
        self._send("GCAL\n")
        time.sleep(1)  # キャリブレーション待機
    
    def cmd_rdst(self) -> None:
        """距離リセット"""
        self._send("RDST\n")
        self._wait_done()
        print("RX: DONE (RDST)")
    
    def cmd_rang(self) -> None:
        """角度リセット"""
        self._send("RANG\n")
        self._wait_done()
        print("RX: DONE (RANG)")
    
    def cmd_fwd(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """前進コマンド"""
        self._send(f"FWD,{speed_mmps},{accel_mmps2},{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (FWD)")
    
    def cmd_stop(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """停止コマンド"""
        self._send(f"STOP,{speed_mmps},{accel_mmps2},{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (STOP)")
    
    def cmd_turn(self, angle_rad: float) -> None:
        """旋回コマンド"""
        self._send(f"TURN,{angle_rad}\n")
        self._wait_done()
        print("RX: DONE (TURN)")

    def cmd_lfwd(self) -> None:
        """低速前進（Lowspeed）"""
        self._send("LFWD\n")
        print("RX: DONE (LFWD)")

    def cmd_lback(self) -> None:
        """低速後退（Lowspeed）"""
        self._send("LBACK\n")
        print("RX: DONE (LBACK)")

    def cmd_lturnl(self) -> None:
        """低速左旋回（Lowspeed）"""
        self._send("LTURNL\n")
        print("RX: DONE (LTURNL)")

    def cmd_lturnr(self) -> None:
        """低速右旋回（Lowspeed）"""
        self._send("LTURNR\n")
        print("RX: DONE (LTURNR)")

    def cmd_lstop(self) -> None:
        """低速動作停止（Lowspeed）"""
        self._send("LSTOP\n")
        print("RX: DONE (LSTOP)")
    
    def cmd_jogfwd(self, distance_mm: float) -> None:
        """低速で指定距離前進（JOG）"""
        self._send(f"JOGFWD,{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (JOGFWD)")
    
    def cmd_jogback(self, distance_mm: float) -> None:
        """低速で指定距離後退（JOG）"""
        self._send(f"JOGBACK,{distance_mm}\n")
        self._wait_done()
        print("RX: DONE (JOGBACK)")
    
    def cmd_wall(self, enable: bool) -> None:
        """壁制御ON/OFF"""
        val = 1 if enable else 0
        self._send(f"WALL,{val}\n")
        # WALLはDONE応答がない
        print(f"RX: DONE (WALL,{val})")
    
    def cmd_sen(self) -> dict | None:
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
    
    def _front_wall_correction(self) -> None:
        sen = self.cmd_sen()
        if not sen:
            return # センサーデータ取得失敗
        lf = sen['lf']
        rf = sen['rf']
        print("#Front wall sensors: lf =", lf, "rf =", rf)
        if sen['lf'] < 100 or sen['rf'] < 100:
            return #壁がない
        
        print("#Starting front wall correction...")

        # 前後補正

        lf_rate = lf / LF_BASE
        rf_rate = rf / RF_BASE

        if (lf_rate+rf_rate) / 2 > 1.0:
            print("#Too close to front wall: backing up")
            # 前方近すぎ：後退
            self.cmd_jogback(10)
        elif (lf_rate+rf_rate) / 2 < 0.9:
            print("#Too far from front wall: moving forward")
            # 前方遠すぎ：前進
            self.cmd_jogfwd(10)
        
        time.sleep(0.1)

        # 旋回補正
        if lf_rate > rf_rate * 1.03:
            # 左壁が近すぎ：右旋回
            print("#Correcting orientation: turning right")
            self.cmd_turn(-0.1)
        elif rf_rate > lf_rate * 1.03:
            # 右壁が近すぎ：左旋回
            print("#Correcting orientation: turning left")
            self.cmd_turn(0.1)

        time.sleep(0.1)
    
    def start(self) -> None:
        """マス中央から走り出す"""
        self.cmd_fwd(FWD_SPEED, FWD_ACC, 90)
    
    def go_right(self) -> None:
        """右90度旋回"""
        self.cmd_stop(FWD_SPEED, FWD_ACC, 90)
#        self._front_wall_correction()
        self.cmd_turn(TURN_R)
        self.start()
    
    def go_left(self) -> None:
        """左90度旋回"""
        self.cmd_stop(FWD_SPEED, FWD_ACC, 90)
#        self._front_wall_correction()
        self.cmd_turn(TURN_L)
        self.start()
    
    def stop(self) -> None:
        """停止して終了"""
        self.cmd_stop(FWD_SPEED, FWD_ACC, 90)

    def go_fwd(self) -> None:
        """前進"""
        self.cmd_fwd(FWD_SPEED, FWD_ACC, 180)
    
    def go_back(self) -> None:
        """戻る"""
        self.cmd_stop(FWD_SPEED, FWD_ACC, 90)
#        self._front_wall_correction()
        self.cmd_turn(self.turn_dir * pi)
        self.turn_dir *= -1  # 方向反転
        self.start()
    
    def turn_left_90(self) -> None:
        """左90度旋回"""
        self.cmd_turn(TURN_L)
    
    def turn_right_90(self) -> None:
        """右90度旋回"""
        self.cmd_turn(TURN_R)

    def turn_180(self) -> None:
        """180度旋回"""
        self.cmd_turn(self.turn_dir * pi)
        self.turn_dir *= -1  # 方向反転


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

        mob = MobileBase(ser, timeout=args.timeout)

        mob.cmd_gcal()
        mob.cmd_rdst()
        mob.cmd_rang()
        mob.cmd_wall(True)  # 壁センサオン

        for _ in range(5):
            mob.start()
            mob.go_right()
            mob.go_right()
            mob.go_back()
            mob.go_left()
            mob.go_right()
            mob.go_right()
            mob.go_left()
            mob.go_left()
            mob.go_back()

            mob.go_right()
            mob.go_right()
            mob.go_left()
            mob.go_fwd()
            mob.stop()
            mob.turn_180()

        mob.cmd_wall(False)  # 壁センサオフ
        # Request sensor data to display current distance
        sensor_data = mob.cmd_sen()
        if sensor_data:
            print(f"\n=== Final Position ===")
            print(f"Distance: {sensor_data['odo_dist']:.2f} mm")
            print(f"Angle: {sensor_data['odo_ang']:.4f} rad ({sensor_data['odo_ang'] * 180 / 3.14159:.2f} deg)")

        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
