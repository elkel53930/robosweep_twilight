#!/usr/bin/env python3
"""ESP32 serial TUI (keyboard-only) for robosweep_twilight.

Protocol
- PC -> ESP32 commands: CSV line ending with '\n', e.g. "MOT,100,100\n"
- ESP32 -> PC sensor stream: "SEN,<...>\n"
- ESP32 -> PC messages: "#<message>\n"

Usage:
  python3 pc_comm.py --port /dev/ttyUSB0 --baud 3000000

Keys:
  q           Quit
  Arrow keys  Change motor target (coarse)
  a/d         Left motor  -/+ (fine)
  j/l         Right motor -/+ (fine)
  SPACE       Send MOT with current targets
  0/1         WALL disable/enable
  r           Toggle raw RX display
  c           Clear log

Notes:
- This is a terminal GUI using curses (no mouse required).
- On Windows, curses may not be available by default.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Deque, Optional

try:
    import curses
except ImportError as e:
    raise SystemExit("This script requires curses (typically available on Linux/macOS).") from e

try:
    import serial  # pyserial
except ImportError as e:
    raise SystemExit("pyserial is required. Install with: pip install pyserial") from e


@dataclass
class SenFrame:
    gyro_dps: float
    vbatt: float
    lf: int
    ls: int
    rs: int
    rf: int
    enc_r: int
    enc_l: int
    odo_dist_mm: float
    odo_ang_deg: float
    ts: float


def parse_sen_line(line: str) -> Optional[SenFrame]:
    parts = line.strip().split(',')
    if len(parts) != 11 or parts[0] != 'SEN':
        return None
    try:
        return SenFrame(
            gyro_dps=float(parts[1]),
            vbatt=float(parts[2]),
            lf=int(parts[3]),
            ls=int(parts[4]),
            rs=int(parts[5]),
            rf=int(parts[6]),
            enc_r=int(parts[7]),
            enc_l=int(parts[8]),
            odo_dist_mm=float(parts[9]),
            odo_ang_deg=float(parts[10]),
            ts=time.time(),
        )
    except ValueError:
        return None


class SerialClient:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._stop = threading.Event()

        self.msg_queue: "queue.Queue[str]" = queue.Queue()
        self.last_sen: Optional[SenFrame] = None
        self.print_raw = False

    def start(self) -> None:
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._rx_thread.join(timeout=1.0)
        except RuntimeError:
            pass
        self.ser.close()

    def send_mot(self, right: int, left: int) -> None:
        line = f"MOT,{right},{left}\n"
        self.ser.write(line.encode('ascii', errors='ignore'))

    def send_wall(self, enabled: bool) -> None:
        line = f"WALL,{1 if enabled else 0}\n"
        self.ser.write(line.encode('ascii', errors='ignore'))

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                self.msg_queue.put(f"#Serial error: {e}")
                break

            if not raw:
                continue

            line = raw.decode('ascii', errors='replace').rstrip('\r\n')
            # curses.addstr/addnstr cannot handle embedded NULs.
            # Also remove other control chars (keep TAB if you ever need it).
            if '\x00' in line:
                line = line.replace('\x00', '')
            line = ''.join(ch for ch in line if (ch >= ' ' or ch == '\t'))

            if self.print_raw:
                self.msg_queue.put(line)

            if line.startswith('#'):
                self.msg_queue.put(line)
                continue

            if line.startswith('SEN,'):
                sen = parse_sen_line(line)
                if sen is not None:
                    self.last_sen = sen
                else:
                    self.msg_queue.put(f"#Bad SEN: {line}")
                continue

            self.msg_queue.put(f"#RX: {line}")


def clamp_i16(n: int) -> int:
    return max(-32768, min(32767, n))


def fmt_age_ms(ts: float) -> str:
    return f"{(time.time() - ts) * 1000.0:5.0f}ms"


def draw(stdscr, client: SerialClient) -> int:
    from collections import deque

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    log: Deque[str] = deque(maxlen=200)
    wall_enabled: Optional[bool] = None

    mot_r = 0
    mot_l = 0
    last_tx = "-"

    # Command line input state
    cmd_mode = False
    cmd_buf = ""

    def push_log(s: str) -> None:
        log.append(s)

    def send_command_from_text(text: str) -> None:
        """Parse 'MOT 20 20' style input and send as CSV line."""
        nonlocal mot_r, mot_l, last_tx

        t = text.strip()
        if not t:
            return

        parts = t.split()
        name = parts[0].upper()
        args = parts[1:]

        # Convert to comma-separated protocol: NAME,arg1,arg2\n
        # Keep MOT targets in sync if user uses MOT
        if name == 'MOT':
            if len(args) != 2:
                push_log("#Usage: MOT <right> <left>")
                return
            try:
                mot_r = clamp_i16(int(args[0]))
                mot_l = clamp_i16(int(args[1]))
            except ValueError:
                push_log("#MOT args must be integers")
                return
            client.send_mot(mot_r, mot_l)
            last_tx = f"MOT,{mot_r},{mot_l}"
            push_log(f"#TX: {last_tx}")
            return

        if name == 'WALL':
            if len(args) != 1 or args[0] not in ('0', '1'):
                push_log("#Usage: WALL 0|1")
                return
            enabled = args[0] == '1'
            client.send_wall(enabled)
            last_tx = f"WALL,{1 if enabled else 0}"
            push_log(f"#TX: {last_tx}")
            return

        # Generic passthrough for future commands
        csv = name
        if args:
            csv += "," + ",".join(args)
        try:
            client.ser.write((csv + "\n").encode('ascii', errors='ignore'))
            last_tx = csv
            push_log(f"#TX: {csv}")
        except Exception as e:
            push_log(f"#TX error: {e}")

    push_log("#Connected")

    while True:
        # Drain RX messages
        while True:
            try:
                msg = client.msg_queue.get_nowait()
            except queue.Empty:
                break
            push_log(msg)
            if msg.startswith("#WallSensor enabled="):
                wall_enabled = msg.strip().endswith("1")

        # Handle keys
        key = stdscr.getch()
        if key != -1:
            # Command input mode
            if cmd_mode:
                if key in (27,):  # ESC
                    cmd_mode = False
                    cmd_buf = ""
                elif key in (curses.KEY_ENTER, 10, 13):
                    send_command_from_text(cmd_buf)
                    cmd_mode = False
                    cmd_buf = ""
                elif key in (curses.KEY_BACKSPACE, 127, 8):
                    cmd_buf = cmd_buf[:-1]
                elif 0 <= key <= 255:
                    ch = chr(key)
                    if ch.isprintable():
                        cmd_buf += ch

            else:
                # Global keys
                if key in (ord('q'), ord('Q')):
                    return 0
                if key in (ord(':'),):
                    cmd_mode = True
                    cmd_buf = ""
                # Keep raw toggle and clear log
                elif key in (ord('r'), ord('R')):
                    client.print_raw = not client.print_raw
                    push_log(f"#raw={client.print_raw}")
                elif key in (ord('c'), ord('C')):
                    log.clear()
                    push_log("#log cleared")

                # (Optional) keep existing motor hotkeys for convenience
                elif key == curses.KEY_UP:
                    mot_r = clamp_i16(mot_r + 50)
                    mot_l = clamp_i16(mot_l + 50)
                elif key == curses.KEY_DOWN:
                    mot_r = clamp_i16(mot_r - 50)
                    mot_l = clamp_i16(mot_l - 50)
                elif key == curses.KEY_LEFT:
                    mot_r = clamp_i16(mot_r - 25)
                    mot_l = clamp_i16(mot_l + 25)
                elif key == curses.KEY_RIGHT:
                    mot_r = clamp_i16(mot_r + 25)
                    mot_l = clamp_i16(mot_l - 25)
                elif key in (ord('a'), ord('A')):
                    mot_l = clamp_i16(mot_l - 10)
                elif key in (ord('d'), ord('D')):
                    mot_l = clamp_i16(mot_l + 10)
                elif key in (ord('j'), ord('J')):
                    mot_r = clamp_i16(mot_r - 10)
                elif key in (ord('l'), ord('L')):
                    mot_r = clamp_i16(mot_r + 10)
                elif key == ord(' '):
                    client.send_mot(mot_r, mot_l)
                    last_tx = f"MOT,{mot_r},{mot_l}"
                    push_log(f"#TX: {last_tx}")

        # Draw UI
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        title = "robosweep_twilight / ESP32 Serial TUI"
        stdscr.addnstr(0, 0, title, w - 1)

        # Command line
        if cmd_mode:
            prompt = ":" + cmd_buf
        else:
            prompt = "Press ':' to enter a command (e.g. MOT 20 20, WALL 1)"
        stdscr.addnstr(1, 0, prompt, w - 1)

        # SEN panel
        sen = client.last_sen
        if sen is None:
            sen_lines = ["SEN: (waiting...)" ]
        else:
            sen_lines = [
                f"SEN age={fmt_age_ms(sen.ts)}  gyro_z={sen.gyro_dps:.2f} dps  vbatt={sen.vbatt:.2f} V",
                f"wall lf/ls/rs/rf = {sen.lf}/{sen.ls}/{sen.rs}/{sen.rf}",
                f"enc  r/l = {sen.enc_r}/{sen.enc_l}",
                f"odo  dist={sen.odo_dist_mm:.2f} mm  ang={sen.odo_ang_deg:.2f} deg",
            ]
        for i, line in enumerate(sen_lines, start=3):
            if i >= h - 1:
                break
            stdscr.addnstr(i, 0, line, w - 1)

        # Control panel
        we = "?" if wall_enabled is None else ("ON" if wall_enabled else "OFF")
        ctrl_y = 3 + len(sen_lines) + 1
        if ctrl_y < h - 1:
            stdscr.addnstr(ctrl_y, 0, f"MOT target: R={mot_r:6d}  L={mot_l:6d}   WALL={we}   last_tx={last_tx}", w - 1)
            ctrl_y += 1
        if ctrl_y < h - 1:
            stdscr.addnstr(
                ctrl_y,
                0,
                "Keys: ':'=command  arrows/a/d/j/l=adjust MOT  SPACE=send MOT  r=raw  c=clear  q=quit",
                w - 1,
            )

        # Log window
        log_top = ctrl_y + 2
        log_height = max(0, h - log_top - 1)
        if log_height > 0:
            stdscr.addnstr(log_top - 1, 0, "Messages:", w - 1)
            lines = list(log)[-log_height:]
            for idx, line in enumerate(lines):
                stdscr.addnstr(log_top + idx, 0, line, w - 1)

        stdscr.refresh()
        time.sleep(0.03)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='Serial port, e.g. /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=3000000, help='Baudrate (default: 3000000)')
    args = ap.parse_args()

    client = SerialClient(args.port, args.baud)
    client.start()

    try:
        return curses.wrapper(draw, client)
    finally:
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
