#!/usr/bin/env python3
"""スレッド化されたMobileBaseクラス

DONE待ちの間もコマンドキューをチェックし、QSTPコマンドを受け付ける。
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional, Dict, Any

import serial


class MobileBaseThread:
    """MobileBaseをスレッドで操作するクラス"""
    
    def __init__(self, ser: serial.Serial, timeout: float = 5.0):
        self.ser = ser
        self.timeout = timeout
        
        # コマンドキュー（メインスレッド → MobileBaseスレッド）
        self.cmd_queue = queue.Queue()
        
        # 応答キュー（MobileBaseスレッド → メインスレッド）
        self.response_queue = queue.Queue()
        
        # センサーデータ共有変数
        self.sensor_data_lock = threading.Lock()
        self.sensor_data: Optional[Dict[str, Any]] = None
        
        # スレッド制御
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # QSTP要求フラグ（DONE待ち中にQSTPを受け付けるため）
        self.qstp_requested = False
        self.qstp_lock = threading.Lock()
    
    def start(self):
        """スレッドを開始"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._thread_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """スレッドを停止"""
        if not self.running:
            return
        
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
    
    def send_command(self, cmd: str, *args) -> None:
        """コマンドを送信（非ブロッキング）"""
        self.cmd_queue.put((cmd, args))
    
    def wait_response(self, timeout: Optional[float] = None, block: bool = True) -> tuple[str, Any]:
        """応答を待つ（ブロッキング）
        
        Returns:
            (response_type, data) のタプル
            response_type: 'DONE', 'QSTPDONE', 'ERROR', 'TIMEOUT'
        """
        try:
            return self.response_queue.get(timeout=timeout or self.timeout, block=block)
        except queue.Empty:
            return ('TIMEOUT', None)
    
    def get_sensor_data(self) -> Optional[Dict[str, Any]]:
        """最新のセンサーデータを取得"""
        with self.sensor_data_lock:
            return self.sensor_data.copy() if self.sensor_data else None
    
    def request_qstp(self):
        """QSTP要求を設定（DONE待ち中に処理される）"""
        with self.qstp_lock:
            self.qstp_requested = True
    
    def _thread_loop(self):
        """スレッドのメインループ"""
        while self.running:
            try:
                # コマンドキューから取得（タイムアウト付き）
                try:
                    cmd, args = self.cmd_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                # コマンド実行
                self._execute_command(cmd, args)
                
            except Exception as e:
                print(f"#MobileBaseThread error: {e}")
                import traceback
                traceback.print_exc()
    
    def _execute_command(self, cmd: str, args: tuple):
        """コマンドを実行"""
        try:
            if cmd == 'GCAL':
                self._cmd_gcal()
            elif cmd == 'RDST':
                self._cmd_rdst()
            elif cmd == 'RANG':
                self._cmd_rang()
            elif cmd == 'WALL':
                self._cmd_wall(args[0])
            elif cmd == 'FWD':
                self._cmd_fwd(*args)
            elif cmd == 'STOP':
                self._cmd_stop(*args)
            elif cmd == 'TURN':
                self._cmd_turn(args[0])
            elif cmd == 'QSTP':
                self._cmd_qstp()
            elif cmd == 'SEN':
                self._cmd_sen()
            else:
                print(f"#Unknown command: {cmd}")
                self.response_queue.put(('ERROR', f'Unknown command: {cmd}'))
        except serial.SerialException as e:
            print(f"#Serial communication error: {e}")
            import traceback
            traceback.print_exc()
            self.response_queue.put(('ERROR', f'Serial error: {e}'))
        except Exception as e:
            print(f"#Command execution error: {e}")
            import traceback
            traceback.print_exc()
            self.response_queue.put(('ERROR', str(e)))
    
    def _send(self, line: str) -> None:
        """コマンドを送信"""
        try:
            self.ser.reset_input_buffer()
        except Exception as e:
            print(f"#Warning: Failed to reset input buffer: {e}")
        
        time.sleep(0.01)
        
        try:
            self.ser.write(line.encode("ascii", errors="ignore"))
            print(f"TX: {line.strip()}")
        except serial.SerialException as e:
            print(f"#Error: Failed to send command: {e}")
            raise
    
    def _wait_done_with_qstp_check(self) -> None:
        """DONE応答を待機（QSTP要求をチェックしながら）"""
        deadline = time.time() + self.timeout
        buf = []

        while time.time() < deadline:
            # QSTP要求をチェック
            with self.qstp_lock:
                if self.qstp_requested:
                    self.qstp_requested = False
                    # QSTPコマンドを即座に実行
                    self._send("QSTP\n")
                    remaining_dist = self._wait_qstp_done()
                    self.response_queue.put(('QSTPDONE', remaining_dist))
                    return
            
            time.sleep(0.01)
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                print(f"#Serial error: {e}, retrying...")
                time.sleep(0.1)
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass
                continue
            
            if not raw:
                continue

            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if "\x00" in line:
                line = line.replace("\x00", "")
            line = "".join(ch for ch in line if (ch >= " " or ch == "\t"))
            line = line.strip()
            
            if not line:
                continue

            # SEN データを更新
            if line.startswith("SEN,"):
                self._parse_sen_line(line)
                continue

            if line.startswith("#"):
                print(line)

            if line == "DONE":
                self.response_queue.put(('DONE', None))
                return

            buf.append(line)

        tail = "\n".join(buf[-50:])
        error_msg = f"Timed out waiting for DONE (last lines):\n{tail}\n"
        print(f"#Error: {error_msg}")
        self.response_queue.put(('TIMEOUT', error_msg))
    
    def _wait_qstp_done(self) -> float:
        """QSTPDONE応答を待機して残距離を返す"""
        deadline = time.time() + self.timeout
        buf = []

        while time.time() < deadline:
            time.sleep(0.01)
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                print(f"#Serial error: {e}, retrying...")
                time.sleep(0.1)
                try:
                    self.ser.reset_input_buffer()
                except:
                    pass
                continue
            
            if not raw:
                continue

            line = raw.decode("ascii", errors="replace").rstrip("\r\n")
            if "\x00" in line:
                line = line.replace("\x00", "")
            line = "".join(ch for ch in line if (ch >= " " or ch == "\t"))
            line = line.strip()
            
            if not line:
                continue

            if line.startswith("SEN,"):
                self._parse_sen_line(line)
                continue

            if line.startswith("#"):
                print(line)

            if line.startswith("QSTPDONE,"):
                try:
                    parts = line.split(",")
                    if len(parts) >= 2:
                        remaining_dist = float(parts[1])
                        print(f"RX: QSTPDONE,{remaining_dist}")
                        return remaining_dist
                except (ValueError, IndexError) as e:
                    print(f"#Failed to parse QSTPDONE response: {line} ({e})")
                    return 0.0

            buf.append(line)

        print(f"#Timed out waiting for QSTPDONE")
        return 0.0
    
    def _parse_sen_line(self, line: str):
        """SENデータをパース
        SEN形式: SEN,gyro,vbatt,lf,ls,rs,rf,enc_r,enc_l,odo_dist,odo_ang
        """
        try:
            parts = line.split(',')
            if len(parts) >= 10:
                data = {
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
                with self.sensor_data_lock:
                    self.sensor_data = data
        except (ValueError, IndexError) as e:
            print(f"#Failed to parse SEN line: {line} ({e})")
    
    # コマンド実装
    def _cmd_gcal(self):
        self._send("GCAL\n")
        time.sleep(1)  # キャリブレーション待機
        self.response_queue.put(('DONE', None))
    
    def _cmd_rdst(self):
        self._send("RDST\n")
        self._wait_done_with_qstp_check()
    
    def _cmd_rang(self):
        self._send("RANG\n")
        self._wait_done_with_qstp_check()
    
    def _cmd_wall(self, enable: bool):
        self._send(f"WALL,{1 if enable else 0}\n")
        # WALLはDONE応答がない
        self.response_queue.put(('DONE', None))
    
    def _cmd_fwd(self, speed: float, accel: float, distance: float):
        self._send(f"FWD,{speed},{accel},{distance}\n")
        self._wait_done_with_qstp_check()
    
    def _cmd_stop(self, speed: float, accel: float, distance: float):
        self._send(f"STOP,{speed},{accel},{distance}\n")
        self._wait_done_with_qstp_check()
    
    def _cmd_turn(self, angle: float):
        self._send(f"TURN,{angle}\n")
        self._wait_done_with_qstp_check()
    
    def _cmd_qstp(self):
        self._send("QSTP\n")
        remaining_dist = self._wait_qstp_done()
        self.response_queue.put(('QSTPDONE', remaining_dist))
    
    def _cmd_sen(self):
        # SENコマンドを送信してセンサーデータを取得
        try:
            self._send("SEN\n")
        except serial.SerialException as e:
            print(f"#Error: Failed to send SEN command: {e}")
            self.response_queue.put(('ERROR', f'Serial write error: {e}'))
            return
        
        time.sleep(0.05)  # 応答待ち
        
        # SEN応答を読み取る
        deadline = time.time() + 2.0  # タイムアウトを2秒に延長
        lines_read = 0
        
        while time.time() < deadline:
            try:
                raw = self.ser.readline()
            except serial.SerialException as e:
                print(f"#Error: Failed to read SEN response: {e}")
                self.response_queue.put(('ERROR', f'Serial read error: {e}'))
                return
            
            if not raw:
                time.sleep(0.01)
                continue
            
            lines_read += 1
            line = raw.decode("ascii", errors="replace").rstrip("\r\n").strip()
            
            # デバッグ: 読み取った行を表示
            if line and not line.startswith("#"):
                print(f"#RX: {line[:50]}...")  # 最初の50文字のみ
            
            if line.startswith("SEN,"):
                self._parse_sen_line(line)
                data = self.get_sensor_data()
                if data:
                    self.response_queue.put(('DONE', data))
                    return
                else:
                    print(f"#Warning: Failed to parse SEN data")
        
        # タイムアウト
        print(f"#Warning: SEN timeout after reading {lines_read} lines")
        self.response_queue.put(('DONE', None))
