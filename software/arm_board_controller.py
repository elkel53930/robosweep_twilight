#!/usr/bin/env python3
"""
arm_board_controller.py - PC側でArduino Nanoを制御するPythonクラス

通信プロトコル:
- 制御コマンド:
  - S1<angle>: サーボ1の角度設定 (0-180)
  - S2<angle>: サーボ2の角度設定 (0-180) 
  - M<speed>: モータ速度設定 (0-100%)
  - STATUS: センサーデータ要求
  - STOP: 緊急停止

- 応答:
  - OK:<command>:<value>: コマンド成功
  - ERROR:<message>: エラー
  - DATA:<battery>,<current>,<power>: センサーデータ (100ms毎)
"""

import serial
import threading
import time
import queue
from dataclasses import dataclass
from typing import Optional, Callable


@dataclass
class SensorData:
    """センサーデータクラス"""
    battery_voltage: float  # バッテリー電圧 [V]
    current_mA: float      # 電流 [mA]
    power_mW: float        # 消費電力 [mW]
    timestamp: float       # タイムスタンプ


class ArmBoardController:
    """Arduino Nano アームボード制御クラス"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """
        初期化
        
        Args:
            port: シリアルポート
            baudrate: ボーレート
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_running = False
        
        # データ受信用
        self.sensor_data_queue = queue.Queue(maxsize=100)
        self.latest_sensor_data: Optional[SensorData] = None
        self.sensor_callback: Optional[Callable[[SensorData], None]] = None
        
        # スレッド用
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
    
    def connect(self) -> bool:
        """
        Arduinoに接続
        
        Returns:
            接続成功の場合True
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            time.sleep(2)  # Arduino初期化待機
            
            self.is_connected = True
            self.is_running = True
            
            # 受信スレッド開始
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            print(f"Connected to Arduino on {self.port}")
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """接続を切断"""
        self.is_running = False
        self.is_connected = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        print("Disconnected from Arduino")
    
    def _send_command(self, command: str) -> bool:
        """
        コマンド送信
        
        Args:
            command: 送信するコマンド
            
        Returns:
            送信成功の場合True
        """
        if not self.is_connected or not self.serial_conn:
            return False
        
        try:
            with self.lock:
                self.serial_conn.write((command + '\n').encode())
                self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"Send command error: {e}")
            return False
    
    def _read_loop(self):
        """受信ループ（別スレッドで実行）"""
        while self.is_running and self.serial_conn:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode().strip()
                    if line:
                        self._process_received_line(line)
                time.sleep(0.001)  # CPU使用率を抑制
                
            except Exception as e:
                print(f"Read error: {e}")
                break
    
    def _process_received_line(self, line: str):
        """受信行を処理"""
        if line.startswith("DATA:"):
            # センサーデータ処理
            try:
                data_part = line[5:]  # "DATA:"を除去
                values = [float(x) for x in data_part.split(',')]
                
                if len(values) >= 3:
                    sensor_data = SensorData(
                        battery_voltage=values[0],
                        current_mA=values[1],
                        power_mW=values[2],
                        timestamp=time.time()
                    )
                    
                    self.latest_sensor_data = sensor_data
                    
                    # キューに追加（満杯の場合は古いデータを破棄）
                    try:
                        self.sensor_data_queue.put_nowait(sensor_data)
                    except queue.Full:
                        try:
                            self.sensor_data_queue.get_nowait()  # 古いデータを破棄
                            self.sensor_data_queue.put_nowait(sensor_data)
                        except queue.Empty:
                            pass
                    
                    # コールバック呼び出し
                    if self.sensor_callback:
                        self.sensor_callback(sensor_data)
                        
            except Exception as e:
                print(f"Sensor data parse error: {e}")
        
        elif line.startswith("OK:") or line.startswith("ERROR:") or line.startswith("DEBUG:"):
            # 制御応答やデバッグメッセージを表示
            print(f"Arduino: {line}")
    
    # 制御メソッド
    
    def set_servo1_angle(self, angle: int) -> bool:
        """
        サーボ1の角度を設定
        
        Args:
            angle: 角度 (0-180)
            
        Returns:
            送信成功の場合True
        """
        angle = max(0, min(180, angle))  # 0-180に制限
        return self._send_command(f"S1{angle:03d}")
    
    def set_servo2_angle(self, angle: int) -> bool:
        """
        サーボ2の角度を設定
        
        Args:
            angle: 角度 (0-180)
            
        Returns:
            送信成功の場合True
        """
        angle = max(0, min(180, angle))  # 0-180に制限
        return self._send_command(f"S2{angle:03d}")
    
    def set_motor_speed(self, speed: int) -> bool:
        """
        モータ速度を設定
        
        Args:
            speed: 速度 (0-100%)
            
        Returns:
            送信成功の場合True
        """
        speed = max(0, min(100, speed))  # 0-100に制限
        return self._send_command(f"M{speed:03d}")
    
    def emergency_stop(self) -> bool:
        """
        緊急停止
        
        Returns:
            送信成功の場合True
        """
        return self._send_command("STOP")
    
    def request_status(self) -> bool:
        """
        ステータス要求
        
        Returns:
            送信成功の場合True
        """
        return self._send_command("STATUS")
    
    # データ取得メソッド
    
    def get_latest_sensor_data(self) -> Optional[SensorData]:
        """
        最新のセンサーデータを取得
        
        Returns:
            最新のセンサーデータ、またはNone
        """
        return self.latest_sensor_data
    
    def get_sensor_data_queue(self) -> queue.Queue:
        """
        センサーデータキューを取得
        
        Returns:
            センサーデータキュー
        """
        return self.sensor_data_queue
    
    def set_sensor_callback(self, callback: Callable[[SensorData], None]):
        """
        センサーデータ受信時のコールバック関数を設定
        
        Args:
            callback: センサーデータを引数とするコールバック関数
        """
        self.sensor_callback = callback
    
    # コンテキストマネージャー対応
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# 使用例
if __name__ == "__main__":
    def sensor_callback(data: SensorData):
        """センサーデータ受信時のコールバック"""
        print(f"Battery: {data.battery_voltage:.2f}V, "
              f"Current: {data.current_mA:.1f}mA, "
              f"Power: {data.power_mW:.1f}mW")
    
    # 使用例1: コンテキストマネージャー
    try:
        with ArmBoardController('/dev/ttyUSB0') as controller:
            # コールバック設定
            controller.set_sensor_callback(sensor_callback)
            
            # 制御例
            print("Testing servo control...")
            controller.set_servo1_angle(90)
            time.sleep(1)
            controller.set_servo2_angle(45)
            time.sleep(1)
            
            print("Testing motor control...")
            controller.set_motor_speed(30)
            time.sleep(2)
            controller.set_motor_speed(0)
            
            print("Running for 10 seconds...")
            time.sleep(10)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")