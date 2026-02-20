#!/usr/bin/env python3
"""
arm.py - 統合アームコントローラー

Futabaサーボ(RS204MD/RS304MD)とArduino Nanoを統合した単一クラス。
- servo_arm: Futabaコマンドサーボ（投擲アーム用）
- servo_launcher: Arduino制御サーボ（ランチャー用）
- motor: Arduino制御DCモータ
- sensor: Arduino経由のセンサーデータ
"""

import serial
import threading
import time
import queue
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Callable


@dataclass
class SensorData:
    """センサーデータクラス"""
    battery_voltage: float  # バッテリー電圧 [V]
    current_mA: float      # 電流 [mA]
    power_mW: float        # 消費電力 [mW]
    timestamp: float       # タイムスタンプ


class ArmBase(ABC):
    """アームコントローラーの抽象基底クラス"""
    
    # アーム角度定数
    CATCH_POSITION = None    # ボールキャッチ位置
    THROW_POSITION = -45.0   # 投擲位置
    RUN_POSITION = None     # 走行位置
    
    # ランチャーサーボ角度定数
    LAUNCHER_RELOAD = 180    # リロード位置
    LAUNCHER_READY = 30      # 待機位置
    LAUNCHER_FIRE = 0        # 発射位置
    
    @abstractmethod
    def connect_arduino(self) -> bool:
        """Arduinoに接続"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """すべての接続を切断"""
        pass
    
    @abstractmethod
    def set_servo_arm_torque(self, enable: bool):
        """アームサーボのトルクON/OFF"""
        pass
    
    @abstractmethod
    def set_servo_arm_angle(self, angle: float, move_time: int = 0):
        """アームサーボの目標角度設定"""
        pass
    
    @abstractmethod
    def set_servo_arm_brake(self):
        """アームサーボをブレーキモードに設定"""
        pass
    
    @abstractmethod
    def set_servo_launcher_angle(self, angle: int) -> bool:
        """ランチャーサーボの角度を設定"""
        pass
    
    @abstractmethod
    def set_servo_launcher_angle_with_speed(self, angle: int, speed: int) -> bool:
        """ランチャーサーボの角度を指定速度で設定"""
        pass
    
    @abstractmethod
    def set_servo_launcher_speed(self, speed: int) -> bool:
        """ランチャーサーボのデフォルト速度を設定"""
        pass
    
    @abstractmethod
    def detach_servo_launcher(self) -> bool:
        """ランチャーサーボを脱力状態にする"""
        pass
    
    @abstractmethod
    def set_motor_speed(self, speed: int) -> bool:
        """モータ速度を設定"""
        pass
    
    @abstractmethod
    def emergency_stop(self) -> bool:
        """緊急停止"""
        pass
    
    @abstractmethod
    def request_status(self) -> bool:
        """ステータス要求"""
        pass
    
    @abstractmethod
    def get_latest_sensor_data(self) -> Optional[SensorData]:
        """最新のセンサーデータを取得"""
        pass
    
    @abstractmethod
    def get_sensor_data_queue(self) -> queue.Queue:
        """センサーデータキューを取得"""
        pass
    
    @abstractmethod
    def set_sensor_callback(self, callback: Callable[[SensorData], None]):
        """センサーデータ受信時のコールバック関数を設定"""
        pass
    
    @abstractmethod
    def __enter__(self):
        """コンテキストマネージャー対応"""
        pass
    
    @abstractmethod
    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャー対応"""
        pass


class Arm(ArmBase):
    """統合アームコントローラークラス"""
    
    # Futabaサーボのメモリマップアドレス
    ADDR_TORQUE_ENABLE = 0x24
    ADDR_GOAL_POSITION = 0x1E
    ADDR_GOAL_TIME = 0x20
    
    # アーム角度定数
    CATCH_POSITION = None    # ボールキャッチ位置
    THROW_POSITION = -45.0   # 投擲位置
    RUN_POSITION = None     # 走行位置
    
    # ランチャーサーボ角度定数
    LAUNCHER_RELOAD = 180    # リロード位置
    LAUNCHER_READY = 30      # 待機位置
    LAUNCHER_FIRE = 0        # 発射位置
    
    def __init__(self, 
                 machine_id: int,
                 futaba_port='/dev/ttyAMA0', 
                 futaba_baudrate=115200,
                 arduino_port='/dev/ttyARM', 
                 arduino_baudrate=115200,
                 arm_servo_id=1,
                 arm_min_angle=-90.0,
                 arm_max_angle=45.0):
        """
        初期化
        
        Args:
            futaba_port: Futabaサーボのシリアルポート (Raspberry Pi 5では /dev/ttyAMA0)
            futaba_baudrate: Futaba通信速度 (デフォルト: 115200bps)
            arduino_port: Arduinoのシリアルポート
            arduino_baudrate: Arduino通信速度
            arm_servo_id: アームサーボのID (デフォルト: 1)
            arm_min_angle: アームサーボの最小角度 [度] (デフォルト: -90.0)
            arm_max_angle: アームサーボの最大角度 [度] (デフォルト: 45.0)
        """
        if machine_id == 1:
            Arm.CATCH_POSITION = 42.0
            Arm.RUN_POSITION = -90.0
        elif machine_id == 2:
            Arm.CATCH_POSITION = 46.0
            Arm.RUN_POSITION = -85
        # Futabaサーボ設定
        self.arm_servo_id = arm_servo_id
        self.arm_min_angle = arm_min_angle
        self.arm_max_angle = arm_max_angle
        self.futaba_serial: Optional[serial.Serial] = None
        
        # Arduino設定
        self.arduino_port = arduino_port
        self.arduino_baudrate = arduino_baudrate
        self.arduino_serial: Optional[serial.Serial] = None
        self.is_arduino_connected = False
        self.is_running = False
        
        # センサーデータ受信用
        self.sensor_data_queue = queue.Queue(maxsize=100)
        self.latest_sensor_data: Optional[SensorData] = None
        self.sensor_callback: Optional[Callable[[SensorData], None]] = None
        
        # スレッド用
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        
        # Futabaサーボ接続
        try:
            self.futaba_serial = serial.Serial(
                port=futaba_port,
                baudrate=futaba_baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            time.sleep(0.1)  # 安定化待ち
            print(f"Connected to Futaba servo on {futaba_port}")
        except Exception as e:
            print(f"Futaba servo connection failed: {e}")
            self.futaba_serial = None
    
    def connect_arduino(self) -> bool:
        """
        Arduinoに接続
        
        Returns:
            接続成功の場合True
        """
        try:
            self.arduino_serial = serial.Serial(
                port=self.arduino_port,
                baudrate=self.arduino_baudrate,
                timeout=1.0
            )
            time.sleep(2)  # Arduino初期化待機
            
            self.is_arduino_connected = True
            self.is_running = True
            
            # 受信スレッド開始
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            print(f"Connected to Arduino on {self.arduino_port}")
            return True
            
        except Exception as e:
            print(f"Arduino connection failed: {e}")
            return False
    
    def disconnect(self):
        """すべての接続を切断"""
        # Arduino切断
        self.is_running = False
        self.is_arduino_connected = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
            print("Disconnected from Arduino")
        
        # Futabaサーボ切断
        if self.futaba_serial and self.futaba_serial.is_open:
            self.futaba_serial.close()
            print("Disconnected from Futaba servo")
    
    def __del__(self):
        """デストラクタ"""
        self.disconnect()
    
    # ========================================
    # Futabaサーボ（アーム）制御メソッド
    # ========================================
    
    def _calculate_checksum(self, data):
        """
        チェックサム計算 (XOR演算)
        
        Args:
            data: ID以降のバイト列
            
        Returns:
            チェックサム値
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def _send_short_packet(self, servo_id, flag, address, data):
        """
        Futabaサーボにショートパケット送信
        
        Args:
            servo_id: サーボID (1-127)
            flag: フラグ
            address: メモリマップアドレス
            data: 送信データ (list or bytes)
        """
        if not self.futaba_serial or not self.futaba_serial.is_open:
            print("Futaba servo not connected")
            return
        
        # パケット構築
        packet = bytearray()
        packet.append(0xFA)  # Header L
        packet.append(0xAF)  # Header H
        packet.append(servo_id)
        packet.append(flag)
        packet.append(address)
        packet.append(len(data))  # Length
        packet.append(0x01)  # Count (ショートパケットは常に1)
        
        # データ追加
        if isinstance(data, int):
            packet.append(data)
        else:
            packet.extend(data)
        
        # チェックサム計算 (ID以降)
        checksum_data = packet[2:]
        checksum = self._calculate_checksum(checksum_data)
        packet.append(checksum)
        
        # 送信
        self.futaba_serial.write(packet)
        self.futaba_serial.flush()
    
    def set_servo_arm_torque(self, enable: bool):
        """
        アームサーボのトルクON/OFF
        
        Args:
            enable: True=トルクON, False=トルクOFF
        """
        data = [0x01 if enable else 0x00]
        self._send_short_packet(self.arm_servo_id, 0x00, self.ADDR_TORQUE_ENABLE, data)
        time.sleep(0.01)
    
    def set_servo_arm_angle(self, angle: float, move_time: int = 0):
        """
        アームサーボの目標角度設定
        
        Args:
            angle: 目標角度 [度]
            move_time: 移動時間 [ms] (0の場合は最高速度)
        """
        # 角度を0.1度単位の整数に変換
        angle_value = int(angle * 10)
        
        # 範囲チェック
        if angle < self.arm_min_angle or angle > self.arm_max_angle:
            raise ValueError(f"角度が範囲外です: {angle}度 ({self.arm_min_angle}度〜{self.arm_max_angle}度まで)")
        
        # サーボの物理的制限チェック
        if angle_value < -1500 or angle_value > 1500:
            raise ValueError(f"角度がサーボの物理的制限を超えています: {angle}度 (±150.0度まで)")
        
        # 2の補数表現に変換（負の値の場合）
        if angle_value < 0:
            angle_value = 0x10000 + angle_value
        
        # データ構築（リトルエンディアン）
        data = bytearray()
        data.append(angle_value & 0xFF)        # Low byte
        data.append((angle_value >> 8) & 0xFF) # High byte
        
        # 移動時間も指定する場合
        if move_time > 0:
            time_value = int(move_time / 10)  # 10ms単位に変換
            data.append(time_value & 0xFF)
            data.append((time_value >> 8) & 0xFF)
        
        self._send_short_packet(self.arm_servo_id, 0x00, self.ADDR_GOAL_POSITION, data)
        time.sleep(0.01)
    
    def set_servo_arm_brake(self):
        """
        アームサーボをブレーキモードに設定
        """
        data = [0x02]
        self._send_short_packet(self.arm_servo_id, 0x00, self.ADDR_TORQUE_ENABLE, data)
        time.sleep(0.01)
    
    # ========================================
    # Arduino制御メソッド
    # ========================================
    
    def _send_arduino_command(self, command: str) -> bool:
        """
        Arduinoにコマンド送信
        
        Args:
            command: 送信するコマンド
            
        Returns:
            送信成功の場合True
        """
        if not self.is_arduino_connected or not self.arduino_serial:
            print("Arduino not connected")
            return False
        
        try:
            with self.lock:
                self.arduino_serial.write((command + '\n').encode())
                self.arduino_serial.flush()
                time.sleep(0.1)
            return True
        except Exception as e:
            print(f"Arduino send command error: {e}")
            return False
    
    def _read_loop(self):
        """Arduino受信ループ（別スレッドで実行）"""
        while self.is_running and self.arduino_serial:
            try:
                if self.arduino_serial.in_waiting > 0:
                    line = self.arduino_serial.readline().decode().strip()
                    if line:
                        self._process_received_line(line)
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Arduino read error: {e}")
                break
    
    def _process_received_line(self, line: str):
        """Arduino受信行を処理"""
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
                            self.sensor_data_queue.get_nowait()
                            self.sensor_data_queue.put_nowait(sensor_data)
                        except queue.Empty:
                            pass
                    
                    # コールバック呼び出し
                    if self.sensor_callback:
                        self.sensor_callback(sensor_data)
                        
            except Exception as e:
                print(f"Sensor data parse error: {e}")
        
        elif line.startswith("OK:") or line.startswith("ERROR:") or line.startswith("DEBUG:"):
            print(f"Arduino: {line}")
    
    # ランチャーサーボ制御メソッド
    
    def set_servo_launcher_angle(self, angle: int) -> bool:
        """
        ランチャーサーボの角度を設定（即座に移動）
        
        Args:
            angle: 角度 (0-180)
            
        Returns:
            送信成功の場合True
        """
        angle = max(0, min(180, angle))
        return self._send_arduino_command(f"S1{angle:03d}")
    
    def set_servo_launcher_angle_with_speed(self, angle: int, speed: int) -> bool:
        """
        ランチャーサーボの角度を指定速度で設定
        
        Args:
            angle: 目標角度 (0-180)
            speed: 移動速度 (1-180 degrees/sec)
            
        Returns:
            送信成功の場合True
        """
        angle = max(0, min(180, angle))
        speed = max(1, min(180, speed))
        return self._send_arduino_command(f"S1A{angle:03d},{speed:03d}")
    
    def set_servo_launcher_speed(self, speed: int) -> bool:
        """
        ランチャーサーボのデフォルト速度を設定
        
        Args:
            speed: デフォルト速度 (1-180 degrees/sec)
            
        Returns:
            送信成功の場合True
        """
        speed = max(1, min(180, speed))
        return self._send_arduino_command(f"S1S{speed:03d}")
    
    def detach_servo_launcher(self) -> bool:
        """
        ランチャーサーボを脱力状態にする（電源OFF）
        
        Returns:
            送信成功の場合True
        """
        return self._send_arduino_command("S1D")
    
    # モータ制御メソッド
    
    def set_motor_speed(self, speed: int) -> bool:
        """
        モータ速度を設定
        
        Args:
            speed: 速度 (0-100%)
            
        Returns:
            送信成功の場合True
        """
        speed = max(0, min(100, speed))
        return self._send_arduino_command(f"M{speed:03d}")
    
    # その他制御メソッド
    
    def emergency_stop(self) -> bool:
        """
        緊急停止（Arduino制御の全アクチュエータを停止）
        
        Returns:
            送信成功の場合True
        """
        return self._send_arduino_command("STOP")
    
    def request_status(self) -> bool:
        """
        Arduinoのステータス要求
        
        Returns:
            送信成功の場合True
        """
        return self._send_arduino_command("STATUS")
    
    # センサーデータ取得メソッド
    
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
        self.connect_arduino()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


class ArmDummy(ArmBase):
    """ダミーアームコントローラークラス（何もしない）"""
    
    def __init__(self, 
                 futaba_port='/dev/ttyAMA0', 
                 futaba_baudrate=115200,
                 arduino_port='/dev/ttyARM', 
                 arduino_baudrate=115200,
                 arm_servo_id=1,
                 arm_min_angle=-90.0,
                 arm_max_angle=45.0):
        """初期化（何もしない）"""
        self.arm_servo_id = arm_servo_id
        self.arm_min_angle = arm_min_angle
        self.arm_max_angle = arm_max_angle
        self.latest_sensor_data: Optional[SensorData] = None
        self.sensor_callback: Optional[Callable[[SensorData], None]] = None
        print("ArmDummy initialized (no hardware connection)")
    
    def connect_arduino(self) -> bool:
        """Arduino接続（ダミー）"""
        print("ArmDummy: Arduino connection (dummy)")
        return True
    
    def disconnect(self):
        """切断（ダミー）"""
        print("ArmDummy: Disconnected (dummy)")
    
    def __del__(self):
        """デストラクタ（ダミー）"""
        pass
    
    def set_servo_arm_torque(self, enable: bool):
        """アームサーボのトルクON/OFF（ダミー）"""
        pass
    
    def set_servo_arm_angle(self, angle: float, move_time: int = 0):
        """アームサーボの目標角度設定（ダミー）"""
        pass
    
    def set_servo_arm_brake(self):
        """アームサーボをブレーキモードに設定（ダミー）"""
        pass
    
    def set_servo_launcher_angle(self, angle: int) -> bool:
        """ランチャーサーボの角度を設定（ダミー）"""
        return True
    
    def set_servo_launcher_angle_with_speed(self, angle: int, speed: int) -> bool:
        """ランチャーサーボの角度を指定速度で設定（ダミー）"""
        return True
    
    def set_servo_launcher_speed(self, speed: int) -> bool:
        """ランチャーサーボのデフォルト速度を設定（ダミー）"""
        return True
    
    def detach_servo_launcher(self) -> bool:
        """ランチャーサーボを脱力状態にする（ダミー）"""
        return True
    
    def set_motor_speed(self, speed: int) -> bool:
        """モータ速度を設定（ダミー）"""
        return True
    
    def emergency_stop(self) -> bool:
        """緊急停止（ダミー）"""
        return True
    
    def request_status(self) -> bool:
        """Arduinoのステータス要求（ダミー）"""
        return True
    
    def get_latest_sensor_data(self) -> Optional[SensorData]:
        """最新のセンサーデータを取得（ダミー）"""
        return self.latest_sensor_data
    
    def get_sensor_data_queue(self) -> queue.Queue:
        """センサーデータキューを取得（ダミー）"""
        return queue.Queue()
    
    def set_sensor_callback(self, callback: Callable[[SensorData], None]):
        """センサーデータ受信時のコールバック関数を設定（ダミー）"""
        self.sensor_callback = callback
    
    def __enter__(self):
        """コンテキストマネージャー対応（ダミー）"""
        self.connect_arduino()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャー対応（ダミー）"""
        self.disconnect()
