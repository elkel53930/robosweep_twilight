#!/usr/bin/env python3
"""
Futaba RS204MD/RS304MD Servo Controller for Raspberry Pi 5
Based on Arduino UNO example from:
https://kirikoshokunin.hatenablog.com/entry/2021/04/09/235732
"""

import serial
import time

class FutabaServo:
    """双葉コマンドサーボ(RS204MD/RS304MD)制御クラス"""
    
    # メモリマップアドレス
    ADDR_TORQUE_ENABLE = 0x24
    ADDR_GOAL_POSITION = 0x1E
    ADDR_GOAL_TIME = 0x20
    
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=0.1):
        """
        初期化
        
        Args:
            port: シリアルポート (Raspberry Pi 5では /dev/ttyAMA0)
            baudrate: 通信速度 (デフォルト: 115200bps)
            timeout: タイムアウト時間
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        time.sleep(0.1)  # 安定化待ち
        
    def __del__(self):
        """デストラクタ"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
    
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
        ショートパケット送信
        
        Args:
            servo_id: サーボID (1-127)
            flag: フラグ
            address: メモリマップアドレス
            data: 送信データ (list or bytes)
        """
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
        self.ser.write(packet)
        self.ser.flush()
    
    def set_torque(self, servo_id, enable):
        """
        トルクON/OFF
        
        Args:
            servo_id: サーボID
            enable: True=トルクON, False=トルクOFF
        """
        data = [0x01 if enable else 0x00]
        self._send_short_packet(servo_id, 0x00, self.ADDR_TORQUE_ENABLE, data)
        time.sleep(0.01)  # コマンド処理待ち
    
    def set_position(self, servo_id, angle, move_time=0):
        """
        目標位置設定
        
        Args:
            servo_id: サーボID
            angle: 目標角度 [度] (-150.0 ~ +150.0)
            move_time: 移動時間 [ms] (0の場合は最高速度)
        """
        # 角度を0.1度単位の整数に変換
        angle_value = int(angle * 10)
        
        # 範囲チェック
        if angle_value < -1500 or angle_value > 1500:
            raise ValueError(f"角度が範囲外です: {angle}度 (±150.0度まで)")
        
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
            self._send_short_packet(servo_id, 0x00, self.ADDR_GOAL_POSITION, data)
        else:
            self._send_short_packet(servo_id, 0x00, self.ADDR_GOAL_POSITION, data)
        
        time.sleep(0.01)
    
    def set_brake_mode(self, servo_id):
        """
        ブレーキモード設定
        
        Args:
            servo_id: サーボID
        """
        data = [0x02]
        self._send_short_packet(servo_id, 0x00, self.ADDR_TORQUE_ENABLE, data)
        time.sleep(0.01)


def main():
    """メイン処理（デモンストレーション）"""
    
    print("Futaba Servo Controller for Raspberry Pi 5")
    print("=" * 50)
    
    # サーボ初期化
    servo = FutabaServo(port='/dev/ttyAMA0', baudrate=115200)
    servo_id = 1  # サーボIDを1に設定
    
    try:
        print(f"サーボID {servo_id} を制御します")
        
        # トルクON
        print("トルクON...")
        servo.set_torque(servo_id, True)
        time.sleep(1)
        
        # +90度へ移動
        print("角度: +90.0度へ移動")
        servo.set_position(servo_id, 90.0, move_time=100)
        time.sleep(1.5)
        
        # 0度へ移動
        print("角度: 0.0度へ移動")
        servo.set_position(servo_id, 0.0, move_time=100)
        time.sleep(1.5)
        
        # -90度へ移動
        print("角度: -90.0度へ移動")
        servo.set_position(servo_id, -90.0, move_time=100)
        time.sleep(1.5)
        
        # 0度へ戻る
        print("角度: 0.0度へ移動")
        servo.set_position(servo_id, 0.0, move_time=100)
        time.sleep(1.5)
        
        # -90度へ移動
        print("角度: -90.0度へ移動")
        servo.set_position(servo_id, -90.0, move_time=100)
        time.sleep(1.5)
        
        # -90度へ移動
        print("角度: -90.0度へ移動")
        servo.set_position(servo_id, 90.0, move_time=100)
        time.sleep(1.5)
        
        # トルクOFF
        print("トルクOFF...")
        servo.set_torque(servo_id, False)
        
        print("完了！")
        
    except KeyboardInterrupt:
        print("\n中断されました")
        servo.set_torque(servo_id, False)
    except Exception as e:
        print(f"エラー: {e}")
        servo.set_torque(servo_id, False)


if __name__ == "__main__":
    main()