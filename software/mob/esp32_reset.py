#!/usr/bin/env python3
"""
ESP32 Serial Reset Script
ESP32をシリアル経由でリセットするスクリプト
"""

import serial
import time
import sys
import argparse

def reset_esp32(port, baudrate=115200, reset_time=0.1):
    """
    ESP32をシリアル経由でリセットする
    
    Args:
        port (str): シリアルポート名 (例: /dev/ttyUSB0, COM3)
        baudrate (int): ボーレート (デフォルト: 115200)
        reset_time (float): リセット信号の保持時間 (秒)
    
    Returns:
        bool: リセット成功時True
    """
    try:
        print(f"Connecting to {port} at {baudrate} baud...")
        
        # シリアルポートを開く
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print("Serial connection established")
            
            # DTR/RTSを使用してリセット
            print("Sending reset signal...")
            ser.dtr = False
            ser.rts = True
            time.sleep(reset_time)
            
            ser.dtr = True
            ser.rts = False
            time.sleep(reset_time)
            
            # 正常状態に戻す
            ser.dtr = False
            ser.rts = False
            
            print("Reset signal sent successfully")
            
            # リセット後に少し待つ
            time.sleep(2)
            
            # ブートメッセージを読み取って表示
            print("Waiting for boot messages...")
            boot_start_time = time.time()
            while time.time() - boot_start_time < 5:
                if ser.in_waiting > 0:
                    try:
                        data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                        if data:
                            print(data, end='')
                    except:
                        pass
                time.sleep(0.1)
            
            return True
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False

def list_serial_ports():
    """利用可能なシリアルポートを一覧表示"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            print("No serial ports found")
            return
            
        print("Available serial ports:")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port.device} - {port.description}")
            
    except ImportError:
        print("pyserial not installed. Install with: pip install pyserial")

def main():
    parser = argparse.ArgumentParser(description='Reset ESP32 via serial connection')
    parser.add_argument('port', nargs='?', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, 
                       help='Baud rate (default: 115200)')
    parser.add_argument('-t', '--reset-time', type=float, default=0.1,
                       help='Reset signal hold time in seconds (default: 0.1)')
    parser.add_argument('-l', '--list', action='store_true',
                       help='List available serial ports')
    
    args = parser.parse_args()
    
    if args.list:
        list_serial_ports()
        return
    
    if not args.port:
        print("Error: Serial port not specified")
        print("Use -l to list available ports")
        list_serial_ports()
        sys.exit(1)
    
    print(f"ESP32 Serial Reset Tool")
    print(f"Port: {args.port}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Reset time: {args.reset_time}s")
    print("-" * 40)
    
    success = reset_esp32(args.port, args.baudrate, args.reset_time)
    
    if success:
        print("\nESP32 reset completed successfully!")
        sys.exit(0)
    else:
        print("\nESP32 reset failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()