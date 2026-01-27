#!/usr/bin/env python3
"""
Simple ESP32 Reset Script
ESP32を簡単にリセットするスクリプト
"""

import serial
import time
import sys

def simple_reset(port="/dev/ttyUSB0"):
    """ESP32を簡単にリセット"""
    try:
        print(f"Resetting ESP32 on {port}...")
        
        with serial.Serial(port, 115200, timeout=1) as ser:
            # リセット信号送信
            ser.dtr = False
            ser.rts = True
            time.sleep(0.1)
            
            ser.dtr = False
            ser.rts = False
            
            print("Reset completed!")
            
            # ブートメッセージを少し表示
            time.sleep(1)
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print("Boot output:", data[:200] + "..." if len(data) > 200 else data)
                
        return True
        
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    print("ESP32 Simple Reset Tool")
    print("Usage: python3 esp32_simple_reset.py [port]")
    print("Example: python3 esp32_simple_reset.py /dev/ttyUSB0")
    print("-" * 50)
    
    if simple_reset(port):
        print("✓ Reset successful!")
    else:
        print("✗ Reset failed!")
        sys.exit(1)