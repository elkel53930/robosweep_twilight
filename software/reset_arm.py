#!/usr/bin/env python3
"""
Arduino Nanoをリセット
"""

import serial
import time
import sys

def reset_arm(port="/dev/ttyARM"):
    """Arduino Nanoをリセット"""
    try:
        print(f"Resetting Arduino Nano on {port}...")
        
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