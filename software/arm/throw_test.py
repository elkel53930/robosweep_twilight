#!/usr/bin/env python3
"""
throw_test.py - 指定されたシーケンスでサーボとモーターを制御するテストスクリプト
"""

import time
from collections import deque
from arm_board_controller import ArmBoardController
from futaba_servo import FutabaServo

def calculate_current_average(controller, duration=0.5, sample_interval=0.03):
    """指定期間の電流平均値を計算"""
    print(f"Measuring current for {duration} seconds...")
    
    current_samples = []
    start_time = time.time()
    
    while time.time() - start_time < duration:
        data = controller.get_latest_sensor_data()
        if data:
            print(f"Current reading: {data.current_mA:.1f}mA")
            current_samples.append(data.current_mA)
        time.sleep(sample_interval)
    
    if current_samples:
        avg_current = sum(current_samples) / len(current_samples)
        print(f"Average current over {duration}s: {avg_current:.1f}mA")
        return avg_current
    else:
        print("Warning: No current data collected")
        return 0.0

def wait_for_current_drop(controller, reference_current, threshold_percentage=95, check_interval=0.03, timeout=5):
    """電流が基準値の指定パーセント未満になるまで待機"""
    threshold_current = reference_current * (threshold_percentage / 100.0)
    print(f"Waiting for 10-point moving average current to drop below {threshold_current:.1f}mA ({threshold_percentage}% of {reference_current:.1f}mA)")
    
    # 移動平均計算用のデータ格納
    current_buffer = deque(maxlen=10)
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        data = controller.get_latest_sensor_data()
        if data:
            current_buffer.append(data.current_mA)
            print(f"Current reading: {data.current_mA:.1f}mA")
            
            # 10ポイントの移動平均を計算
            if len(current_buffer) >= 10:
                moving_avg = sum(current_buffer) / len(current_buffer)
                print(f"Current 10-point avg: {moving_avg:.1f}mA (target: <{threshold_current:.1f}mA)")
                
                if moving_avg < threshold_current:
                    print(f"✓ Current dropped below threshold! Moving average: {moving_avg:.1f}mA")
                    return True
        
        time.sleep(check_interval)
    
    print(f"✗ Timeout: Current did not drop below threshold within {timeout} seconds")
    return False

CATCH_POSITION = 43
THROW_POSITION = -45
RUN_POSITION = -90

def main():
    print("=== Throw Test Sequence ===")
    
    # Arduino接続
    controller = ArmBoardController()
    futaba = FutabaServo()
    servo_id = 1
    
    futaba.set_torque(servo_id, True)
    time.sleep(0.5)
    
    if not controller.connect():
        print("Failed to connect to Arduino")
        return
    
    print("Connected to Arduino!")
    
    try:
        print("\n--- Starting Test Sequence ---")
        
		# (アームを初期位置へ移動)
        futaba.set_position(servo_id, RUN_POSITION, move_time=400)
        time.sleep(0.3)  # コマンド処理待ち
        
        # Servo2の角度を180度に設定
		# (リロードギアを初期位置へ移動)
        print("\nSetting Servo2 to 180°")
        controller.set_servo1_angle(180)
        time.sleep(0.1)  # コマンド処理待ち
                
        # モーターの速度を50%に設定
		# (吸引開始)
        print("\nSetting motor speed to 50%")
        for i in range(5):
            controller.set_motor_speed(i*10)
            time.sleep(0.1) 

        # Servo2の角度を35度に設定
		# (スマッシャーをリロード)
        print("\nSetting Servo2 to 30°")
        controller.set_servo1_angle(30)
        
        # 回転速度が上がるまでスリープ
        time.sleep(0.5)
        
        # スリープ後、1秒間の電流の平均値を覚えておく
        print("\nMeasuring average current over 1 second")
        reference_current = calculate_current_average(controller)
        
		# (アームをボールに向ける)
        print("\nSetting Servo1 to 165° at speed 200")
        futaba.set_position(servo_id, CATCH_POSITION, move_time=400)
        time.sleep(0.5)  # コマンド処理待ち
        
        # 電流の移動平均を監視してしきい値未満で次へ
		# (ボールを掴んだことを確認)
        print("\nWaiting for current to drop below 95% of reference")
        success = wait_for_current_drop(controller, reference_current, threshold_percentage=95, timeout=10)
        
        if not success:
            print("Warning: Continuing despite timeout...")
            controller.set_motor_speed(0)
            futaba.set_position(servo_id, THROW_POSITION, move_time=500)
            controller.set_servo1_angle(180)
            time.sleep(3)
            raise Exception("Test aborted due to current threshold timeout")
        
		# (アームを走行位置へ移動)
        futaba.set_position(servo_id, RUN_POSITION, move_time=1200)
        time.sleep(0.5)  # コマンド処理待ち
        
        # モーターの速度を0%に設定
		# (吸引停止)
        print("\nSetting motor speed to 0%")
        controller.set_motor_speed(0)
        
        # 2秒スリープ
        time.sleep(2)  # 1.5s + コマンド処理待ち0.5s = 2s
        
        # 投擲位置へアームを移動
        futaba.set_position(servo_id, THROW_POSITION, move_time=500)
        time.sleep(0.6)
        
        # Servo2を0度に設定
		# (バネ開放で投擲)
        print("\nSetting Servo2 to 0°")
        controller.set_servo1_angle(0)
        time.sleep(0.5)  # コマンド処理待ち
        
        # 終了
        print("\nTest sequence completed!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError during test: {e}")
    finally:
        # 安全停止
        print("\nStopping all motors and disconnecting...")
        controller.emergency_stop()
        time.sleep(0.1)
        controller.detach_servo1()
        futaba.set_torque(servo_id, False)
        time.sleep(0.1)
        controller.disconnect()
        print("Test completed.")

if __name__ == "__main__":
    main()
