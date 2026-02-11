#!/usr/bin/env python3
"""
catch_throw.py - ボールをキャッチして投擲するテストスクリプト

統合アームコントローラー(Arm)を使用して、以下のシーケンスを実行：
1. アームを初期位置へ移動
2. ランチャーサーボを初期位置へセット
3. モーターで吸引開始
4. ランチャーサーボをリロード
5. アームをボールに向ける
6. 電流監視でボールキャッチを確認
7. アームを走行位置へ移動
8. 吸引停止
9. アームを投擲位置へ移動
10. ランチャーサーボ開放で投擲
"""

import time
from collections import deque
from arm.arm import Arm

MOTOR_SPEED = 35

def calculate_current_average(arm, duration=0.5, sample_interval=0.03):
    """
    指定期間の電流平均値を計算
    
    Args:
        arm: Armインスタンス
        duration: 測定時間 [秒]
        sample_interval: サンプリング間隔 [秒]
        
    Returns:
        平均電流値 [mA]
    """
    print(f"Measuring current for {duration} seconds...")
    
    current_samples = []
    start_time = time.time()
    
    while time.time() - start_time < duration:
        data = arm.get_latest_sensor_data()
        if data:
            print(f"Current reading: {data.current_mA:.1f}mA")
            current_samples.append(data.current_mA)
        time.sleep(sample_interval)
    
    if current_samples:
        # ソートして、外れ値を除去
        current_samples.sort()
        trimmed_samples = current_samples[2:-2]  # 上下2つずつ除去
        
        avg_current = sum(trimmed_samples) / len(trimmed_samples)
        print(f"Average current over {duration}s: {avg_current:.1f}mA")
        return avg_current
    else:
        print("Warning: No current data collected")
        return 0.0

def wait_for_current_drop(arm, reference_current, threshold_percentage=95, check_interval=0.03, timeout=3):
    """
    電流が基準値の指定パーセント未満になるまで待機
    
    Args:
        arm: Armインスタンス
        reference_current: 基準電流値 [mA]
        threshold_percentage: しきい値パーセンテージ
        check_interval: チェック間隔 [秒]
        timeout: タイムアウト時間 [秒]
        
    Returns:
        成功したらTrue、タイムアウトでFalse
    """
    threshold_current = reference_current * (threshold_percentage / 100.0)
    print(f"Waiting for 10-point moving average current to drop below {threshold_current:.1f}mA ({threshold_percentage}% of {reference_current:.1f}mA)")
    
    # 移動平均計算用のデータ格納
    current_buffer = deque(maxlen=10)
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        data = arm.get_latest_sensor_data()
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

def catch(arm: Arm, timeout: int=3) -> bool:
    # ランチャーサーボを初期位置へセット（リロードギアを初期位置へ移動）
    print(f"\n[2] Setting launcher servo to {Arm.LAUNCHER_RELOAD}°")
    arm.set_servo_launcher_angle(Arm.LAUNCHER_RELOAD)
    time.sleep(0.1)
    
    # モーターで吸引開始
    print("\n[3] Starting motor (suction)")
    for i in range(10):
        arm.set_motor_speed(int((i+1) * (MOTOR_SPEED / 10)))
        time.sleep(0.1)
    
    # ランチャーサーボをリロード（スマッシャーをリロード）
    print(f"\n[4] Reloading launcher servo to {Arm.LAUNCHER_READY}°")
    arm.set_servo_launcher_angle(Arm.LAUNCHER_READY)
    time.sleep(2)  # 回転速度が上がるまで待機
    
    # スリープ後、1秒間の電流の平均値を記録
    print("\n[5] Measuring average current over 1 second")
    reference_current = calculate_current_average(arm, duration=1.0)
    
    # アームをボールに向ける
    print(f"\n[6] Moving arm to CATCH position ({Arm.CATCH_POSITION}°)")
    arm.set_servo_arm_angle(Arm.CATCH_POSITION, move_time=400)
    
    # 電流の移動平均を監視してボールキャッチを確認
    print("\n[7] Waiting for ball catch (current drop detection)")
    success = wait_for_current_drop(arm, reference_current, threshold_percentage=97, timeout=timeout)
    
    if not success:
        print("Warning: Ball catch not confirmed, aborting...")
        arm.set_motor_speed(0)
        arm.set_servo_arm_angle(Arm.RUN_POSITION, move_time=500)
        arm.set_servo_launcher_angle(Arm.LAUNCHER_RELOAD)
        time.sleep(3)
        return False
    
    # アームを走行位置へ移動
    print(f"\n[8] Moving arm to RUN position ({Arm.RUN_POSITION}°)")
    arm.set_servo_arm_angle(Arm.RUN_POSITION, move_time=1200)
    time.sleep(0.5)
    
    # 吸引停止
    print("\n[9] Stopping motor (suction off)")
    arm.set_motor_speed(0)

    # 終了
    print("\n✅Catch sequence completed successfully!")
    return True
        

def throw(arm: Arm) -> None:
        # アームを投擲位置へ移動
        print(f"\n[10] Moving arm to THROW position ({Arm.THROW_POSITION}°)")
        arm.set_servo_arm_angle(Arm.THROW_POSITION, move_time=500)
        time.sleep(0.6)
        
        # ランチャーサーボ開放で投擲（バネ開放）
        print(f"\n[11] Firing! Setting launcher servo to {Arm.LAUNCHER_FIRE}°")
        arm.set_servo_launcher_angle(Arm.LAUNCHER_FIRE)
        time.sleep(0.5)
        
        # アームを走行位置へ戻す
        print(f"\n[12] Returning arm to RUN position ({Arm.RUN_POSITION}°)")
        arm.set_servo_arm_angle(Arm.RUN_POSITION, move_time=400)
        time.sleep(0.5)



def main():
    print("=== Catch and Throw Test Sequence ===")
    
    # Arm初期化
    arm = Arm(futaba_port='/dev/ttyAMA0',
              arduino_port='/dev/ttyARM',
              arm_servo_id=1,
              arm_min_angle=-90.0,
              arm_max_angle=45.0)
    
    # Arduino接続
    if not arm.connect_arduino():
        print("Failed to connect to Arduino")
        return
    
    print("Connected to all devices!")

    try:
        print("\nEnabling arm servo torque...")
        # アームサーボのトルクON
        arm.set_servo_arm_torque(True)
        time.sleep(0.5)
        print("\n--- Starting Test Sequence ---")
        arm.set_servo_arm_angle(Arm.RUN_POSITION, move_time=800)
        result = catch(arm)
        time.sleep(1.5)
        print(f"result = {result}")
        if result:
            print("\n--- Proceeding to Throw ---")
            throw(arm)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 安全停止
        print("\nStopping all motors and disconnecting...")
        arm.emergency_stop()
        time.sleep(0.1)
        arm.detach_servo_launcher()
        time.sleep(0.1)
        arm.set_servo_arm_torque(False)
        time.sleep(0.1)
        arm.disconnect()
if __name__ == "__main__":
    main()
