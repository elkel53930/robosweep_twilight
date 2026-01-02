#!/usr/bin/env python3
"""
arm_board_test.py - Arduino Nano アームボード制御のテストスクリプト
"""

import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
from arm_board_controller import ArmBoardController, SensorData


class ArmBoardTester:
    """アームボードのテスト用クラス"""
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.controller = ArmBoardController(port)
        self.sensor_history = deque(maxlen=100)  # 最大100個のデータを保持
        self.is_plotting = False
    
    def basic_control_test(self):
        """基本的な制御テスト"""
        print("=== Basic Control Test ===")
        
        if not self.controller.connect():
            print("Failed to connect to Arduino")
            return
        
        try:
            # サーボテスト
            print("Testing Servo 1...")
            angles = [0, 45, 90, 135, 180, 90]
            for angle in angles:
                print(f"Setting Servo 1 to {angle} degrees")
                self.controller.set_servo1_angle(angle)
                time.sleep(1)
            
            print("Testing Servo 2...")
            for angle in angles:
                print(f"Setting Servo 2 to {angle} degrees")
                self.controller.set_servo2_angle(angle)
                time.sleep(1)
            
            # モーターテスト
            print("Testing Motor...")
            speeds = [0, 25, 50, 75, 100, 50, 0]
            for speed in speeds:
                print(f"Setting Motor speed to {speed}%")
                self.controller.set_motor_speed(speed)
                time.sleep(2)
            
            # センサーデータ表示
            print("Reading sensor data for 5 seconds...")
            start_time = time.time()
            while time.time() - start_time < 5:
                data = self.controller.get_latest_sensor_data()
                if data:
                    print(f"Battery: {data.battery_voltage:.2f}V, "
                          f"Current: {data.current_mA:.1f}mA, "
                          f"Power: {data.power_mW:.1f}mW")
                time.sleep(0.5)
            
        except KeyboardInterrupt:
            print("Test interrupted")
        finally:
            self.controller.emergency_stop()
            self.controller.disconnect()
    
    def sensor_data_callback(self, data: SensorData):
        """センサーデータ受信コールバック"""
        self.sensor_history.append(data)
    
    def realtime_plot_test(self):
        """リアルタイムプロット付きテスト"""
        print("=== Real-time Plot Test ===")
        
        if not self.controller.connect():
            print("Failed to connect to Arduino")
            return
        
        # コールバック設定
        self.controller.set_sensor_callback(self.sensor_data_callback)
        
        # プロット設定
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
        fig.suptitle('Arduino Arm Board - Real-time Sensor Data')
        
        # データ格納用
        times = deque(maxlen=100)
        voltages = deque(maxlen=100)
        currents = deque(maxlen=100)
        powers = deque(maxlen=100)
        
        def update_plot(frame):
            if self.sensor_history:
                # 最新データを取得
                latest_data = list(self.sensor_history)
                
                # データを整理
                times.clear()
                voltages.clear()
                currents.clear()
                powers.clear()
                
                start_time = latest_data[0].timestamp if latest_data else time.time()
                for data in latest_data:
                    times.append(data.timestamp - start_time)
                    voltages.append(data.battery_voltage)
                    currents.append(data.current_mA)
                    powers.append(data.power_mW)
                
                # プロット更新
                ax1.clear()
                ax1.plot(times, voltages, 'b-', label='Battery Voltage')
                ax1.set_ylabel('Voltage [V]')
                ax1.legend()
                ax1.grid(True)
                
                ax2.clear()
                ax2.plot(times, currents, 'r-', label='Current')
                ax2.set_ylabel('Current [mA]')
                ax2.legend()
                ax2.grid(True)
                
                ax3.clear()
                ax3.plot(times, powers, 'g-', label='Power')
                ax3.set_xlabel('Time [s]')
                ax3.set_ylabel('Power [mW]')
                ax3.legend()
                ax3.grid(True)
        
        try:
            self.is_plotting = True
            
            # バックグラウンドで制御を実行
            def control_sequence():
                time.sleep(2)  # プロット開始待ち
                
                print("Starting control sequence...")
                
                # サーボとモーターの組み合わせテスト
                test_sequence = [
                    (90, 90, 20),   # (servo1, servo2, motor_speed)
                    (45, 135, 40),
                    (135, 45, 60),
                    (0, 180, 30),
                    (180, 0, 50),
                    (90, 90, 0),
                ]
                
                for servo1, servo2, motor_speed in test_sequence:
                    if not self.is_plotting:
                        break
                    
                    print(f"Setting: Servo1={servo1}°, Servo2={servo2}°, Motor={motor_speed}%")
                    self.controller.set_servo1_angle(servo1)
                    time.sleep(0.2)
                    self.controller.set_servo2_angle(servo2)
                    time.sleep(0.2)
                    self.controller.set_motor_speed(motor_speed)
                    
                    time.sleep(5)  # 各設定で5秒間維持
                
                print("Control sequence completed")
            
            # 制御スレッド開始
            control_thread = threading.Thread(target=control_sequence, daemon=True)
            control_thread.start()
            
            # アニメーション開始
            ani = FuncAnimation(fig, update_plot, interval=200, cache_frame_data=False)
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("Plot test interrupted")
        finally:
            self.is_plotting = False
            self.controller.emergency_stop()
            self.controller.disconnect()
    
    def interactive_control(self):
        """インタラクティブ制御テスト"""
        print("=== Interactive Control Test ===")
        print("Commands:")
        print("  s1 <angle>  - Set servo 1 angle (0-180)")
        print("  s2 <angle>  - Set servo 2 angle (0-180)")
        print("  m <speed>   - Set motor speed (0-100)")
        print("  stop        - Emergency stop")
        print("  status      - Request status")
        print("  quit        - Exit")
        
        if not self.controller.connect():
            print("Failed to connect to Arduino")
            return
        
        self.controller.set_sensor_callback(self.sensor_data_callback)
        
        try:
            while True:
                command = input(">>> ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'stop':
                    self.controller.emergency_stop()
                elif command == 'status':
                    self.controller.request_status()
                    data = self.controller.get_latest_sensor_data()
                    if data:
                        print(f"Latest data: Battery={data.battery_voltage:.2f}V, "
                              f"Current={data.current_mA:.1f}mA, Power={data.power_mW:.1f}mW")
                elif command.startswith('s1 '):
                    try:
                        angle = int(command.split()[1])
                        self.controller.set_servo1_angle(angle)
                    except (ValueError, IndexError):
                        print("Usage: s1 <angle>")
                elif command.startswith('s2 '):
                    try:
                        angle = int(command.split()[1])
                        self.controller.set_servo2_angle(angle)
                    except (ValueError, IndexError):
                        print("Usage: s2 <angle>")
                elif command.startswith('m '):
                    try:
                        speed = int(command.split()[1])
                        self.controller.set_motor_speed(speed)
                    except (ValueError, IndexError):
                        print("Usage: m <speed>")
                else:
                    print("Unknown command")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("Interactive control interrupted")
        finally:
            self.controller.emergency_stop()
            self.controller.disconnect()


if __name__ == "__main__":
    import sys
    
    tester = ArmBoardTester()
    
    if len(sys.argv) > 1:
        test_mode = sys.argv[1]
    else:
        print("Select test mode:")
        print("1. Basic control test")
        print("2. Real-time plot test")
        print("3. Interactive control")
        choice = input("Enter choice (1-3): ")
        test_mode = {'1': 'basic', '2': 'plot', '3': 'interactive'}.get(choice, 'basic')
    
    try:
        if test_mode == 'basic':
            tester.basic_control_test()
        elif test_mode == 'plot':
            tester.realtime_plot_test()
        elif test_mode == 'interactive':
            tester.interactive_control()
        else:
            print("Invalid test mode")
    
    except Exception as e:
        print(f"Test error: {e}")