#!/usr/bin/env python3
"""
test.py - Interactive Arduino Servo Control with Script File Support
"""

import time
import sys
import os
import threading
import argparse
from collections import deque
from datetime import datetime
from arm_board_controller import ArmBoardController

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.dates import DateFormatter
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    print("Warning: matplotlib not installed. Real-time plotting disabled.")
    print("Install with: pip install matplotlib")
    MATPLOTLIB_AVAILABLE = False

class RealTimePlotter:
    """INA219電流値のリアルタイムプロッター"""
    
    def __init__(self, controller, max_points=300, moving_avg_window=10):
        self.controller = controller
        self.max_points = max_points
        self.moving_avg_window = moving_avg_window
        self.times = deque(maxlen=max_points)
        self.currents = deque(maxlen=max_points)
        self.current_averages = deque(maxlen=max_points)
        self.is_running = False
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.line_current = None
        self.line_average = None
        self.ani = None
        
    def start_plotting(self):
        """グラフ表示を開始"""
        if not MATPLOTLIB_AVAILABLE:
            print("Matplotlib not available. Skipping real-time plot.")
            return False
            
        self.is_running = True
        
        # フィギュアとサブプロットを作成
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('INA219 Current Monitoring', fontsize=14)
        
        # 電流グラフ（生データ）
        self.ax1.set_title('Current - Raw Data (mA)')
        self.ax1.set_ylabel('Current [mA]')
        self.ax1.grid(True)
        self.line_current, = self.ax1.plot([], [], 'b-', linewidth=1, alpha=0.7, label='Raw')
        self.ax1.legend()
        
        # 電流グラフ（移動平均）
        self.ax2.set_title(f'Current - {self.moving_avg_window}-Point Moving Average (mA)')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Current [mA]')
        self.ax2.grid(True)
        self.line_average, = self.ax2.plot([], [], 'r-', linewidth=2, label=f'{self.moving_avg_window}-Point Avg')
        self.ax2.legend()
        
        # 時間軸フォーマット
        time_formatter = DateFormatter('%H:%M:%S')
        self.ax2.xaxis.set_major_formatter(time_formatter)
        
        plt.tight_layout()
        
        # アニメーション開始
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False
        )
        
        # 非ブロッキングでプロット表示
        plt.ion()
        plt.show()
        
        return True
    
    def calculate_moving_average(self):
        """電流値の移動平均を計算"""
        if len(self.currents) < self.moving_avg_window:
            return None
        
        # 最新のN個のデータの平均を計算
        recent_currents = list(self.currents)[-self.moving_avg_window:]
        return sum(recent_currents) / len(recent_currents)
    
    def update_plot(self, frame):
        """プロットを更新"""
        if not self.is_running:
            return self.line_current, self.line_average
            
        # センサーデータを取得
        data = self.controller.get_latest_sensor_data()
        if data:
            current_time = datetime.now()
            self.times.append(current_time)
            self.currents.append(data.current_mA)
            
            # 移動平均を計算
            moving_avg = self.calculate_moving_average()
            if moving_avg is not None:
                self.current_averages.append(moving_avg)
            
            # グラフを更新
            if len(self.times) > 1:
                self.line_current.set_data(self.times, self.currents)
                
                # 移動平均グラフ（データ数が十分な場合のみ）
                if len(self.current_averages) > 0:
                    avg_times = list(self.times)[-len(self.current_averages):]
                    self.line_average.set_data(avg_times, self.current_averages)
                
                # 軸の範囲を自動調整
                self.ax1.relim()
                self.ax1.autoscale_view()
                self.ax2.relim()
                self.ax2.autoscale_view()
                
                # 時間軸を回転
                self.fig.autofmt_xdate()
        
        return self.line_current, self.line_average
    
    def stop_plotting(self):
        """グラフ表示を停止"""
        self.is_running = False
        if self.ani:
            self.ani.event_source.stop()
        if self.fig:
            plt.close(self.fig)
            plt.ioff()

def print_help():
	"""ヘルプを表示"""
	print("\n=== Available Commands ===")
	print("s1 <angle>          - Set servo1 angle instantly (0-180)")
	print("s2 <angle>          - Set servo2 angle instantly (0-180)")
	print("s1s <angle> <speed> - Set servo1 angle with speed control")
	print("                      angle: 0-180 degrees")
	print("                      speed: 1-180 degrees/sec")
	print("s2s <angle> <speed> - Set servo2 angle with speed control")
	print("                      angle: 0-180 degrees") 
	print("                      speed: 1-180 degrees/sec")
	print("s1d                 - Detach servo1 (power off, relaxed state)")
	print("s2d                 - Detach servo2 (power off, relaxed state)")
	print("m <speed>           - Set motor speed (0-100%)")
	print("stop                - Emergency stop")
	print("status              - Show sensor data")
	print("voltage             - Show battery voltage")
	print("help                - Show this help")
	print("quit                - Exit program")
	print("file <filename>     - Load and execute commands from file")
	print("\nExample commands:")
	print("  s1 90             - Move servo1 to 90 degrees instantly")
	print("  s1s 180 30        - Move servo1 to 180 degrees at 30°/sec")
	print("  s2s 0 60          - Move servo2 to 0 degrees at 60°/sec")
	print("  m 50              - Set motor to 50% speed")
	print("  file commands.txt - Execute commands from file")
	print("===========================\n")

def load_commands_from_file(filename):
	"""テキストファイルからコマンドリストを読み込み"""
	try:
		with open(filename, 'r') as f:
			commands = []
			for line_num, line in enumerate(f, 1):
				line = line.strip()
				if line and not line.startswith('#'):  # 空行とコメント行を除く
					commands.append((line_num, line))
		return commands
	except FileNotFoundError:
		print(f"Error: File '{filename}' not found")
		return None
	except Exception as e:
		print(f"Error reading file: {e}")
		return None

def execute_command(controller, command):
	"""単一コマンドを実行"""
	command = command.strip().lower()
	
	if command.startswith('s1s '):
		# Servo1 with speed: s1s <angle> <speed>
		try:
			parts = command.split()
			if len(parts) >= 3:
				angle = int(parts[1])
				speed = int(parts[2])
				controller.set_servo1_angle_with_speed(angle, speed)
				print(f"Servo1: {angle}° at {speed}°/sec")
				return True
			else:
				print("Usage: s1s <angle> <speed>")
				return False
		except ValueError:
			print("Error: Invalid number format")
			return False
	elif command.startswith('s2s '):
		# Servo2 with speed: s2s <angle> <speed>
		try:
			parts = command.split()
			if len(parts) >= 3:
				angle = int(parts[1])
				speed = int(parts[2])
				controller.set_servo2_angle_with_speed(angle, speed)
				print(f"Servo2: {angle}° at {speed}°/sec")
				return True
			else:
				print("Usage: s2s <angle> <speed>")
				return False
		except ValueError:
			print("Error: Invalid number format")
			return False
	elif command.startswith('s1 '):
		# Servo1: s1 <angle>
		try:
			angle = int(command.split()[1])
			controller.set_servo1_angle(angle)
			print(f"Servo1: {angle}°")
			return True
		except (ValueError, IndexError):
			print("Usage: s1 <angle>")
			return False
	elif command.startswith('s2 '):
		# Servo2: s2 <angle>
		try:
			angle = int(command.split()[1])
			controller.set_servo2_angle(angle)
			print(f"Servo2: {angle}°")
			return True
		except (ValueError, IndexError):
			print("Usage: s2 <angle>")
			return False
	elif command.startswith('m '):
		# Motor: m <speed>
		try:
			speed = int(command.split()[1])
			controller.set_motor_speed(speed)
			print(f"Motor: {speed}%")
			return True
		except (ValueError, IndexError):
			print("Usage: m <speed>")
			return False
	elif command == 's1d':
		controller.detach_servo1()
		print("Servo1 detached (power off)")
		return True
	elif command == 's2d':
		controller.detach_servo2()
		print("Servo2 detached (power off)")
		return True
	elif command == 'stop':
		controller.emergency_stop()
		print("Emergency stop activated")
		return True
	elif command == 'status':
		data = controller.get_latest_sensor_data()
		if data:
			print(f"Battery: {data.battery_voltage:.2f}V, "
				  f"Current: {data.current_mA:.1f}mA")
		else:
			print("No sensor data available")
		return True
	elif command == 'voltage':
		data = controller.get_latest_sensor_data()
		if data:
			print(f"Battery Voltage: {data.battery_voltage:.2f}V")
		else:
			print("No sensor data available")
		return True
	else:
		print(f"Unknown command: '{command}'")
		return False

def run_file_mode(controller, commands):
	"""ファイルモードでコマンドを実行（リアルタイムグラフ付き）"""
	if not commands:
		return
	
	print(f"\n=== File Mode: {len(commands)} commands loaded ===")
	print("Press ENTER to execute next command, 'q' + ENTER to quit file mode")
	
	# リアルタイムプロッターを開始
	plotter = RealTimePlotter(controller)
	plot_started = plotter.start_plotting()
	
	if plot_started:
		print("\n📊 Real-time current/power monitoring started")
		print("Close the graph window to stop monitoring")
	else:
		print("\n⚠️  Real-time plotting not available")
	
	current_index = 0
	
	try:
		while True:
			# 現在のコマンドを表示
			line_num, command = commands[current_index]
			print(f"\n[{current_index + 1}/{len(commands)}] Line {line_num}: {command}")
			
			# センサーデータも表示
			data = controller.get_latest_sensor_data()
			if data:
				print(f"Current: {data.current_mA:.1f}mA, Battery: {data.battery_voltage:.2f}V")
			
			# ユーザー入力待ち
			user_input = input(">>> ").strip()
			
			if user_input.lower() == 'q':
				print("Exiting file mode")
				break
			elif user_input == '':
				# エンターが押された場合、コマンドを実行
				print(f"Executing: {command}")
				execute_command(controller, command)
				
				# 次のコマンドへ（最後まで行ったら最初に戻る）
				current_index = (current_index + 1) % len(commands)
				if current_index == 0:
					print("\n--- Reached end of file, looping back to start ---")
			else:
				print("Press ENTER to execute, or 'q' to quit file mode")
				
			# プロットウィンドウが閉じられたかチェック
			if plot_started and not plt.get_fignums():
				print("\n📊 Graph window closed - continuing without plot")
				plot_started = False
				
	finally:
		# プロッターを停止
		if plot_started:
			plotter.stop_plotting()
			print("\n📊 Real-time monitoring stopped")

def create_sample_file():
	"""サンプルファイルを作成"""
	sample_commands = [
		"# Sample command file for Arduino servo control",
		"s1 180",
		"s2s 0 10", 
		"s1 45",
		"m 50",
		"m 0",
		"s1 0"
	]
	
	filename = "sample_commands.txt"
	with open(filename, 'w') as f:
		for line in sample_commands:
			f.write(line + '\n')
	print(f"Sample file '{filename}' created")
	return filename

def main():
	# コマンドライン引数のパース
	parser = argparse.ArgumentParser(
		description='Arduino Servo Controller - Interactive Mode',
		formatter_class=argparse.RawDescriptionHelpFormatter,
		epilog='''
Examples:
  %(prog)s                    # Default port (/dev/ttyUSB0)
  %(prog)s -p /dev/ttyUSB1    # Specify port
  %(prog)s --port COM3        # Windows port example
		''')
	
	parser.add_argument('-p', '--port', 
						default='/dev/ttyUSB0',
						help='Serial port for Arduino connection (default: /dev/ttyUSB0)')
	
	args = parser.parse_args()
	
	print("Arduino Servo Controller - Interactive Mode")
	print(f"Using port: {args.port}")
	
	# Arduinoに接続
	controller = ArmBoardController(port=args.port)
	if not controller.connect():
		print(f"Failed to connect to Arduino on {args.port}")
		print("Available options:")
		print("  - Check if the device is connected")
		print("  - Try different port with -p option")
		print("  - Check permissions (may need sudo or add user to dialout group)")
		return
	
	print("Connected to Arduino!")
	print("Type 'help' for available commands, 'quit' to exit")
	
	try:
		while True:
			try:
				# コマンド入力
				command = input(">>> ").strip().lower()
				
				if command == 'quit':
					print("Exiting...")
					break
				elif command == 'help':
					print_help()
				elif command.startswith('file '):
					# ファイルモード
					filename = command.split(' ', 1)[1]
					commands = load_commands_from_file(filename)
					if commands:
						run_file_mode(controller, commands)
				elif command == 'sample':
					# サンプルファイル作成
					filename = create_sample_file()
					print(f"Use 'file {filename}' to execute sample commands")
				else:
					# 既存のコマンドハンドリング
					execute_command(controller, command)
				
			except KeyboardInterrupt:
				print("\nKeyboard interrupt detected. Type 'quit' to exit properly.")
			except Exception as e:
				print(f"Error: {e}")
	
	finally:
		# 安全停止とクリーンアップ
		print("Stopping all motors and disconnecting...")
		controller.emergency_stop()
		time.sleep(0.1)
		controller.disconnect()

if __name__ == "__main__":
	main()
