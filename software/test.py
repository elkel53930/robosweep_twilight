#!/usr/bin/env python3
"""
test.py - Interactive Arduino Servo Control with Script File Support
"""

import time
import sys
import os
from arm_board_controller import ArmBoardController

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
	print("m <speed>           - Set motor speed (0-100%)")
	print("stop                - Emergency stop")
	print("status              - Show sensor data")
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
	elif command == 'stop':
		controller.emergency_stop()
		print("Emergency stop activated")
		return True
	elif command == 'status':
		data = controller.get_latest_sensor_data()
		if data:
			print(f"Battery: {data.battery_voltage:.2f}V, "
				  f"Current: {data.current_mA:.1f}mA, "
				  f"Power: {data.power_mW:.1f}mW")
		else:
			print("No sensor data available")
		return True
	else:
		print(f"Unknown command: '{command}'")
		return False

def run_file_mode(controller, commands):
	"""ファイルモードでコマンドを実行"""
	if not commands:
		return
	
	print(f"\n=== File Mode: {len(commands)} commands loaded ===")
	print("Press ENTER to execute next command, 'q' + ENTER to quit file mode")
	
	current_index = 0
	
	while True:
		# 現在のコマンドを表示
		line_num, command = commands[current_index]
		print(f"\n[{current_index + 1}/{len(commands)}] Line {line_num}: {command}")
		
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
	print("Arduino Servo Controller - Interactive Mode")
	
	# Arduinoに接続
	controller = ArmBoardController()
	if not controller.connect():
		print("Failed to connect to Arduino")
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
