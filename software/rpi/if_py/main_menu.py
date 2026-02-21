#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
メインメニュースクリプト
スタートアップ時に起動されるメインメニュー
"""

from typing import Optional
import argparse
import shlex
import time
import subprocess
import socket
import os
import signal
import sys

from text_display import TextDisplay
from direction_buttons import DirectionButtons, Direction
from menu_selector import MenuSelector, MenuItem
from config import create_display_device

SOFTWARE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if SOFTWARE_ROOT not in sys.path:
    sys.path.append(SOFTWARE_ROOT)

from reset_arm import reset_arm
from mob.esp32_reset import esp32_reset
from arm.arm import Arm

# タイムアウト設定
NETWORK_TIMEOUT = 3
COMMAND_TIMEOUT = 2
STATUS_UPDATE_INTERVAL = 1.0
BUTTON_POLL_INTERVAL = 0.01


def get_network_status() -> str:
    """ネットワーク接続状況を取得
    
    Returns:
        str: "Connected" または "Disconnected"
    """
    try:
        # Google DNSサーバーへの接続確認
        socket.create_connection(("8.8.8.8", 53), timeout=NETWORK_TIMEOUT)
        return "Connected"
    except OSError:
        return "Disconnected"


def get_ip_address() -> str:
    """IPアドレスを取得
    
    Returns:
        str: IPアドレス、取得できない場合は "N/A"
    """
    try:
        result = subprocess.run(
            ["hostname", "-I"],
            capture_output=True,
            text=True,
            timeout=COMMAND_TIMEOUT
        )
        ip = result.stdout.strip().split()[0] if result.stdout.strip() else "N/A"
        return ip
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, IndexError):
        return "N/A"


def get_power_voltage() -> str:
    """電源状態を取得
    
    vcgencmd get_throttledコマンドを使用して電源状態を確認
    
    Returns:
        str: 電源状態 ("OK", "Low voltage!", "OK (was low)", "N/A")
    """
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=COMMAND_TIMEOUT
        )
        if result.stdout:
            # "throttled=0x0" 形式から16進数値を抽出
            throttled_hex = result.stdout.strip().split("=")[1]
            throttled_value = int(throttled_hex, 16)
            
            # ビット0: 現在低電圧状態
            # ビット16: 過去に低電圧発生
            if throttled_value & 0x1:
                return "Low voltage!"
            elif throttled_value & 0x10000:
                return "OK (was low)"
            else:
                return "OK"
        return "N/A"
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, ValueError, IndexError):
        return "N/A"


def get_current_time() -> str:
    """現在時刻を取得
    
    Returns:
        str: フォーマットされた現在時刻
    """
    return time.strftime("%Y-%m-%d %H:%M:%S")


def show_rpi_status(device, buttons: DirectionButtons) -> None:
    """RPi statusを表示（1秒毎に更新、ボタンで終了）
    
    Args:
        device: ディスプレイデバイス
        buttons: ボタン入力インスタンス
    """
    display = TextDisplay(device, font_size=8, text_color="white", bg_color="black")
    
    try:
        while True:
            # ステータス情報を取得
            status_lines = [
                f"Network: {get_network_status()}",
                f"IP: {get_ip_address()}",
                f"Power: {get_power_voltage()}",
                get_current_time()
            ]
            
            # 画面に表示
            display.display_multi_line(status_lines, y_start=2, line_spacing=6)
            
            # ボタンチェック（ノンブロッキング）
            start_time = time.time()
            while time.time() - start_time < STATUS_UPDATE_INTERVAL:
                if any([
                    buttons.is_pressed(Direction.UP),
                    buttons.is_pressed(Direction.DOWN),
                    buttons.is_pressed(Direction.LEFT),
                    buttons.is_pressed(Direction.RIGHT)
                ]):
                    return
                time.sleep(BUTTON_POLL_INTERVAL)
    
    except KeyboardInterrupt:
        pass


def action_rpi_status(device, buttons: DirectionButtons) -> None:
    """RPi statusアクション
    
    Args:
        device: ディスプレイデバイス
        buttons: ボタン入力インスタンス
    """
    show_rpi_status(device, buttons)
    return None


def action_power_off_confirm(device) -> str:
    """Power off確認後の実行
    
    Args:
        device: ディスプレイデバイス
        
    Returns:
        str: "exit"
    """
    display = TextDisplay(device, font_size=10, text_color="white", bg_color="black")
    display.display_text("Shutting down...", align="center")
    time.sleep(1)
    
    # シャットダウンコマンドを実行
    try:
        subprocess.run(["sudo", "poweroff"], check=True)
    except subprocess.CalledProcessError as e:
        # エラーが発生した場合は表示
        display.display_text(f"Error: {e}", align="center")
        time.sleep(2)
    
    return "exit"


def create_power_off_menu() -> list[MenuItem]:
    """Power offの確認メニューを作成
    
    Returns:
        list[MenuItem]: No/Yesメニュー項目のリスト
    """
    return [
        MenuItem("No", action=lambda: None),
        MenuItem("Yes", action=lambda: "power_off_yes"),
    ]


def create_exit_menu() -> list[MenuItem]:
    """Exitの確認メニューを作成
    
    Returns:
        list[MenuItem]: No/Yesメニュー項目のリスト
    """
    return [
        MenuItem("No", action=lambda: None),
        MenuItem("Yes", action=lambda: "exit"),
    ]


def _normalize_exec_command(exec_args: Optional[list[str]]) -> Optional[list[str]]:
    if not exec_args:
        return None
    if len(exec_args) == 1:
        return shlex.split(exec_args[0])
    return exec_args


def _show_message(device, message: str, duration: float = 1.0) -> None:
    display = TextDisplay(device, font_size=10, text_color="white", bg_color="black")
    display.display_text(message, align="center")
    time.sleep(duration)


def action_reset(device) -> None:
    """ESP32とArduino Nanoをリセット"""
    _show_message(device, "Resetting...", duration=0.5)
    esp_ok = esp32_reset()
    arm_ok = reset_arm()
    if esp_ok and arm_ok:
        _show_message(device, "Reset OK", duration=1.0)
    else:
        _show_message(device, "Reset failed", duration=1.5)
        
    arm = Arm(machine_id=1) # トルクを切るだけなので機体IDは関係ないが、Armクラスの仕様上必要
    arm.set_servo_arm_torque(False)
    return None


def main() -> None:
    """メインメニューを実行"""
    try:
        parser = argparse.ArgumentParser(description="Main menu")
        parser.add_argument(
            "--exec",
            nargs=argparse.REMAINDER,
            help="Command to run when selecting RUN",
        )
        args = parser.parse_args()
        exec_cmd = _normalize_exec_command(args.exec)

        # ディスプレイデバイスの作成
        device = create_display_device()

        run_process: Optional[subprocess.Popen] = None
        run_menu_item = MenuItem("RUN")

        def _set_run_label(is_running: bool) -> None:
            run_menu_item.label = "SHUTDOWN" if is_running else "RUN"
            menu._render_menu()

        def action_run_or_shutdown():
            nonlocal run_process

            if run_process is not None and run_process.poll() is not None:
                run_process = None

            if run_process is None:
                if not exec_cmd:
                    _show_message(device, "--exec not set", duration=1.5)
                    return None
                try:
                    run_process = subprocess.Popen(exec_cmd, start_new_session=True)
                    _set_run_label(True)
                    _show_message(device, "Running...", duration=0.5)
                except Exception as e:
                    _show_message(device, f"Error: {e}", duration=2.0)
                    run_process = None
                    _set_run_label(False)
            else:
                try:
                    os.killpg(run_process.pid, signal.SIGTERM)
                    run_process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(run_process.pid, signal.SIGKILL)
                    run_process.wait(timeout=3.0)
                finally:
                    run_process = None
                    _set_run_label(False)
                    _show_message(device, "Stopped", duration=0.5)
            return None
        
        # メニューセレクターを実行
        with MenuSelector(device, [], font_size=10) as menu:
            # メインメニューを定義（buttonsを参照できるようにここで定義）
            menu_items = [
                MenuItem("RUN", action=action_run_or_shutdown),
                MenuItem("Reset", action=lambda: action_reset(device)),
                MenuItem("RPi status", action=lambda: action_rpi_status(device, menu.buttons)),
                MenuItem("Power off", submenu=create_power_off_menu()),
                MenuItem("Exit", submenu=create_exit_menu()),
            ]
            run_menu_item = menu_items[0]
            menu.current_menu = menu_items
            
            # メインループ
            while True:
                result = menu.run()
                
                # Power off確認
                if result == "power_off_yes":
                    action_power_off_confirm(device)
                    break
                
                # Exit確認
                if result == "exit":
                    break
                
                # メニューに戻る
                menu.running = True
                menu._render_menu()
    
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
