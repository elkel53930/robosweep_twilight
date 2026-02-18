#!/usr/bin/env python3
"""迷路を解くスクリプト（スレッド版）

MobileBaseをスレッド化し、非同期処理を実現。
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from datetime import datetime
import builtins

from charset_normalizer import detect
import serial

# パスを追加してインポート
sys.path.append('mob')
sys.path.append('search')

from mob.mobile_base_threaded import MobileBaseThread
from mob.esp32_reset import esp32_reset
from search.micromouse_algorithms import AdachiExplorer, Direction, INF
from rpi.camera_py.ball_detect_threaded import BallDetectThread
from rpi.camera_py.interpolate_position import PositionInterpolator
from arm.arm import Arm, ArmDummy, ArmBase
from dataclasses import dataclass
from math import fabs
from enum import Enum
import arm.catch_throw as catch_throw
import math
import os

global machine_id
machine_id = None

def _install_timestamped_print() -> None:
    original_print = builtins.print

    def timestamped_print(*args, **kwargs):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        original_print(f"[{ts}]", *args, **kwargs)

    builtins.print = timestamped_print


_install_timestamped_print()

# 絶対パスを生成してcsvファイルを指定
# ファイルはこのスクリプトからみて./rpi/camera_py/calibration_data_with_mirror.csvに置いておく必要がある
dir_path = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(dir_path, 'rpi', 'camera_py', 'calibration_data_with_mirror.csv')
position_interpolator = PositionInterpolator(csv_path)

class State(Enum):
    SEARCH = 0 # ボールを持っていない状態でうろついている
    SEARCH_WITH_BALL = 1 # ボールを持っている状態で投擲位置を目指している    

@dataclass
class Robot:
    """ロボット全体の構造体"""
    mob_thread: MobileBaseThread
    ball_thread: BallDetectThread
    arm: ArmBase
    explorer: AdachiExplorer


# センサーしきい値
LS_THRESHOLD = 100  # 左前壁検出
RS_THRESHOLD = 100  # 右前壁検出
LF_RF_THRESHOLD = 50  # 左側・右側壁検出（両方がこの値以上で前壁あり）

FWD_SPEED = 300
FWD_ACC = 1000

#THROW_POSITION = [(0,7), (1,7), (2,7), (3,7), (4,7), (5,7), (6,7), (7,7),
#                  (8,7), (9,7), (10,7), (11,7), (12,7), (13,7), (14,7), (15,7)] # ボール投擲位置の候補（迷路の北側中央一列）
THROW_POSITION = [(2,2)]

def detect_walls(sensor_data: dict) -> tuple[bool, bool, bool]:
    """センサーデータから壁の有無を判定
    
    Args:
        sensor_data: センサーデータ辞書
    
    Returns:
        (left_wall, front_wall, right_wall) のタプル
    """
    lf = sensor_data['lf']
    rf = sensor_data['rf']
    ls = sensor_data['ls']
    rs = sensor_data['rs']
    
    left_wall = ls >= LS_THRESHOLD
    right_wall = rs >= RS_THRESHOLD
    # 前壁：左側と右側の両方が閾値以上
    front_wall = lf >= LF_RF_THRESHOLD and rf >= LF_RF_THRESHOLD
    
    print(f"#Wall detection: LF={lf} RF={rf} LS={ls} RS={rs}")
    print(f"#  Left={left_wall}, Front={front_wall}, Right={right_wall}")
    
    return left_wall, front_wall, right_wall


def relative_to_action(current_heading: Direction, next_heading: Direction) -> str:
    """現在の向きと次の向きから必要なアクションを決定
    
    Returns:
        'fwd', 'left', 'right', 'back' のいずれか
    """
    diff = (int(next_heading) - int(current_heading)) % 4
    
    if diff == 0:
        return 'fwd'
    elif diff == 1:  # 右に90度
        return 'right'
    elif diff == 3:  # 左に90度
        return 'left'
    elif diff == 2:  # 180度
        return 'back'
    else:
        raise ValueError(f"Invalid direction difference: {diff}")


def initialize_explorer(maze_size: int, goals: list[tuple[int, int]]) -> AdachiExplorer:
    """迷路探索アルゴリズムを初期化
    
    Args:
        maze_size: 迷路のサイズ
        goals: ゴール座標のリスト
    
    Returns:
        初期化されたAdachiExplorer
    """
    explorer = AdachiExplorer(maze_size=maze_size, goals=goals)
    
    # 外周の壁を設定
    for x in range(maze_size):
        explorer.mark_wall(x, 0, Direction.SOUTH, True)
        explorer.mark_wall(x, maze_size - 1, Direction.NORTH, True)
    
    for y in range(maze_size):
        explorer.mark_wall(0, y, Direction.WEST, True)
        explorer.mark_wall(maze_size - 1, y, Direction.EAST, True)
    
    # スタート位置(0,0)の右側（EAST）の壁を設定
    explorer.mark_wall(0, 0, Direction.EAST, True)
    
    print(f"#初期壁情報: 外周の壁と(0,0)の右壁を設定")
    
    # ロボスイープ特有の壁
#    for x in range(maze_size):
#        explorer.mark_wall(x, 7, Direction.NORTH, True) # 相手チームとの境界線
#
#    # スタート位置が４箇所ある
#    explorer.mark_wall(15, 0, Direction.WEST, True)
#    explorer.mark_wall(0, 7, Direction.EAST, True)
#    explorer.mark_wall(15, 7, Direction.WEST, True)

    print("迷路初期化完了")    
    print(f"\n{explorer.render_text(show_goal=goals, show_distance=True)}")
        
    return explorer


def initialize_mobile_base(ser: serial.Serial, timeout: float) -> MobileBaseThread:
    """ロボットの初期化コマンドを実行
    
    Args:
        ser: シリアルポート
        timeout: タイムアウト時間 [秒]
    """
    # MobileBaseスレッドを初期化・開始
    mob_thread = MobileBaseThread(ser, timeout=timeout)
    mob_thread.start()
    print("MobileBaseスレッド開始")

    return mob_thread


def initialize_ball_detector() -> BallDetectThread:
    """ボール検出スレッドを初期化・開始
    
    Returns:
        初期化されたBallDetectThread
    """
    from picamera2 import Picamera2
    from rpi.camera_py.ball_detect import BallDetect
    
    print("=== カメラ初期化 ===")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (640, 360), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    
    detector = BallDetect(debug=False)
    ball_thread = BallDetectThread(picam2, detector)
    ball_thread.start()
    print("ボール検出スレッド開始")
    
    return ball_thread


def initialize_arm_dummy() -> ArmDummy:
    """ダミーアームを初期化"""
    arm = ArmDummy()
    print("ダミーアーム初期化")
    return arm


def initialize_arm(arduino_port: str = '/dev/ttyARM') -> Arm:
    global machine_id
    arm = Arm(futaba_port='/dev/ttyAMA0',
          arduino_port=arduino_port,
          arm_servo_id=1,
          arm_min_angle=-90.0,
          arm_max_angle=45.0,
          machine_id=machine_id)
    if not arm.connect_arduino():
        raise RuntimeError("Arduinoへの接続に失敗しました")
    
    arm.set_motor_speed(10)
    arm.set_servo_arm_torque(True)
    arm.set_servo_arm_angle(Arm.RUN_POSITION, 500)
    time.sleep(0.5)
    arm.set_motor_speed(0)
    
    return arm


def get_sensor_data_with_retry(robot: Robot, max_retries: int = 3) -> tuple[dict | None, bool]:
    """センサーデータを取得（リトライ付き）
    
    Args:
        robot: Robotインスタンス
        max_retries: 最大リトライ回数
    
    Returns:
        (sensor_data, success) のタプル
    """
    for retry in range(max_retries):
        robot.mob_thread.send_command('SEN')
        response_type, sensor_data = robot.mob_thread.wait_response(timeout=3.0)
        
        if response_type == 'DONE' and sensor_data is not None:
            return sensor_data, True
        elif response_type == 'ERROR':
            print(f"#センサーデータ取得エラー（試行 {retry+1}/{max_retries}）: {sensor_data}")
            time.sleep(0.2)
        else:
            print(f"#センサーデータ取得失敗（試行 {retry+1}/{max_retries}）")
            time.sleep(0.1)
    
    return None, False


def generate_action(robot: Robot, action: str) -> list[tuple]:
    """アクションに対応するコマンドリストを生成
    
    Args:
        robot: Robotインスタンス
        action: 'fwd', 'left', 'right', 'back' のいずれか
    
    Returns:
        コマンドのリスト [(command, args...), ...]
        各要素は send_command() の引数タプル
    """
    if action == 'fwd':
        # 次のマスへ前進（180mm）
        return [('FWD', FWD_SPEED, FWD_ACC, 90),
                ('FWD', FWD_SPEED, FWD_ACC, 90)]
    elif action == 'left':
        # 停止 → 左旋回 → 前進
        return [
            ('STOP', FWD_SPEED, FWD_ACC, 90),
            ('TURN', 1.5708),  # pi/2
            ('FWD', FWD_SPEED, FWD_ACC, 90)
        ]
    elif action == 'right':
        # 停止 → 右旋回 → 前進
        return [
            ('STOP', FWD_SPEED, FWD_ACC, 90),
            ('TURN', -1.5708),  # -pi/2
            ('FWD', FWD_SPEED, FWD_ACC, 90)
        ]
    elif action == 'back':
        # 停止 → 180度旋回 → 前進
        return [
            ('STOP', FWD_SPEED, FWD_ACC, 90),
            ('TURN', 3.14159),  # pi
            ('FWD', FWD_SPEED, FWD_ACC, 90)
        ]
    else:
        raise ValueError(f"Unknown action: {action}")


def wait_for_done(robot: Robot, max_wait_seconds: float = 10.0) -> bool:
    """DONE応答を待つ
    
    Args:
        robot: Robotインスタンス
        max_wait_seconds: 最大待機時間（秒）
    
    Returns:
        正常にDONEを受信したかどうか
    """
    start_time = time.time()
    
    print("#Waiting for DONE...")
    
    while (time.time() - start_time) < max_wait_seconds:
        try:
            response_type, data = robot.mob_thread.wait_response(block=False)
            
            if response_type == 'DONE':
                print("RX: DONE(from queue)")
                return True
            elif response_type == 'QSTPDONE':
                print(f"RX: QSTPDONE, 残り距離={data:.2f}mm")
                return True
            elif response_type == 'TIMEOUT':
                # まだ応答がない、継続
                pass
            elif response_type == 'ERROR':
                print(f"RX: ERROR - {data}")
                return False
                
        except Exception as e:
            print(f"#応答待機エラー: {e}")
            return False
    
        time.sleep(0.01)

    print(f"#Warning: ❕DONE応答のタイムアウト ({max_wait_seconds}秒)")
    return False

def catch_ball(robot: Robot, state: State) -> bool:
    detected = False
    ball_info = None
    moved_angle = 0.0
    moved_distance = 0.0
    
    if state == State.SEARCH_WITH_BALL:
        print("#すでに持っているボールを落とします")
        robot.arm.set_servo_launcher_angle(Arm.LAUNCHER_RELOAD)
        time.sleep(1)
    
    def return_original_position():
        # 移動分戻る
        if moved_distance > 0:
            robot.mob_thread.send_command('JOGBACK', moved_distance)
        else:
            robot.mob_thread.send_command('JOGFWD', -moved_distance)
        robot.mob_thread.wait_response()
        time.sleep(0.5)
        robot.mob_thread.send_command('TURN', -moved_angle)
        robot.mob_thread.wait_response()
        reset_sensors(robot)
        print(f"#位置を元に戻しました: 角度={-moved_angle:.4f} rad, 距離={-moved_distance:.2f} mm")
    
    def detect() -> tuple[bool, dict]:
        for i in range(10):
            d, info = robot.ball_thread.is_ball_detected()
            if d:
                break
            print(f"#ボールなし (試行 {i+1}/10)")
            robot.ball_thread.wait_detection_update()
        if not d:
            print("#ボールが見つかりません")
            robot.ball_thread.save_next_frame()
            time.sleep(3)
        return d, info
    
    # ボールを再度確認
    # 正面を向く
    repeat = 10
    detect_results= []
    time.sleep(1)
    for _ in range(repeat):
        detected, ball_info = detect()
        if detected:
            detect_results.append(ball_info)
        else: # detect()のなかで10回トライしたのに見つけられなかった
            print("#ボールが見つかりませんでした")
            reset_sensors(robot)
            return False
        time.sleep(0.1)
    
    if len(detect_results) < 7:
        print("#ボールが十分な回数見つかりませんでした")
        reset_sensors(robot)
        return False

    xs = [info['center'][0] for info in detect_results]
    ys = [info['center'][1] for info in detect_results]
    # ソートして外れ値を除去
    xs.sort()
    ys.sort()
    xs = xs[2:-2]
    ys = ys[2:-2]
    avg_x = sum(xs) / len(xs)
    avg_y = sum(ys) / len(ys)
    print(f"#ボール位置の平均: ({avg_x:.2f}, {avg_y:.2f})")

    angle, distance = position_interpolator.interpolate(avg_x, avg_y, True)    
    # angleの単位はπラジアン -> ラジアンに変換
    angle *= math.pi
    # ボールから170mm手前に移動するため、距離を減算
    distance -= 170
    
    print(f"angle={angle:.4f} rad")
    if fabs(angle) > 0.01:
        moved_angle += angle
        robot.mob_thread.send_command('TURN', angle)
        robot.mob_thread.wait_response()
    
    print(f"distance={distance:.2f} mm")
    if fabs(distance) > 2:
        moved_distance += distance
        if distance < 0:
            robot.mob_thread.send_command('JOGBACK', -distance)    
        else:
            robot.mob_thread.send_command('JOGFWD', distance)
        robot.mob_thread.wait_response()
        time.sleep(0.5)
    
    print(f"#Moved angle total: {moved_angle:.4f} rad, distance total: {moved_distance:.2f} mm")
    # 位置調整完了
 
    catch_result = catch_throw.catch(robot.arm, timeout=3)   
    if not catch_result:
        print("#❌ボールキャッチ失敗")
        return_original_position()
        return False
    
    time.sleep(2.0)

    return_original_position()    

    return True # キャッチ成功


def reset_sensors(robot: Robot) -> None:
    print("=== センサーリセット ===")
    robot.mob_thread.send_command('GCAL')
    robot.mob_thread.wait_response()
    
    robot.mob_thread.send_command('RDST')
    robot.mob_thread.wait_response()
    
    robot.mob_thread.send_command('RANG')
    robot.mob_thread.wait_response()
    
def catch_ball_when_detect(robot: Robot, last_command: tuple, state: State, front_wall_check: bool=True) -> str:
    """
    戻り値
    - 'CATCHED': ボールキャッチ成功
    - 'BALL_LOST': ボールを失った（SEARCH_WITH_BALL状態でキャッチ失敗）
    - 'CATCH_FAILED': ボールキャッチ失敗（SEARCH状態でキャッチ失敗）
    - 'NOT_DETECTED': ボール未検出
    """
    
    print("#Command done")
    # コマンドの間にボール検出
    detected, ball_info = robot.ball_thread.is_ball_detected()
    robot.ball_thread.save_next_frame()
    if detected:
        print(f"#ボール検出{ball_info}")
        
        # 眼の前に壁がないか確認
        sensor_data, _ = get_sensor_data_with_retry(robot)
        _, is_front_wall, _ = detect_walls(sensor_data)
        # 前壁チェックなし, または前壁チェックありかつ前壁がない場合
        if (not front_wall_check) or (front_wall_check and not is_front_wall):
            # 前壁チェックなし かつ 前壁がない
            if last_command[0] == 'FWD':
                # 前進中にボールを検出したら停止して位置を戻す。
                robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 60)
                robot.mob_thread.wait_response()
                robot.mob_thread.send_command('JOGBACK', 65)
                robot.mob_thread.wait_response()
            else:
                time.sleep(0.5)
            
            if catch_ball(robot, state):
                if state == State.SEARCH_WITH_BALL:
                    print("#ボールを捨ててボールキャッチ成功")
                    return "NEW_BALL_CATCHED"
                elif state == State.SEARCH:
                    print("#ボールキャッチ成功")
                    return "CATCHED"
            else:
                if state == State.SEARCH_WITH_BALL:
                    print("#新しいボールのキャッチに失敗")
                    return "BALL_LOST"
                elif state == State.SEARCH:
                    print("#ボールキャッチ失敗")
                    return "CATCH_FAILED"
        else:
            print("#前壁があるためボールキャッチをスキップ")
            
        reset_sensors(robot)
        return "NOT_DETECTED"
    

def update_maze_map(robot: Robot) -> None:
    sensor_data, _ = get_sensor_data_with_retry(robot)
    left_wall, front_wall, right_wall = detect_walls(sensor_data)
    robot.explorer.update_walls_from_relative(left_wall, front_wall, right_wall) # 壁を更新


def run_maze_exploration(robot: Robot, max_steps: int, state: State) -> str:
    """迷路探索のメインループを実行
    
    Args:
        robot: Robotインスタンス
        max_steps: 最大ステップ数
    
    Returns:
        探索結果を示す文字列:
        - 'GOAL_REACHED': ゴール到達
        - 'CATCHTED': ボールキャッチ成功
        - 'NO_PATH': ゴールへの道がない
        - 'SENSOR_ERROR': センサーエラー多発
        - 'MAX_STEPS': 最大ステップ数到達
        - 'BALL_LOST': ボールを失った（SEARCH_WITH_BALL状態でキャッチ失敗）
    """
    
    # TODO : ボールを持って走行中にボールを発見したケース
    
    reset_sensors(robot)
    
    print("\n=== 迷路探索開始 ===")
    
    robot.mob_thread.send_command('WALL', True)
    robot.mob_thread.wait_response()
    
    # 最初の前進
    cmd = ('FWD', FWD_SPEED, FWD_ACC, 90) # 最初の前進コマンド
    robot.mob_thread.send_command(*cmd)
    wait_for_done(robot)
    ball_result = catch_ball_when_detect(robot, cmd, state, front_wall_check=False)
    if ball_result == "CATCHED":
        print("\n=== ボールキャッチ成功 ===")
        update_maze_map(robot) # 壁情報を更新
        robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
        robot.mob_thread.wait_response()
        return 'CATCHTED'
    elif ball_result == "CATCH_FAILED":
        print("\n=== ボールキャッチ失敗 ===")
        update_maze_map(robot) # 壁情報を更新
        robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
        robot.mob_thread.wait_response()
        return 'CATCH_FAILED'
    elif ball_result == "NEW_BALL_CATCHED" or ball_result == "NOT_DETECTED":
        pass # 何もせず続行
    elif ball_result == "BALL_LOST":
        print("\n=== ボールを失った ===")
        update_maze_map(robot) # 壁情報を更新
        robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
        robot.mob_thread.wait_response()
        return 'BALL_LOST'
    
    step_count = 0
    consecutive_sensor_failures = 0
    
    while step_count < max_steps:
        step_count += 1
        print(f"\n--- Step {step_count} ---")
        
        # センサーデータ取得
        sensor_data, success = get_sensor_data_with_retry(robot)
        
        if not success:
            consecutive_sensor_failures += 1
            print(f"#センサーデータ取得失敗（連続{consecutive_sensor_failures}回）")
            
            if consecutive_sensor_failures >= 5:
                print("\n=== センサー取得エラーが多発しています。中断します ===")
                robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
                robot.mob_thread.wait_response(timeout=1.0)
                return 'SENSOR_ERROR'
            
            time.sleep(0.5)
            continue
        
        consecutive_sensor_failures = 0
        
        # 壁検出
        left_wall, front_wall, right_wall = detect_walls(sensor_data)
        
        # 現在の位置と向き
        current_x, current_y = robot.explorer.pose.x, robot.explorer.pose.y
        current_heading = robot.explorer.pose.heading
        print(f"#Current pose: ({current_x}, {current_y}) heading={current_heading.name}")
        
        # ゴール判定
        if (current_x, current_y) in robot.explorer.goals:
            robot.explorer.update_walls_from_relative(left_wall, front_wall, right_wall)
            print(f"\n=== ゴール到達！ ({current_x}, {current_y}) ===")
            robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
            robot.mob_thread.wait_response()
            return 'GOAL_REACHED'
        
        # 次の進行方向を決定
        next_heading = robot.explorer.decide_heading(left_wall, front_wall, right_wall)
        
        if next_heading is None:
            print(f"\n=== ゴールまでの道がありません！ ===")
            print(f"現在位置 ({current_x}, {current_y}) からゴールに到達できません")
            robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
            robot.mob_thread.wait_response()
            return 'NO_PATH'
        
        print(f"#Next heading: {next_heading.name}")
        
        # アクション決定
        action = relative_to_action(current_heading, next_heading)
        print(f"#Action: {action}")
        
        # アクションに対応するコマンドリストを取得
        command_queue = generate_action(robot, action)
        print(f"#Command queue: {len(command_queue)} commands")
        
        # コマンドキューを1つずつ実行
        # 各コマンドを送信→DONE待ち→次のコマンド送信
        ball_result = None
        for i, cmd in enumerate(command_queue):
            print(f"#Sending command {i+1}/{len(command_queue)}: {cmd[0]}")
            last_command = cmd
            robot.mob_thread.send_command(*cmd)
            if not wait_for_done(robot):
                print(f"#Warning: コマンド {i+1}/{len(command_queue)} の応答待機に失敗")
                return "EXIT"
            if ball_result != "CATCHED": # すでにボールキャッチ成功している場合は以降のコマンド実行後もボール検出をスキップ
                ball_result = catch_ball_when_detect(robot,last_command, state, front_wall_check=last_command[0]=='STOP')
        
        if state == State.SEARCH and ball_result == "CATCHED":
            print("\n=== 🟡ボールキャッチ成功 ===")
            sensor_data, _ = get_sensor_data_with_retry(robot)
            left_wall, front_wall, right_wall = detect_walls(sensor_data)
            robot.explorer.update_walls_from_relative(left_wall, front_wall, right_wall) # 壁を更新
            robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
            robot.mob_thread.wait_response()
            return 'CATCHTED'
        elif state == State.SEARCH_WITH_BALL and ball_result == "BALL_LOST":
            print("\n=== ❌ボールを失った ===")
            robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
            robot.mob_thread.wait_response()
            return 'BALL_LOST'

        # 迷路マップ表示（10ステップごと）
        if step_count % 10 == 0:
            print("\n=== 現在の迷路マップ ===")
            maze_map = robot.explorer.render_text(show_goal=robot.explorer.goals[0] if robot.explorer.goals else None, show_distance=True)
            print(f"\n{maze_map}")
    
    if step_count >= max_steps:
        print(f"\n=== 最大ステップ数 {max_steps} に到達しました ===")
        robot.mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
        robot.mob_thread.wait_response()
        return 'MAX_STEPS'
    
    # 通常はここには到達しないが、念のため
    return 'UNKNOWN'


def show_final_results(robot: Robot) -> None:
    """最終結果を表示
    
    Args:
        robot: Robotインスタンス
    """
    
    robot.mob_thread.send_command('SEN')
    response_type, sensor_data = robot.mob_thread.wait_response(timeout=2.0)
    if response_type == 'DONE' and sensor_data:
        print(f"\n=== 最終位置 ===")
        print(f"Distance: {sensor_data['odo_dist']:.2f} mm")
        print(f"Angle: {sensor_data['odo_ang']:.4f} rad ({sensor_data['odo_ang'] * 180 / 3.14159:.2f} deg)")
    
    print("\n=== 最終迷路マップ ===")
    maze_map = robot.explorer.render_text(show_goal=robot.explorer.goals[0] if robot.explorer.goals else None, show_distance=True)
    print(f"\n{maze_map}")


def choice_next_goal(explorer: AdachiExplorer) -> list[tuple[int, int]]:
    max_steps_since_visit = -1
    least_visited_cells = []

    current_pos_x, current_pos_y = explorer.get_current_position()
    dist_map = explorer.get_distance_map(current_pos_x, current_pos_y)
    for y in range(explorer.size):
        for x in range(explorer.size):
            # 到達不可能なマス（距離が無限大）は除外
            if dist_map[y][x] >= INF:
                continue
            
            steps = explorer.get_steps_since_visit(x, y)
            if steps > max_steps_since_visit:
                max_steps_since_visit = steps
                least_visited_cells = [(x, y)]
            elif steps == max_steps_since_visit:
                least_visited_cells.append((x, y))
    if least_visited_cells:
        next_goals = [random.choice(least_visited_cells)]
        print(f"\n=== 最も訪れていないマス ===")
        print(f"経過ステップ数: {max_steps_since_visit}")
        print(f"該当マス数: {len(least_visited_cells)}")
        print(f"選択されたマス: {next_goals}")
    else:
        next_goals = [(0, 0)]
    return next_goals

def show_title() -> None:
    import pyfiglet
    # 赤い文字でTWILIGHTと表示
    global machine_id
    figlet = pyfiglet.figlet_format(f"{machine_id}", font="banner3")
    print(f"\n\033[31mTWILIGHT\033[0m\n{figlet}")
    time.sleep(1)

def main() -> int:
    ap = argparse.ArgumentParser(description='迷路を解くスクリプト（スレッド版）')
    ap.add_argument('--mob-port', default='/dev/ttyMOB', help='MOBシリアルポート (デフォルト: /dev/ttyMOB)')
    ap.add_argument('--arm-port', default='/dev/ttyARM', help='ARMシリアルポート (デフォルト: /dev/ttyARM)')
    ap.add_argument('--baud', type=int, default=3000000, help='ボーレート (デフォルト: 3000000)')
    ap.add_argument('--timeout', type=float, default=5.0, help='DONE待機タイムアウト秒 (デフォルト: 5.0)')
    ap.add_argument('--maze-size', type=int, default=16, help='迷路サイズ (デフォルト: 16)')
    ap.add_argument('--max-steps', type=int, default=1000, help='最大ステップ数 (デフォルト: 1000)')
    ap.add_argument('--goal-x', type=int, default=None, help='ゴールX座標 (デフォルト: 迷路中央)')
    ap.add_argument('--goal-y', type=int, default=None, help='ゴールY座標 (デフォルト: 迷路中央)')
    ap.add_argument('--use-dummy-arm', action='store_true', help='ダミーアームを使用')
    ap.add_argument('--machine-id', type=int, default=1, help='機体番号')
    args = ap.parse_args()
    
    global machine_id
    machine_id = args.machine_id
    
    show_title()
    
    # シリアルポート接続
    ser = serial.Serial(port=args.mob_port, baudrate=args.baud, timeout=0.1)
    mob_thread = None
    ball_thread = None
    arm = None
    state = State.SEARCH
    
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # ゴール位置の設定
        if args.goal_x is not None and args.goal_y is not None:
            goals = [(args.goal_x, args.goal_y)]
            print(f"=== ゴール位置: ({args.goal_x}, {args.goal_y}) ===")
        else:
            c = args.maze_size // 2
            goals = [(c - 1, c - 1), (c, c - 1), (c - 1, c), (c, c)]
            print(f"=== ゴール位置: 中央4マス {goals} ===")
        
        # 探索アルゴリズムを初期化
        explorer = initialize_explorer(args.maze_size, goals)
        
        # ボール検出スレッド初期化
        ball_thread = initialize_ball_detector()
        
        # アーム初期化
        if args.use_dummy_arm:
            arm = initialize_arm_dummy()
        else:
            arm = initialize_arm(args.arm_port)
        
        # ロボット初期化
        mob_thread = initialize_mobile_base(ser, timeout=args.timeout)
        
        # Robotインスタンスを作成
        robot = Robot(mob_thread=mob_thread, ball_thread=ball_thread, arm=arm, explorer=explorer)        

        # 一番最初の前進はアルゴリズムによらずに実行されるため、
        # その分をアルゴリズムに通知しておく
        robot.explorer.step_forward() 
        while True:
            # 迷路探索実行
            result = run_maze_exploration(robot, args.max_steps, state)
            
            # 探索結果を表示
            print(f"\n=== 探索結果: {result} ===")
            if result == 'GOAL_REACHED':
                print("ゴールに到達しました！")
            elif result == 'CATCHTED':
                print("ボールをキャッチしました！")
            elif result == 'NO_PATH':
                print("ゴールへの道が見つかりませんでした。")
            elif result == 'SENSOR_ERROR':
                print("センサーエラーが多発したため中断しました。")
            elif result == 'MAX_STEPS':
                print(f"最大ステップ数 {args.max_steps} に到達しました。")
            elif result == 'BALL_LOST':
                print("ボールを失いました。")
            elif result == 'EXIT':
                print("探索を終了します。")
                return 0
            else:
                print(f"不明な結果: {result}")
            
            # 最終結果表示
            show_final_results(robot)

            # 経過ステップ数が最も大きいマスを探す（到達不可能なマスは除外）
            next_goals = []
            
            if state == State.SEARCH:
                if result == 'CATCHTED':
                    print(f"\n=== ボール投擲位置へ移動 ===")
                    print(f"ボール投擲位置: {THROW_POSITION}")
                    next_goals = THROW_POSITION
                    state = State.SEARCH_WITH_BALL
                else:
                    # GOAL_REACHED, NO_PATH, SENSOR_ERROR, MAX_STEPSのいずれの場合も、次は訪れていないマスを優先して探索する
                    next_goals = choice_next_goal(robot.explorer)
            elif state == State.SEARCH_WITH_BALL:
                if result == 'GOAL_REACHED':
                    # 投擲位置に到着
                    print(f"\n=== ボール投擲位置に到着 ===")
                    # 北を向き、ボールを投げる
                    current_heading = robot.explorer.pose.heading
                    action = relative_to_action(current_heading, Direction.NORTH)
                    print(f"#Current heading: {current_heading.name}")
                    print(f"#Turning to NORTH for throwing. Action: {action}")
                    if action == "fwd":
                        pass
                    elif action == "left":
                        robot.mob_thread.send_command('TURN', 1.5708)  # pi/2
                        wait_for_done(robot)
                    elif action == "right":
                        robot.mob_thread.send_command('TURN', -1.5708)  # -pi/2
                        wait_for_done(robot)
                    elif action == "back":
                        robot.mob_thread.send_command('TURN', 3.14159)  # pi
                        wait_for_done(robot)
                    # ボール投げ
                    catch_throw.throw(robot.arm) 
                    robot.explorer.pose.heading = Direction.NORTH # 北向き
                    state = State.SEARCH # 投擲後は再び探索状態に戻る
                    next_goals = choice_next_goal(robot.explorer)
                elif result == 'BALL_LOST':
                    print(f"\n=== ボールを失いました。探索を続行します ===")
                    state = State.SEARCH
                    next_goals = choice_next_goal(robot.explorer)
                elif result in ['SENSOR_ERROR', 'MAX_STEPS']:
                    print(f"\n=== 状態遷移: {result} → SEARCH ===")
                    state = State.SEARCH
                    next_goals = choice_next_goal(robot.explorer)
                else:
                    # CATCHED, NO_PATHが発生
                    print("❌❌❌❌❌❌❌❌❌❌")
                    print(f"予期しない状態遷移です。探索を続行します。state={state}, result={result}")
                    print("Launcher servoをリロード位置に戻します")
                    robot.arm.set_servo_launcher_angle(Arm.LAUNCHER_RELOAD)
                    print("❌❌❌❌❌❌❌❌❌❌")
                    next_goals = choice_next_goal(robot.explorer)
                    
            reset_sensors(robot)

            # 新たなゴールを設定して探索再開
            robot.explorer.set_goals(next_goals)
            current_heading = robot.explorer.pose.heading
            next_heading = robot.explorer.decide_heading(None, None, None)
            # 到達不可能だとわかっているマスは選択されないので、必ず進行方向が得られる
            print(f"#Next heading: {next_heading.name}")
            print(f"#Current heading: {current_heading.name}")
            action = relative_to_action(current_heading, next_heading)
            print(f"#Next action: {action}")
            
            if action == "fwd":
                pass # run_maze_explorationで前進を行う
            elif action == "left":
                robot.mob_thread.send_command('TURN', 1.5708)  # pi/2
                wait_for_done(robot)
            elif action == "right":
                robot.mob_thread.send_command('TURN', -1.5708)  # -pi/2
                wait_for_done(robot)
            elif action == "back":
                robot.mob_thread.send_command('TURN', 3.14159)  # pi
                wait_for_done(robot)

        
    except KeyboardInterrupt:
        print("\n\n=== 中断されました ===")
        if mob_thread:
            mob_thread.send_command('STOP', FWD_SPEED, FWD_ACC, 90)
            mob_thread.wait_response(timeout=1.0)
        return 1
    except Exception as e:
        print(f"\n\n=== エラー発生 ===")
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # スレッド停止
        # ESP32をリセット
        if mob_thread:
            mob_thread.stop()
            print("MobileBaseスレッド停止")

        print("\n=== ESP32をリセットしています... ===")
        esp32_reset(args.mob_port)
            
        if ball_thread:
            ball_thread.stop()
            print("ボール検出スレッド停止")
        
        if arm:
            print("\nStopping all motors and disconnecting...")
            arm.set_servo_launcher_angle(Arm.LAUNCHER_RELOAD)
            time.sleep(0.5)
            arm.emergency_stop()
            arm.detach_servo_launcher()
            time.sleep(0.1)
            arm.set_servo_arm_torque(False)
            time.sleep(0.1)
            arm.disconnect()

            print("アーム切断")
        
        ser.close()


if __name__ == '__main__':
    raise SystemExit(main())
