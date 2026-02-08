#!/usr/bin/env python3
"""迷路を解くスクリプト（スレッド版）

MobileBaseをスレッド化し、非同期処理を実現。
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

# パスを追加してインポート
sys.path.append('mob')
sys.path.append('search')

from mob.mobile_base_threaded import MobileBaseThread
from mob.esp32_reset import esp32_reset
from search.micromouse_algorithms import AdachiExplorer, Direction, INF
from rpi.camera_py.ball_detect_threaded import BallDetectThread
from arm.arm import Arm, ArmDummy, ArmBase
from dataclasses import dataclass


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

FWD_SPEED = 200
FWD_ACC = 1000

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
        
    print("=== 初期化 ===")
    mob_thread.send_command('GCAL')
    mob_thread.wait_response()
    
    mob_thread.send_command('RDST')
    mob_thread.wait_response()
    
    mob_thread.send_command('RANG')
    mob_thread.wait_response()

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


def initialize_arm() -> Arm:
    arm = Arm(futaba_port='/dev/ttyAMA0',
          arduino_port='/dev/ttyUSB0',
          arm_servo_id=1,
          arm_min_angle=-90.0,
          arm_max_angle=45.0)
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
        return [('FWD', FWD_SPEED, FWD_ACC, 180)]
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


def wait_for_done(robot: Robot, ball_thread: BallDetectThread | None = None,
                  max_wait_seconds: float = 10.0) -> bool:
    """DONE応答を待つ
    
    Args:
        robot: Robotインスタンス
        ball_thread: ボール検出スレッド（オプション）
        max_wait_seconds: 最大待機時間（秒）
    
    Returns:
        正常にDONEを受信したかどうか
    """
    start_time = time.time()
    
    print("#Waiting for DONE...")
    
    while (time.time() - start_time) < max_wait_seconds:
        try:
            response_type, data = robot.mob_thread.wait_response(block=False)
            
            if ball_thread is not None:
                detected, ball_info = ball_thread.is_ball_detected()
                if detected:
                    print(f"#ボール検出！{ball_info}")
            
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

    print(f"#Warning: DONE応答のタイムアウト ({max_wait_seconds}秒)")
    return False


def run_maze_exploration(robot: Robot, max_steps: int) -> str:
    """迷路探索のメインループを実行
    
    Args:
        robot: Robotインスタンス
        max_steps: 最大ステップ数
    
    Returns:
        探索結果を示す文字列:
        - 'GOAL_REACHED': ゴール到達
        - 'NO_PATH': ゴールへの道がない
        - 'SENSOR_ERROR': センサーエラー多発
        - 'MAX_STEPS': 最大ステップ数到達
    """
    print("\n=== 迷路探索開始 ===")
    
    robot.mob_thread.send_command('WALL', True)
    robot.mob_thread.wait_response()
    
    # 最初の前進
    robot.mob_thread.send_command('FWD', FWD_SPEED, FWD_ACC, 90)
    robot.mob_thread.wait_response()
    
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
        for i, cmd in enumerate(command_queue):
            print(f"#Sending command {i+1}/{len(command_queue)}: {cmd[0]}")
            robot.mob_thread.send_command(*cmd)
            
            # FWDコマンドの場合のみボール検出を有効にする
            ball_thread_ = robot.ball_thread if (cmd[0] == 'FWD' or cmd[0] == 'STOP') else None
            
            
            if not wait_for_done(robot, ball_thread_):
                print(f"#Warning: コマンド {i+1}/{len(command_queue)} の応答待機に失敗")
                break
        
        # 迷路マップ表示（10ステップごと）
        if step_count % 10 == 0:
            print("\n=== 現在の迷路マップ ===")
            maze_map = robot.explorer.render_text(show_goal=robot.explorer.goals[0] if robot.explorer.goals else None)
            print(maze_map)
    
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
    maze_map = robot.explorer.render_text(show_goal=robot.explorer.goals[0] if robot.explorer.goals else None)
    print(maze_map)


def main() -> int:
    ap = argparse.ArgumentParser(description='迷路を解くスクリプト（スレッド版）')
    ap.add_argument('--port', required=True, help='シリアルポート (例: /dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=3000000, help='ボーレート (デフォルト: 3000000)')
    ap.add_argument('--timeout', type=float, default=5.0, help='DONE待機タイムアウト秒 (デフォルト: 5.0)')
    ap.add_argument('--maze-size', type=int, default=16, help='迷路サイズ (デフォルト: 16)')
    ap.add_argument('--max-steps', type=int, default=1000, help='最大ステップ数 (デフォルト: 1000)')
    ap.add_argument('--goal-x', type=int, default=None, help='ゴールX座標 (デフォルト: 迷路中央)')
    ap.add_argument('--goal-y', type=int, default=None, help='ゴールY座標 (デフォルト: 迷路中央)')
    ap.add_argument('--use-dummy-arm', action='store_true', help='ダミーアームを使用')
    args = ap.parse_args()
    
    # シリアルポート接続
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    mob_thread = None
    ball_thread = None
    arm = None
    
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
            arm = initialize_arm()
        
        # ロボット初期化
        mob_thread = initialize_mobile_base(ser, timeout=args.timeout)
        
        # Robotインスタンスを作成
        robot = Robot(mob_thread=mob_thread, ball_thread=ball_thread, arm=arm, explorer=explorer)        

        # 一番最初の前進はアルゴリズムによらずに実行されるため、
        # その分をアルゴリズムに通知しておく
        robot.explorer.step_forward() 
        while True:
            # 迷路探索実行
            result = run_maze_exploration(robot, args.max_steps)
            
            # 探索結果を表示
            print(f"\n=== 探索結果: {result} ===")
            if result == 'GOAL_REACHED':
                print("ゴールに到達しました！")
            elif result == 'NO_PATH':
                print("ゴールへの道が見つかりませんでした。")
            elif result == 'SENSOR_ERROR':
                print("センサーエラーが多発したため中断しました。")
            elif result == 'MAX_STEPS':
                print(f"最大ステップ数 {args.max_steps} に到達しました。")
            else:
                print(f"不明な結果: {result}")
            
            # 最終結果表示
            show_final_results(robot)

            # 経過ステップ数が最も大きいマスを探す（到達不可能なマスは除外）
            import random
            max_steps_since_visit = -1
            least_visited_cells = []
            
            current_pos_x, current_pos_y = robot.explorer.get_current_position()
            dist_map = robot.explorer.get_distance_map(current_pos_x, current_pos_y)
            for y in range(args.maze_size):
                for x in range(args.maze_size):
                    # 到達不可能なマス（距離が無限大）は除外
                    if dist_map[y][x] >= INF:
                        continue
                    
                    steps = robot.explorer.get_steps_since_visit(x, y)
                    if steps > max_steps_since_visit:
                        max_steps_since_visit = steps
                        least_visited_cells = [(x, y)]
                    elif steps == max_steps_since_visit:
                        least_visited_cells.append((x, y))
            selected_cell = None
            if least_visited_cells:
                selected_cell = random.choice(least_visited_cells)
                print(f"\n=== 最も訪れていないマス ===")
                print(f"経過ステップ数: {max_steps_since_visit}")
                print(f"該当マス数: {len(least_visited_cells)}")
                print(f"選択されたマス: {selected_cell}")
            else:
                selected_cell = (0, 0)

            robot.explorer.set_goals([selected_cell])
            current_heading = robot.explorer.pose.heading
            next_heading = robot.explorer.decide_heading(None, None, None)
            # 到達不可能だとわかっているマスは選択されないので、必ず進行方向が得られる
            print(f"#Next heading: {next_heading.name}")
            action = relative_to_action(current_heading, next_heading)
            print(f"#Current heading: {current_heading.name}")
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
        if mob_thread:
            robot.mob_thread.send_command('WALL', False)
            robot.mob_thread.wait_response()
            mob_thread.stop()
            print("MobileBaseスレッド停止")
            
        if ball_thread:
            ball_thread.stop()
            print("ボール検出スレッド停止")
        
        if arm:
            print("\nStopping all motors and disconnecting...")
            arm.emergency_stop()
            time.sleep(0.1)
            arm.detach_servo_launcher()
            time.sleep(0.1)
            arm.set_servo_arm_torque(False)
            time.sleep(0.1)
            arm.disconnect()

            print("アーム切断")
        
        ser.close()
        # ESP32をリセット
        print("\n=== ESP32をリセットしています... ===")
        esp32_reset(args.port)


if __name__ == '__main__':
    raise SystemExit(main())
