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
from search.micromouse_algorithms import AdachiExplorer, Direction


# センサーしきい値
LS_THRESHOLD = 100  # 左前壁検出
RS_THRESHOLD = 100  # 右前壁検出
LF_RF_THRESHOLD = 50  # 左側・右側壁検出（両方がこの値以上で前壁あり）


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


def initialize_robot(mob_thread: MobileBaseThread) -> None:
    """ロボットの初期化コマンドを実行
    
    Args:
        mob_thread: MobileBaseスレッド
    """
    print("=== 初期化 ===")
    mob_thread.send_command('GCAL')
    mob_thread.wait_response()
    
    mob_thread.send_command('RDST')
    mob_thread.wait_response()
    
    mob_thread.send_command('RANG')
    mob_thread.wait_response()
    
    mob_thread.send_command('WALL', True)
    mob_thread.wait_response()


def get_sensor_data_with_retry(mob_thread: MobileBaseThread, max_retries: int = 3) -> tuple[dict | None, bool]:
    """センサーデータを取得（リトライ付き）
    
    Args:
        mob_thread: MobileBaseスレッド
        max_retries: 最大リトライ回数
    
    Returns:
        (sensor_data, success) のタプル
    """
    for retry in range(max_retries):
        mob_thread.send_command('SEN')
        response_type, sensor_data = mob_thread.wait_response(timeout=3.0)
        
        if response_type == 'DONE' and sensor_data is not None:
            return sensor_data, True
        elif response_type == 'ERROR':
            print(f"#センサーデータ取得エラー（試行 {retry+1}/{max_retries}）: {sensor_data}")
            time.sleep(0.2)
        else:
            print(f"#センサーデータ取得失敗（試行 {retry+1}/{max_retries}）")
            time.sleep(0.1)
    
    return None, False


def execute_action(mob_thread: MobileBaseThread, action: str) -> None:
    """アクションを実行
    
    Args:
        mob_thread: MobileBaseスレッド
        action: 'fwd', 'left', 'right', 'back' のいずれか
    """
    if action == 'fwd':
        # 次のマスへ前進（180mm）
        mob_thread.send_command('FWD', 300, 1200, 180)
    elif action == 'left':
        # 停止 → 左旋回 → 前進
        mob_thread.send_command('STOP', 300, 1200, 90)
        mob_thread.wait_response()
        mob_thread.send_command('TURN', 1.5708)  # pi/2
        mob_thread.wait_response()
        mob_thread.send_command('FWD', 300, 1200, 90)
    elif action == 'right':
        # 停止 → 右旋回 → 前進
        mob_thread.send_command('STOP', 300, 1200, 90)
        mob_thread.wait_response()
        mob_thread.send_command('TURN', -1.5708)  # -pi/2
        mob_thread.wait_response()
        mob_thread.send_command('FWD', 300, 1200, 90)
    elif action == 'back':
        # 停止 → 180度旋回 → 前進
        mob_thread.send_command('STOP', 300, 1200, 90)
        mob_thread.wait_response()
        mob_thread.send_command('TURN', 3.14159)  # pi
        mob_thread.wait_response()
        mob_thread.send_command('FWD', 300, 1200, 90)


def wait_for_done(mob_thread: MobileBaseThread, max_wait_cycles: int = 100) -> bool:
    """DONE応答を待つ
    
    Args:
        mob_thread: MobileBaseスレッド
        max_wait_cycles: 最大待機サイクル
    
    Returns:
        正常にDONEを受信したかどうか
    """
    wait_cycles = 0
    
    while wait_cycles < max_wait_cycles:
        wait_cycles += 1
        
        try:
            response_type, data = mob_thread.wait_response(timeout=0.1)
            
            if response_type == 'DONE':
                print("RX: DONE")
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
    
    print(f"#Warning: DONE応答のタイムアウト")
    return False


def run_maze_exploration(mob_thread: MobileBaseThread, explorer: AdachiExplorer, 
                         max_steps: int) -> None:
    """迷路探索のメインループを実行
    
    Args:
        mob_thread: MobileBaseスレッド
        explorer: AdachiExplorer
        max_steps: 最大ステップ数
    """
    print("\n=== 迷路探索開始 ===")
    
    # 最初の前進
    mob_thread.send_command('FWD', 300, 1200, 90)
    mob_thread.wait_response()
    explorer.step_forward()
    
    step_count = 0
    consecutive_sensor_failures = 0
    
    while step_count < max_steps:
        step_count += 1
        print(f"\n--- Step {step_count} ---")
        
        # センサーデータ取得
        sensor_data, success = get_sensor_data_with_retry(mob_thread)
        
        if not success:
            consecutive_sensor_failures += 1
            print(f"#センサーデータ取得失敗（連続{consecutive_sensor_failures}回）")
            
            if consecutive_sensor_failures >= 5:
                print("\n=== センサー取得エラーが多発しています。中断します ===")
                mob_thread.send_command('STOP', 300, 1200, 90)
                mob_thread.wait_response(timeout=1.0)
                break
            
            time.sleep(0.5)
            continue
        
        consecutive_sensor_failures = 0
        
        # 壁検出
        left_wall, front_wall, right_wall = detect_walls(sensor_data)
        
        # 現在の位置と向き
        current_x, current_y = explorer.pose.x, explorer.pose.y
        current_heading = explorer.pose.heading
        print(f"#Current pose: ({current_x}, {current_y}) heading={current_heading.name}")
        
        # ゴール判定
        if (current_x, current_y) in explorer.goals:
            explorer.update_walls_from_relative(left_wall, front_wall, right_wall)
            print(f"\n=== ゴール到達！ ({current_x}, {current_y}) ===")
            mob_thread.send_command('STOP', 300, 1200, 90)
            mob_thread.wait_response()
            break
        
        # 次の進行方向を決定
        next_heading = explorer.decide_heading(left_wall, front_wall, right_wall)
        
        if next_heading is None:
            print(f"\n=== ゴールまでの道がありません！ ===")
            print(f"現在位置 ({current_x}, {current_y}) からゴールに到達できません")
            mob_thread.send_command('STOP', 300, 1200, 90)
            mob_thread.wait_response()
            break
        
        print(f"#Next heading: {next_heading.name}")
        
        # アクション決定と実行
        action = relative_to_action(current_heading, next_heading)
        print(f"#Action: {action}")
        
        execute_action(mob_thread, action)
        wait_for_done(mob_thread)
        
        # 迷路マップ表示（10ステップごと）
        if step_count % 10 == 0:
            print("\n=== 現在の迷路マップ ===")
            maze_map = explorer.render_text(show_goal=explorer.goals[0] if explorer.goals else None)
            print(maze_map)
    
    if step_count >= max_steps:
        print(f"\n=== 最大ステップ数 {max_steps} に到達しました ===")
        mob_thread.send_command('STOP', 300, 1200, 90)
        mob_thread.wait_response()


def show_final_results(mob_thread: MobileBaseThread, explorer: AdachiExplorer) -> None:
    """最終結果を表示
    
    Args:
        mob_thread: MobileBaseスレッド
        explorer: AdachiExplorer
    """
    mob_thread.send_command('WALL', False)
    mob_thread.wait_response()
    
    mob_thread.send_command('SEN')
    response_type, sensor_data = mob_thread.wait_response(timeout=2.0)
    if response_type == 'DONE' and sensor_data:
        print(f"\n=== 最終位置 ===")
        print(f"Distance: {sensor_data['odo_dist']:.2f} mm")
        print(f"Angle: {sensor_data['odo_ang']:.4f} rad ({sensor_data['odo_ang'] * 180 / 3.14159:.2f} deg)")
    
    print("\n=== 最終迷路マップ ===")
    maze_map = explorer.render_text(show_goal=explorer.goals[0] if explorer.goals else None)
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
    args = ap.parse_args()
    
    # シリアルポート接続
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    mob_thread = None
    
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # MobileBaseスレッドを初期化・開始
        mob_thread = MobileBaseThread(ser, timeout=args.timeout)
        mob_thread.start()
        print("MobileBaseスレッド開始")
        
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
        
        # ロボット初期化
        initialize_robot(mob_thread)
        
        # 迷路探索実行
        run_maze_exploration(mob_thread, explorer, args.max_steps)
        
        # 最終結果表示
        show_final_results(mob_thread, explorer)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n=== 中断されました ===")
        if mob_thread:
            mob_thread.send_command('STOP', 300, 1200, 90)
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
            mob_thread.stop()
            print("MobileBaseスレッド停止")
        
        ser.close()
        # ESP32をリセット
        print("\n=== ESP32をリセットしています... ===")
        esp32_reset(args.port)


if __name__ == '__main__':
    raise SystemExit(main())
