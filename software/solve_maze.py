#!/usr/bin/env python3
"""迷路を解くスクリプト

AdachiアルゴリズムとMobileBaseを使用してマイクロマウス迷路を探索・解決します。
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

# パスを追加してインポート
sys.path.append('mob')
sys.path.append('search')

from mob.mobile_base import MobileBase
from search.micromouse_algorithms import AdachiExplorer, Direction


# センサーしきい値
LS_THRESHOLD = 100  # 左前壁検出
RS_THRESHOLD = 100  # 右前壁検出
LF_RF_THRESHOLD = 50  # 左側・右側壁検出（両方がこの値以上で前壁あり）


def detect_walls(sensor_data: dict) -> tuple[bool, bool, bool]:
    """センサーデータから壁の有無を判定
    
    Args:
        sensor_data: cmd_sen()の戻り値
    
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


def main() -> int:
    ap = argparse.ArgumentParser(description='迷路を解くスクリプト')
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
    
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # MobileBaseとAdachiExplorerを初期化
        mob = MobileBase(ser, timeout=args.timeout)
        
        # ゴール位置の設定
        if args.goal_x is not None and args.goal_y is not None:
            # 単一ゴールを指定
            goals = [(args.goal_x, args.goal_y)]
            print(f"=== ゴール位置: ({args.goal_x}, {args.goal_y}) ===")
        else:
            # デフォルト: 迷路中央の4マス
            c = args.maze_size // 2
            goals = [(c - 1, c - 1), (c, c - 1), (c - 1, c), (c, c)]
            print(f"=== ゴール位置: 中央4マス {goals} ===")
        
        explorer = AdachiExplorer(maze_size=args.maze_size, goals=goals)
        
        # 初期化コマンド
        print("=== 初期化 ===")
        mob.cmd_gcal()  # ジャイロキャリブレーション
        mob.cmd_rdst()  # 距離リセット
        mob.cmd_rang()  # 角度リセット
        mob.cmd_wall(True)  # 壁センサON
        
        # スタート（最初のセンサー読み取り後にアクション実行する）
        print("\n=== 迷路探索開始 ===")
        
        step_count = 0
        first_move = True  # 最初の移動フラグ
        
        while step_count < args.max_steps:
            step_count += 1
            print(f"\n--- Step {step_count} ---")
            
            # センサー情報取得
            sensor_data = mob.cmd_sen()
            if sensor_data is None:
                print("#センサーデータ取得失敗")
                time.sleep(0.1)
                continue
            
            # 壁検出
            left_wall, front_wall, right_wall = detect_walls(sensor_data)
            
            # 現在の位置と向き（decide_heading()呼び出し前に保存）
            current_x, current_y = explorer.pose.x, explorer.pose.y
            current_heading = explorer.pose.heading
            print(f"#Current pose: ({current_x}, {current_y}) heading={current_heading.name}")
            
            # ゴール判定
            if (current_x, current_y) in explorer.goals:
                print(f"\n=== ゴール到達！ ({current_x}, {current_y}) ===")
                mob.stop()
                break
            
            # 次の進行方向を決定（内部で姿勢が更新される）
            next_heading = explorer.decide_heading(left_wall, front_wall, right_wall)
            print(f"#Next heading: {next_heading.name}")
            
            # 必要なアクションを決定（現在の物理的な向きと、目指すべき絶対方向を比較）
            action = relative_to_action(current_heading, next_heading)
            print(f"#Action: {action}")
            
            # アクション実行
            # 最初の移動の場合はstart()を使用
            if first_move and action == 'fwd':
                mob.start()
                first_move = False
            elif action == 'fwd':
                mob.go_fwd()
            elif action == 'left':
                # 最初の移動で左の場合（ありえるケース）
                if first_move:
                    mob.start()
                    first_move = False
                mob.go_left()
            elif action == 'right':
                # 最初の移動で右の場合（ありえるケース）
                if first_move:
                    mob.start()
                    first_move = False
                mob.go_right()
            elif action == 'back':
                # 最初の移動で後退の場合（稀なケース）
                if first_move:
                    mob.start()
                    first_move = False
                mob.go_back()
            
            # 迷路マップ表示（10ステップごと）
            if step_count % 10 == 0:
                print("\n=== 現在の迷路マップ ===")
                maze_map = explorer.render_text(show_goal=explorer.goals[0] if explorer.goals else None)
                print(maze_map)
        
        if step_count >= args.max_steps:
            print(f"\n=== 最大ステップ数 {args.max_steps} に到達しました ===")
            mob.stop()
        
        # 最終結果表示
        mob.cmd_wall(False)  # 壁センサOFF
        sensor_data = mob.cmd_sen()
        if sensor_data:
            print(f"\n=== 最終位置 ===")
            print(f"Distance: {sensor_data['odo_dist']:.2f} mm")
            print(f"Angle: {sensor_data['odo_ang']:.4f} rad ({sensor_data['odo_ang'] * 180 / 3.14159:.2f} deg)")
        
        print("\n=== 最終迷路マップ ===")
        maze_map = explorer.render_text(show_goal=explorer.goals[0] if explorer.goals else None)
        print(maze_map)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\n=== 中断されました ===")
        mob.stop()
        return 1
    except Exception as e:
        print(f"\n\n=== エラー発生 ===")
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        ser.close()


if __name__ == '__main__':
    raise SystemExit(main())
