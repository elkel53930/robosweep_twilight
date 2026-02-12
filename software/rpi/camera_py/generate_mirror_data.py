#!/usr/bin/env python3
"""
generate_mirror_data.py

キャリブレーションデータの左右対称データを生成するスクリプト

カメラの特性が左右対称であると仮定し、正の角度データから
負の角度データを生成して元のCSVに追加します。

使用方法:
    python3 generate_mirror_data.py [input_csv] [output_csv]
    
    input_csv: 入力CSVファイル（デフォルト: calibration_data.csv）
    output_csv: 出力CSVファイル（デフォルト: calibration_data_with_mirror.csv）
"""

import csv
import sys
from datetime import datetime


def mirror_calibration_data(input_csv='calibration_data.csv', 
                            output_csv='calibration_data_with_mirror.csv',
                            image_width=640):
    """
    キャリブレーションデータを左右対称に反転
    
    Args:
        input_csv: 入力CSVファイルパス
        output_csv: 出力CSVファイルパス
        image_width: 画像の幅（ピクセル）
    """
    
    # 元データの読み込み
    original_data = []
    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            original_data.append(row)
    
    print(f"読み込んだデータ数: {len(original_data)}")
    
    # ミラーデータの生成
    mirror_data = []
    for row in original_data:
        input_angle = float(row['input_angle_pi_rad'])
        
        # 角度が0の場合はミラーデータを作らない（重複するため）
        if abs(input_angle) < 1e-6:
            continue
        
        # 正の角度のデータのみをミラー化
        if input_angle > 0:
            mirror_row = row.copy()
            
            # タイムスタンプは現在時刻に更新（オプション）
            # mirror_row['timestamp'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # 角度を反転
            mirror_row['input_angle_pi_rad'] = f"{-input_angle:.4f}"
            
            # X座標を左右反転
            original_x = float(row['detected_x_px'])
            mirror_x = image_width - original_x
            mirror_row['detected_x_px'] = f"{mirror_x:.2f}"
            
            # 計算角度も反転
            calc_angle = float(row['calc_angle_pi_rad'])
            mirror_row['calc_angle_pi_rad'] = f"{-calc_angle:.4f}"
            
            # その他のフィールド（距離、直径、Y座標、計算距離）は変更なし
            mirror_data.append(mirror_row)
    
    print(f"生成したミラーデータ数: {len(mirror_data)}")
    
    # 元データとミラーデータを結合してソート
    all_data = original_data + mirror_data
    
    # 角度と距離でソート
    all_data.sort(key=lambda x: (float(x['input_angle_pi_rad']), 
                                 float(x['input_distance_mm'])))
    
    print(f"合計データ数: {len(all_data)}")
    
    # 出力CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_data)
    
    print(f"出力ファイル: {output_csv}")
    
    # 統計情報を表示
    print("\n=== データ統計 ===")
    angles = sorted(set(float(row['input_angle_pi_rad']) for row in all_data))
    print(f"角度の範囲: {min(angles):.4f}πrad ～ {max(angles):.4f}πrad")
    print(f"角度の種類数: {len(angles)}")
    print(f"角度一覧: {', '.join(f'{a:.4f}' for a in angles)}")
    
    distances = sorted(set(float(row['input_distance_mm']) for row in all_data))
    print(f"\n距離の範囲: {min(distances):.0f}mm ～ {max(distances):.0f}mm")
    print(f"距離の種類数: {len(distances)}")


def main():
    """メイン関数"""
    if len(sys.argv) > 3:
        print(__doc__)
        sys.exit(1)
    
    input_csv = sys.argv[1] if len(sys.argv) > 1 else 'calibration_data.csv'
    output_csv = sys.argv[2] if len(sys.argv) > 2 else 'calibration_data_with_mirror.csv'
    
    print(f"入力ファイル: {input_csv}")
    print(f"出力ファイル: {output_csv}")
    print(f"画像幅: 640px")
    print()
    
    try:
        mirror_calibration_data(input_csv, output_csv)
        print("\n✓ 完了しました")
    except FileNotFoundError:
        print(f"エラー: ファイル '{input_csv}' が見つかりません")
        sys.exit(1)
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
