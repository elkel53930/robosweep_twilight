#!/usr/bin/env python3
"""
interpolate_position.py

キャリブレーションデータを使って、画像上の検出結果（X, Y）から
実際の角度と距離を線形補間で推定するスクリプト

使用方法:
    python3 interpolate_position.py <x> <y> [calibration_csv]
    
    x: 検出されたボール中心のX座標 (pixels)
    y: 検出されたボール中心のY座標 (pixels)
    calibration_csv: キャリブレーションCSVファイル（デフォルト: calibration_data_with_mirror.csv）

例:
    python3 interpolate_position.py 320 180
    python3 interpolate_position.py 320 180 calibration_data.csv
"""

import csv
import sys
import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator


class PositionInterpolator:
    """画像座標から実世界座標への補間クラス"""
    
    def __init__(self, calibration_csv='calibration_data_with_mirror.csv'):
        """
        初期化
        
        Args:
            calibration_csv: キャリブレーションデータのCSVファイル
        """
        self.calibration_csv = calibration_csv
        self.load_calibration_data()
        self.setup_interpolators()
    
    def load_calibration_data(self):
        """キャリブレーションデータを読み込む"""
        print(f"Loading calibration data from: {self.calibration_csv}")
        
        x_list = []
        y_list = []
        angle_list = []
        distance_list = []
        
        with open(self.calibration_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x_list.append(float(row['detected_x_px']))
                y_list.append(float(row['detected_y_px']))
                angle_list.append(float(row['input_angle_pi_rad']))
                distance_list.append(float(row['input_distance_mm']))
        
        # numpy配列に変換（2次元: X, Y）
        self.input_points = np.column_stack([x_list, y_list])
        self.angle_values = np.array(angle_list)
        self.distance_values = np.array(distance_list)
        
        print(f"Loaded {len(x_list)} calibration points")
        print(f"  X range: {min(x_list):.1f} - {max(x_list):.1f}")
        print(f"  Y range: {min(y_list):.1f} - {max(y_list):.1f}")
        print(f"  Angle range: {min(angle_list):.4f} - {max(angle_list):.4f} πrad")
        print(f"  Distance range: {min(distance_list):.1f} - {max(distance_list):.1f} mm")
    
    def setup_interpolators(self):
        """補間関数をセットアップ"""
        print("Setting up interpolators...")
        
        # 線形補間器（メイン）
        self.angle_interpolator = LinearNDInterpolator(self.input_points, self.angle_values)
        self.distance_interpolator = LinearNDInterpolator(self.input_points, self.distance_values)
        
        # 最近傍補間器（外挿用フォールバック）
        self.angle_nearest = NearestNDInterpolator(self.input_points, self.angle_values)
        self.distance_nearest = NearestNDInterpolator(self.input_points, self.distance_values)
        
        print("Interpolators ready")
    
    def interpolate(self, x, y, verbose=True):
        """
        画像座標から角度と距離を補間
        
        Args:
            x: 画像上のX座標 (pixels)
            y: 画像上のY座標 (pixels)
            verbose: 詳細情報を表示するか
        
        Returns:
            (angle_pi_rad, distance_mm): 角度（πラジアン）と距離（mm）のタプル
        """
        point = np.array([[x, y]])
        
        # 線形補間を試みる
        angle = self.angle_interpolator(point)[0]
        distance = self.distance_interpolator(point)[0]
        
        # 外挿領域（NaN）の場合は最近傍を使用
        use_nearest = False
        if np.isnan(angle) or np.isnan(distance):
            if verbose:
                print("Warning: Point is outside calibration range, using nearest neighbor")
            angle = self.angle_nearest(point)[0]
            distance = self.distance_nearest(point)[0]
            use_nearest = True
        
        if verbose:
            print(f"\n=== Interpolation Result ===")
            print(f"Input:")
            print(f"  X: {x:.2f} px")
            print(f"  Y: {y:.2f} px")
            print(f"\nOutput:")
            print(f"  Angle: {angle:.4f} πrad ({angle * 180:.2f}°)")
            print(f"  Distance: {distance:.2f} mm ({distance / 10:.2f} cm)")
            print(f"\nInterpolation method: {'Nearest Neighbor' if use_nearest else 'Linear'}")
        
        return angle, distance
    
    def batch_interpolate(self, points):
        """
        複数の点を一括で補間
        
        Args:
            points: [(x, y), ...] のリスト
        
        Returns:
            [(angle, distance), ...] のリスト
        """
        points_array = np.array(points)
        angles = self.angle_interpolator(points_array)
        distances = self.distance_interpolator(points_array)
        
        # NaNの場合は最近傍を使用
        nan_mask = np.isnan(angles) | np.isnan(distances)
        if np.any(nan_mask):
            print(f"Warning: {np.sum(nan_mask)} points outside calibration range")
            angles[nan_mask] = self.angle_nearest(points_array[nan_mask])
            distances[nan_mask] = self.distance_nearest(points_array[nan_mask])
        
        return list(zip(angles, distances))
    
    def find_nearest_calibration_point(self, x, y, n=3):
        """
        最も近いキャリブレーションポイントを見つける
        
        Args:
            x, y: 入力座標
            n: 返すポイント数
        
        Returns:
            最も近いn個のポイント情報のリスト
        """
        point = np.array([x, y])
        distances = np.linalg.norm(self.input_points - point, axis=1)
        nearest_indices = np.argsort(distances)[:n]
        
        print(f"\n=== Nearest {n} Calibration Points ===")
        for i, idx in enumerate(nearest_indices, 1):
            dist = distances[idx]
            x_cal, y_cal = self.input_points[idx]
            angle_cal = self.angle_values[idx]
            distance_cal = self.distance_values[idx]
            print(f"{i}. Distance: {dist:.2f}")
            print(f"   X={x_cal:.1f}, Y={y_cal:.1f}")
            print(f"   Angle={angle_cal:.4f} πrad, Distance={distance_cal:.1f} mm")


def main():
    """メイン関数"""
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        calibration_csv = sys.argv[3] if len(sys.argv) > 3 else 'calibration_data_with_mirror.csv'
    except ValueError:
        print("Error: X and Y must be numeric values")
        sys.exit(1)
    
    print("=" * 50)
    print("Position Interpolation from Calibration Data")
    print("=" * 50)
    
    try:
        # 補間器を初期化
        interpolator = PositionInterpolator(calibration_csv)
        
        # 補間を実行
        angle, distance = interpolator.interpolate(x, y)
        
        # 最近傍ポイントを表示
        interpolator.find_nearest_calibration_point(x, y, n=3)
        
        print("\n" + "=" * 50)
        
    except FileNotFoundError:
        print(f"Error: File '{calibration_csv}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
