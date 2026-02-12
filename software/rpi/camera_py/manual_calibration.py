#!/usr/bin/env python3
# manual_calibration.py

"""
ボール検出のキャリブレーションアプリ

操作手順:
1. カメラの前にボールを置く
2. アプリにボールの距離と角度を入力する
3. アプリがカメラ映像からボールを検出し、直径と位置を計算する(10回計測し、ソート後中央値を採用)
4. ユーザーがGUIで結果を確認し、OKなら距離、角度、直径、位置X、位置YをCSVに保存する
5. 1に戻る
"""

import tkinter as tk
from tkinter import ttk, messagebox
import cv2
import numpy as np
from PIL import Image, ImageTk
from picamera2 import Picamera2
from ball_detect import BallDetect
import csv
import os
from datetime import datetime
from threading import Thread, Event
import time
import sys

# ../../arm/arm.pyをインポート
sys.path.append('/home/k-iida/dev/robosweep_twilight/software')
from arm.arm import Arm


class ManualCalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ボール検出キャリブレーション")
        self.root.geometry("900x700")
        
        # カメラとボール検出器の初期化
        self.picam2 = None
        self.detector = BallDetect(debug=False)
        self.camera_running = False
        self.stop_event = Event()
        
        # アームコントローラーの初期化
        self.arm = None
        self.init_arm()
        
        # 計測データ
        self.measurements = []
        self.measurement_count = 10
        
        # CSV出力先
        self.csv_filename = "calibration_data.csv"
        
        # GUIのセットアップ
        self.setup_gui()
        
        # 初期値を設定
        self.set_initial_values()
        
        # カメラの初期化と開始
        self.init_camera()
        
    def init_arm(self):
        """アームの初期化とRUN_POSITIONへの移動"""
        try:
            self.arm = Arm()
            
            # アームサーボのトルクをON
            self.arm.set_servo_arm_torque(True)
            time.sleep(0.1)
            
            # RUN_POSITIONに移動
            print(f"アームをRUN_POSITION ({Arm.RUN_POSITION}度) に移動中...")
            self.arm.set_servo_arm_angle(Arm.RUN_POSITION, move_time=1000)
            time.sleep(1.5)  # 移動完了を待つ
            
            print("アーム初期化完了")
            
        except Exception as e:
            print(f"アーム初期化エラー: {e}")
            messagebox.showwarning("警告", f"アームの初期化に失敗しました: {e}\nアームなしで続行します。")
            self.arm = None
        
    def set_initial_values(self):
        """距離と角度の初期値を設定"""
        self.distance_entry.delete(0, tk.END)
        self.distance_entry.insert(0, "100")
        
        self.angle_entry.delete(0, tk.END)
        self.angle_entry.insert(0, "0.0")
    
    def setup_gui(self):
        """GUIのセットアップ"""
        
        # ========== 上部: 入力エリア ==========
        input_frame = ttk.LabelFrame(self.root, text="入力", padding=10)
        input_frame.pack(fill="x", padx=10, pady=5)
        
        # 距離入力
        ttk.Label(input_frame, text="距離 (mm):").grid(row=0, column=0, sticky="e", padx=5, pady=5)
        self.distance_entry = ttk.Entry(input_frame, width=15)
        self.distance_entry.grid(row=0, column=1, padx=5, pady=5)
        
        # 角度入力
        ttk.Label(input_frame, text="角度 (πrad):").grid(row=0, column=2, sticky="e", padx=5, pady=5)
        self.angle_entry = ttk.Entry(input_frame, width=15)
        self.angle_entry.grid(row=0, column=3, padx=5, pady=5)
        
        # 計測開始ボタン
        self.measure_btn = ttk.Button(input_frame, text="計測開始", command=self.start_measurement)
        self.measure_btn.grid(row=0, column=4, padx=10, pady=5)
        
        # ========== 中央: カメラ映像表示エリア ==========
        camera_frame = ttk.LabelFrame(self.root, text="カメラ映像", padding=10)
        camera_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.camera_label = tk.Label(camera_frame, bg="black")
        self.camera_label.pack(fill="both", expand=True)
        
        # ========== 下部左: 計測結果表示エリア ==========
        result_frame = ttk.LabelFrame(self.root, text="計測結果", padding=10)
        result_frame.pack(fill="x", padx=10, pady=5)
        
        # 計測進捗
        ttk.Label(result_frame, text="計測進捗:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.progress_label = ttk.Label(result_frame, text="0 / 10", font=("Arial", 12, "bold"))
        self.progress_label.grid(row=0, column=1, sticky="w", padx=5, pady=5)
        
        # 結果表示
        ttk.Label(result_frame, text="検出直径 (px):").grid(row=1, column=0, sticky="e", padx=5, pady=3)
        self.diameter_label = ttk.Label(result_frame, text="--", font=("Arial", 10))
        self.diameter_label.grid(row=1, column=1, sticky="w", padx=5, pady=3)
        
        ttk.Label(result_frame, text="位置X (px):").grid(row=2, column=0, sticky="e", padx=5, pady=3)
        self.pos_x_label = ttk.Label(result_frame, text="--", font=("Arial", 10))
        self.pos_x_label.grid(row=2, column=1, sticky="w", padx=5, pady=3)
        
        ttk.Label(result_frame, text="位置Y (px):").grid(row=3, column=0, sticky="e", padx=5, pady=3)
        self.pos_y_label = ttk.Label(result_frame, text="--", font=("Arial", 10))
        self.pos_y_label.grid(row=3, column=1, sticky="w", padx=5, pady=3)
        
        ttk.Label(result_frame, text="計算角度 (πrad):").grid(row=1, column=2, sticky="e", padx=5, pady=3)
        self.calc_angle_label = ttk.Label(result_frame, text="--", font=("Arial", 10))
        self.calc_angle_label.grid(row=1, column=3, sticky="w", padx=5, pady=3)
        
        ttk.Label(result_frame, text="計算距離 (mm):").grid(row=2, column=2, sticky="e", padx=5, pady=3)
        self.calc_distance_label = ttk.Label(result_frame, text="--", font=("Arial", 10))
        self.calc_distance_label.grid(row=2, column=3, sticky="w", padx=5, pady=3)
        
        # ========== 下部: ボタンエリア ==========
        button_frame = ttk.Frame(self.root, padding=10)
        button_frame.pack(fill="x", padx=10, pady=5)
        
        self.save_btn = ttk.Button(button_frame, text="OK - CSVに保存", 
                                   command=self.save_to_csv, state="disabled")
        self.save_btn.pack(side="left", padx=5)
        
        self.cancel_btn = ttk.Button(button_frame, text="キャンセル", 
                                     command=self.cancel_measurement)
        self.cancel_btn.pack(side="left", padx=5)
        
        ttk.Button(button_frame, text="終了", 
                  command=self.on_closing).pack(side="right", padx=5)
        
    def init_camera(self):
        """カメラの初期化と映像スレッドの開始"""
        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"size": (640, 360), "format": "BGR888"}
            )
            self.picam2.configure(config)
            self.picam2.start()
            
            self.camera_running = True
            self.camera_thread = Thread(target=self.update_camera, daemon=True)
            self.camera_thread.start()
            
        except Exception as e:
            messagebox.showerror("エラー", f"カメラの初期化に失敗しました: {e}")
            
    def update_camera(self):
        """カメラ映像の更新（別スレッドで実行）"""
        while self.camera_running and not self.stop_event.is_set():
            try:
                # フレーム取得（Picamera2はRGB形式で返す）
                frame_rgb = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                # ボール検出（BallDetectクラスはBGR形式を期待）
                result = self.detector.detect(frame_bgr)
                
                # 描画用のフレームはRGB形式のまま使用
                display_frame = frame_rgb.copy()
                
                # 検出結果を描画
                if result is not None:
                    cx, cy = result["center"]
                    r = int(round(result["radius"]))
                    
                    # 円と中心を描画（RGB形式で指定）
                    cv2.circle(display_frame, (cx, cy), r, (0, 255, 0), 2)  # 緑色の円
                    cv2.circle(display_frame, (cx, cy), 3, (0, 0, 255), -1)  # 赤色の点
                    
                    # 情報をテキスト表示（白色）
                    text = f"r={r} conf={result['confidence']:.2f}"
                    cv2.putText(display_frame, text, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # PIL Imageに変換してtkinterで表示
                image = Image.fromarray(display_frame)
                image = image.resize((640, 360), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(image=image)
                
                # メインスレッドで表示更新
                self.camera_label.configure(image=photo)
                self.camera_label.image = photo
                
                time.sleep(0.03)  # 約30fps
                
            except Exception as e:
                print(f"カメラ更新エラー: {e}")
                time.sleep(0.1)
                
    def start_measurement(self):
        """計測開始"""
        # 入力チェック
        try:
            distance = float(self.distance_entry.get())
            angle_pi_rad = float(self.angle_entry.get())  # πラジアン単位で入力
            # 内部では度単位で保持（後で使用）
            self.input_angle_pi_rad = angle_pi_rad
        except ValueError:
            messagebox.showwarning("入力エラー", "距離と角度を正しく入力してください")
            return
        
        # 計測データをリセット
        self.measurements = []
        self.progress_label.configure(text=f"0 / {self.measurement_count}")
        
        # ボタンの状態を変更
        self.measure_btn.configure(state="disabled")
        self.save_btn.configure(state="disabled")
        
        # 計測スレッドを開始
        Thread(target=self.measure_loop, daemon=True).start()
        
    def measure_loop(self):
        """計測ループ（別スレッドで実行）"""
        for i in range(self.measurement_count):
            try:
                # フレーム取得
                frame_rgb = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                # ボール検出
                result = self.detector.detect(frame_bgr)
                
                if result is not None:
                    # 直径と位置を記録
                    diameter = result['radius'] * 2
                    x, y = result['center']
                    
                    # 角度と距離を計算
                    angle, distance = self.detector.calc_angle_distance(
                        diameter=int(round(diameter)),
                        x=x,
                        y=y
                    )
                    
                    self.measurements.append({
                        'diameter': diameter,
                        'x': x,
                        'y': y,
                        'angle': angle,
                        'distance': distance
                    })
                    
                    # 進捗を更新
                    self.root.after(0, self.update_progress, len(self.measurements))
                    
                time.sleep(0.1)  # 100ms間隔
                
            except Exception as e:
                print(f"計測エラー: {e}")
        
        # 計測完了後、中央値を計算して表示
        self.root.after(0, self.calculate_median)
        
    def update_progress(self, count):
        """進捗表示の更新"""
        self.progress_label.configure(text=f"{count} / {self.measurement_count}")
        
    def calculate_median(self):
        """中央値を計算して表示"""
        if len(self.measurements) < self.measurement_count:
            messagebox.showwarning("計測失敗", 
                                 f"十分なデータが取得できませんでした ({len(self.measurements)}/{self.measurement_count})")
            self.measure_btn.configure(state="normal")
            return
        
        # 各パラメータをソートして中央値を取得
        diameters = sorted([m['diameter'] for m in self.measurements])
        xs = sorted([m['x'] for m in self.measurements])
        ys = sorted([m['y'] for m in self.measurements])
        angles = sorted([m['angle'] for m in self.measurements])
        distances = sorted([m['distance'] for m in self.measurements])
        
        mid = len(diameters) // 2
        
        self.median_result = {
            'diameter': diameters[mid],
            'x': xs[mid],
            'y': ys[mid],
            'angle': angles[mid],
            'distance': distances[mid]
        }
        
        # 結果を表示（角度は度からπラジアンに変換）
        self.diameter_label.configure(text=f"{self.median_result['diameter']:.2f}")
        self.pos_x_label.configure(text=f"{self.median_result['x']:.2f}")
        self.pos_y_label.configure(text=f"{self.median_result['y']:.2f}")
        angle_pi_rad = self.median_result['angle'] / 180.0  # 度 → πラジアン
        self.calc_angle_label.configure(text=f"{angle_pi_rad:.4f}")
        self.calc_distance_label.configure(text=f"{self.median_result['distance']:.2f}")
        
        # 保存ボタンを有効化
        self.save_btn.configure(state="normal")
        self.measure_btn.configure(state="normal")
        
    def save_to_csv(self):
        """CSVに保存"""
        try:
            # 入力値を取得（πラジアン単位）
            input_distance = float(self.distance_entry.get())
            input_angle_pi_rad = float(self.angle_entry.get())
            
            # 計算角度も度からπラジアンに変換
            calc_angle_pi_rad = self.median_result['angle'] / 180.0
            
            # CSVファイルが存在しない場合はヘッダーを書き込む
            file_exists = os.path.isfile(self.csv_filename)
            
            with open(self.csv_filename, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'input_distance_mm', 'input_angle_pi_rad', 
                            'detected_diameter_px', 'detected_x_px', 'detected_y_px',
                            'calc_angle_pi_rad', 'calc_distance_mm']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                if not file_exists:
                    writer.writeheader()
                
                writer.writerow({
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'input_distance_mm': input_distance,
                    'input_angle_pi_rad': f"{input_angle_pi_rad:.4f}",
                    'detected_diameter_px': f"{self.median_result['diameter']:.2f}",
                    'detected_x_px': f"{self.median_result['x']:.2f}",
                    'detected_y_px': f"{self.median_result['y']:.2f}",
                    'calc_angle_pi_rad': f"{calc_angle_pi_rad:.4f}",
                    'calc_distance_mm': f"{self.median_result['distance']:.2f}"
                })
            
            messagebox.showinfo("保存完了", f"データを{self.csv_filename}に保存しました")
            
            # 次の計測のために値を自動更新
            self.update_next_values(input_distance, input_angle_pi_rad)
            
            # 結果表示をクリア
            self.diameter_label.configure(text="--")
            self.pos_x_label.configure(text="--")
            self.pos_y_label.configure(text="--")
            self.calc_angle_label.configure(text="--")
            self.calc_distance_label.configure(text="--")
            self.progress_label.configure(text="0 / 10")
            
            # 保存ボタンを無効化
            self.save_btn.configure(state="disabled")
            
        except Exception as e:
            messagebox.showerror("保存エラー", f"CSV保存に失敗しました: {e}")
            
    def update_next_values(self, current_distance, current_angle_pi_rad):
        """次の計測値を自動設定"""
        # 距離が260mmの場合
        if current_distance >= 260:
            next_distance = 100
            next_angle = current_angle_pi_rad + 0.025
        else:
            # それ以外は距離+20mm、角度はそのまま
            next_distance = current_distance + 20
            next_angle = current_angle_pi_rad
        
        # 入力フィールドを更新
        self.distance_entry.delete(0, tk.END)
        self.distance_entry.insert(0, str(int(next_distance)))
        
        self.angle_entry.delete(0, tk.END)
        self.angle_entry.insert(0, f"{next_angle:.4f}")
    
    def cancel_measurement(self):
        """計測のキャンセル"""
        self.measurements = []
        self.progress_label.configure(text="0 / 10")
        self.diameter_label.configure(text="--")
        self.pos_x_label.configure(text="--")
        self.pos_y_label.configure(text="--")
        self.calc_angle_label.configure(text="--")
        self.calc_distance_label.configure(text="--")
        self.save_btn.configure(state="disabled")
        self.measure_btn.configure(state="normal")
        
    def on_closing(self):
        """アプリケーション終了時の処理"""
        self.camera_running = False
        self.stop_event.set()
        
        if self.picam2:
            self.picam2.stop()
        
        self.detector.cleanup()
        
        # アームサーボのトルクをオフ
        if self.arm:
            try:
                print("アームサーボのトルクをオフにしています...")
                self.arm.set_servo_arm_torque(False)
                self.arm.disconnect()
                print("アーム切断完了")
            except Exception as e:
                print(f"アーム切断エラー: {e}")
        
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ManualCalibrationApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
