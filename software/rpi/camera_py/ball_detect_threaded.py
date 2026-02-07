#!/usr/bin/env python3
"""スレッド化されたBallDetectクラス

カメラからの映像を解析し、ボールの位置を検出する。
連続5フレームの検出/非検出で判定する。
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Dict, Any

import cv2
from picamera2 import Picamera2

import sys
sys.path.append('rpi/camera_py')
from ball_detect import BallDetect


class BallDetectThread:
    """BallDetectをスレッドで操作するクラス"""
    
    # ボール検出の閾値
    MIN_BALL_RADIUS = 60
    MAX_BALL_RADIUS = 180
    MIN_CENTER_Y = 100  # ボール中心のY座標の最小値
    
    DETECTION_THRESHOLD = 2  # 連続検出回数の閾値
    
    def __init__(self, picam2: Picamera2, detector: BallDetect, debug: bool = False):
        self.picam2 = picam2
        self.detector = detector
        self.debug = debug
        
        # ボール検出状態の共有変数
        self.ball_detected_lock = threading.Lock()
        self.ball_detected = False  # ボールが検出されているか
        self.ball_info: Optional[Dict[str, Any]] = None  # ボール情報
        
        # デバッグ表示用フレーム（メインスレッドから読み取る）
        self.debug_frame = None
        self.debug_frame_lock = threading.Lock()
        
        # スレッド制御
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 内部状態
        self._consecutive_detected = 0  # 連続検出カウント
        self._consecutive_not_detected = 0  # 連続非検出カウント
        
        # デバッグウィンドウ
        if self.debug:
            cv2.namedWindow("BallDetect - Debug", cv2.WINDOW_NORMAL)
    
    def start(self):
        """スレッドを開始"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._thread_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """スレッドを停止"""
        if not self.running:
            return
        
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        # デバッグウィンドウを閉じる
        if self.debug:
            try:
                cv2.destroyWindow("BallDetect - Debug")
            except:
                pass
    
    def is_ball_detected(self) -> tuple[bool, Optional[Dict[str, Any]]]:
        """ボールが検出されているかを取得
        
        Returns:
            (detected, ball_info) のタプル
            detected: ボールが検出されているか
            ball_info: ボール情報（検出されている場合）
        """
        with self.ball_detected_lock:
            return self.ball_detected, self.ball_info.copy() if self.ball_info else None
    
    def get_debug_frame(self):
        """デバッグ表示用フレームを取得（メインスレッドから呼ぶ）"""
        with self.debug_frame_lock:
            return self.debug_frame.copy() if self.debug_frame is not None else None
    
    def _thread_loop(self):
        """スレッドのメインループ"""
        # 最初の数フレームをスキップ（カメラの露光調整待ち）
        warmup_frames = 10
        frame_count = 0
        
        while self.running:
            try:
                # フレーム取得
                # Picamera2の"BGR888"設定でもRGB形式で返されるため、BGRに変換が必要
                frame_rgb = self.picam2.capture_array()
                frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                frame_count += 1
                
                # ウォームアップ中はスキップ
                if frame_count <= warmup_frames:
                    time.sleep(0.1)
                    continue
                                # 最初の有効フレームで情報表示
                if frame_count == warmup_frames + 1:
                    print(f"#BallDetect: First valid frame: shape={frame.shape}, dtype={frame.dtype}")
                    print(f"#BallDetect: Frame min={frame.min()}, max={frame.max()}, mean={frame.mean():.1f}")
                                # ボール検出
                result = self.detector.detect(frame)
                
                # ボールの有無を判定（サイズチェック＋Y座標チェック）
                is_ball_in_frame = False
                if result is not None:
                    radius = result['radius']
                    center_x, center_y = result['center']
                    if (self.MIN_BALL_RADIUS <= radius < self.MAX_BALL_RADIUS and
                        center_y >= self.MIN_CENTER_Y):
                        is_ball_in_frame = True
                
                # 連続検出カウントを更新
                if is_ball_in_frame:
                    self._consecutive_detected += 1
                    self._consecutive_not_detected = 0
                else:
                    self._consecutive_not_detected += 1
                    self._consecutive_detected = 0
                
                # ボール検出状態を更新
                with self.ball_detected_lock:
                    # 5フレーム連続で検出 → ボールあり
                    if self._consecutive_detected >= self.DETECTION_THRESHOLD:
                        if not self.ball_detected:
                            print(f"#BallDetect: ボール検出確定 (連続{self.DETECTION_THRESHOLD}フレーム)")
                        self.ball_detected = True
                        self.ball_info = result
                    
                    # 5フレーム連続で非検出 → ボールなし
                    elif self._consecutive_not_detected >= self.DETECTION_THRESHOLD:
                        if self.ball_detected:
                            print(f"#BallDetect: ボール消失確定 (連続{self.DETECTION_THRESHOLD}フレーム)")
                        self.ball_detected = False
                        self.ball_info = None
                
                # デバッグ表示用フレームを準備（メインスレッドで表示）
                if self.debug:
                    self._prepare_debug_frame(frame, result, is_ball_in_frame)
                
                # フレームレート調整（約10fps）
                time.sleep(0.1)
                
            except Exception as e:
                print(f"#BallDetectThread error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.5)  # エラー時は少し待つ
    
    def _prepare_debug_frame(self, frame_bgr, result, is_ball_in_frame: bool):
        """デバッグ用の画像を準備（サブスレッドで実行、描画のみ）
        
        Args:
            frame_bgr: 元のBGR画像
            result: 検出結果（None可）
            is_ball_in_frame: サイズ条件を満たすボールか
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return
        
        disp = frame_bgr.copy()
        
        # フレーム情報を表示
        height, width = disp.shape[:2]
        cv2.putText(disp, f"Frame: {width}x{height}", (width - 150, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if result is not None:
            cx, cy = result["center"]
            r = int(round(result["radius"]))
            conf = result["confidence"]
            circ = result["circularity"]
            area_ratio = result["area_ratio"]
            
            # サイズ条件を満たすかで色を変える
            color = (0, 255, 0) if is_ball_in_frame else (0, 165, 255)  # 緑 or オレンジ
            
            # 円と中心を描画
            cv2.circle(disp, (cx, cy), r, color, 2)
            cv2.circle(disp, (cx, cy), 3, (0, 0, 255), -1)
            
            # 情報をテキスト表示
            text = f"r={r} conf={conf:.2f} circ={circ:.2f} area={area_ratio:.2f}"
            cv2.putText(disp, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 255), 2)
            cv2.putText(disp, f"center=({cx}, {cy})", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # サイズ判定結果
            size_text = "SIZE: OK" if is_ball_in_frame else "SIZE: NG"
            size_color = (0, 255, 0) if is_ball_in_frame else (0, 0, 255)
            cv2.putText(disp, size_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, size_color, 2)
        else:
            cv2.putText(disp, "No ball detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 連続カウント表示
        count_text = f"Detected: {self._consecutive_detected}/{self.DETECTION_THRESHOLD}  Not: {self._consecutive_not_detected}/{self.DETECTION_THRESHOLD}"
        cv2.putText(disp, count_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 確定状態表示
        with self.ball_detected_lock:
            status_text = "STATUS: BALL DETECTED" if self.ball_detected else "STATUS: NO BALL"
            status_color = (0, 255, 0) if self.ball_detected else (128, 128, 128)
        cv2.putText(disp, status_text, (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # フレームを保存（メインスレッドで表示）
        with self.debug_frame_lock:
            self.debug_frame = disp


def main():
    """
    デモ用メイン関数
    """
    import argparse
    import sys
    sys.path.append('../..')
    
    ap = argparse.ArgumentParser(description='ボール検出スレッドのテスト')
    ap.add_argument('--no-debug', action='store_true', help='デバッグウィンドウを表示しない')
    args = ap.parse_args()
    
    # カメラ初期化
    print("=== カメラ初期化 ===")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (640, 360), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    
    # BallDetectクラスのインスタンス作成
    detector = BallDetect(debug=False)
    
    # BallDetectThreadのインスタンス作成
    ball_thread = BallDetectThread(picam2, detector, debug=not args.no_debug)
    ball_thread.start()
    
    # デバッグウィンドウの作成（メインスレッドで）
    if not args.no_debug:
        cv2.namedWindow("BallDetect - Debug", cv2.WINDOW_NORMAL)
    
    print("Ball detection thread started.")
    print("Press Ctrl+C to quit.")
    
    try:
        while True:
            # ボール検出状態を取得
            ball_detected, ball_info = ball_thread.is_ball_detected()
            
            if ball_detected and ball_info:
                print(f"Ball detected: center={ball_info['center']}, "
                      f"radius={ball_info['radius']:.1f}, "
                      f"confidence={ball_info['confidence']:.2f}")
            
            # デバッグフレームを表示（メインスレッドで）
            if not args.no_debug:
                debug_frame = ball_thread.get_debug_frame()
                if debug_frame is not None:
                    cv2.imshow("BallDetect - Debug", debug_frame)
                cv2.waitKey(10)
            else:
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        ball_thread.stop()
        picam2.stop()
        cv2.destroyAllWindows()
        print("Finished.")


if __name__ == "__main__":
    main()
