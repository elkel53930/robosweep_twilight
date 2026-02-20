#!/usr/bin/env python3
"""スレッド化されたBallDetectクラス

カメラからの映像を解析し、ボールの位置を検出する。
連続5フレームの検出/非検出で判定する。
"""

from __future__ import annotations

import os
import threading
import time
from datetime import datetime
from typing import Optional, Dict, Any

import cv2
from picamera2 import Picamera2

import sys
sys.path.append('rpi/camera_py')
from ball_detect import BallDetect


class BallDetectThread:
    """BallDetectをスレッドで操作するクラス"""
    

    def __init__(self, picam2: Picamera2, detector: BallDetect, debug: bool = False):
        self.picam2 = picam2
        self.detector = detector
        self.debug = debug
        
        # ボール検出状態の共有変数
        self.ball_detected_lock = threading.Lock()
        self._detection_updated_since_last_check = False  # 検出状態が更新されたかのフラグ
        self._save_next_frame = False  # 次回フレームを保存するかのフラグ
        
        # デバッグ表示用フレーム（メインスレッドから読み取る）
        self.debug_frame = None
        self.debug_frame_lock = threading.Lock()
        
        # スレッド制御
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # ログディレクトリの作成
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = f"log/detect_{timestamp}"
        os.makedirs(self.log_dir, exist_ok=True)
        print(f"#BallDetect: ログディレクトリ作成: {self.log_dir}")
        
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
            ball_detected, ball_info, _, _ = self.detector.get_detection_info()
            self._detection_updated_since_last_check = False  # フラグをクリア
            return ball_detected, ball_info
    
    def has_detection_updated(self) -> bool:
        """前回のチェック以降に検出状態が更新されたかを取得
        
        このメソッドを呼ぶと内部フラグがクリアされる。
        is_ball_detected()を呼んだ場合もフラグはクリアされる。
        
        Returns:
            bool: 検出状態が更新されていればTrue
        """
        with self.ball_detected_lock:
            updated = self._detection_updated_since_last_check
            self._detection_updated_since_last_check = False  # フラグをクリア
            return updated
    
    def wait_detection_update(self, timeout: float = 5.0) -> bool:
        """検出状態が更新されるまで待機
        
        Args:
            timeout: タイムアウト時間（秒）
        
        Returns:
            bool: 更新があった場合True、タイムアウトした場合False
        """
        # 呼び出し前の更新を無視するため、フラグをクリア
        with self.ball_detected_lock:
            self._detection_updated_since_last_check = False
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.has_detection_updated():
                return True
            time.sleep(0.05)
        return False
    
    def get_debug_frame(self):
        """デバッグ表示用フレームを取得（メインスレッドから呼ぶ）"""
        with self.debug_frame_lock:
            return self.debug_frame.copy() if self.debug_frame is not None else None
    
    def save_next_frame(self):
        """次回のフレーム取得時に検出結果をjpgファイルに保存する
        
        このメソッドを呼び出すと、次のフレーム処理時に自動的に
        検出結果が描画された画像がlog/detect_YYYYMMDD_HHMMSS/に保存されます。
        """
        with self.ball_detected_lock:
            self._save_next_frame = True
            print("#BallDetect: 次回フレームを保存します")
    
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
                
                # 検出状態を更新（BallDetectクラス内で処理）
                with self.ball_detected_lock:
                    is_ball_in_frame, detection_changed = self.detector.update_detection_state(result)
                    
                    # ボールの情報が更新されたらフラグを立てる
                    self._detection_updated_since_last_check = True
                    # 検出確定の瞬間に画像を保存
                    if detection_changed and self.detector.ball_detected:
                        self._save_detection_image(frame, result, is_ball_in_frame, True)
                    
                    # 次回フレーム保存フラグがセットされていたら保存
                    if self._save_next_frame:
                        ball_detected_status = self.detector.ball_detected
                        self._save_detection_image(frame, result, is_ball_in_frame, ball_detected_status)
                        self._save_next_frame = False  # フラグをクリア
                        print("#BallDetect: フレームを保存しました")
                
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
    
    def _save_detection_image(self, frame_bgr, result, is_ball_in_frame: bool, ball_detected_status: bool):
        """ボール検出確定時の画像を保存
        
        Args:
            frame_bgr: 元のBGR画像
            result: 検出結果（None可）
            is_ball_in_frame: サイズ条件を満たすボールか
            ball_detected_status: ボール検出状態
        """
        try:
            # デバッグフレームと同じ画像を作成
            debug_image = self._create_debug_image(frame_bgr, result, is_ball_in_frame, ball_detected_status)
            
            if debug_image is not None:
                # 日時を含むファイル名を生成
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(self.log_dir, f"detect_{timestamp}.jpg")
                
                # 画像を保存
                cv2.imwrite(filename, debug_image)
                print(f"#BallDetect: 検出画像を保存しました: {filename}")
        except Exception as e:
            print(f"#BallDetect: 画像保存エラー: {e}")
    
    def _create_debug_image(self, frame_bgr, result, is_ball_in_frame: bool, ball_detected_status: bool):
        """デバッグ用の画像を作成（保存とデバッグ表示の両方で使用）
        
        Args:
            frame_bgr: 元のBGR画像
            result: 検出結果（None可）
            is_ball_in_frame: サイズ条件を満たすボールか
            ball_detected_status: ボール検出状態
        
        Returns:
            デバッグ情報が描画された画像（None if エラー）
        """
        if frame_bgr is None or frame_bgr.size == 0:
            return None
        
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
        _, _, consecutive_detected, consecutive_not_detected = self.detector.get_detection_info()
        count_text = f"Detected: {consecutive_detected}/{self.detector.detection_threshold}  Not: {consecutive_not_detected}/{self.detector.detection_threshold}"
        cv2.putText(disp, count_text, (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 確定状態表示
        status_text = "STATUS: BALL DETECTED" if ball_detected_status else "STATUS: NO BALL"
        status_color = (0, 255, 0) if ball_detected_status else (128, 128, 128)
        cv2.putText(disp, status_text, (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        return disp
    
    def _prepare_debug_frame(self, frame_bgr, result, is_ball_in_frame: bool):
        """デバッグ用の画像を準備（サブスレッドで実行、描画のみ）
        
        Args:
            frame_bgr: 元のBGR画像
            result: 検出結果（None可）
            is_ball_in_frame: サイズ条件を満たすボールか
        """
        # ball_detected状態を取得（ロック外なので安全にアクセス）
        with self.ball_detected_lock:
            ball_detected_status, _, _, _ = self.detector.get_detection_info()
        
        disp = self._create_debug_image(frame_bgr, result, is_ball_in_frame, ball_detected_status)
        
        if disp is not None:
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
