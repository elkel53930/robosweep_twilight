#!/usr/bin/env python3
# ball_detect.py

from picamera2 import Picamera2
import cv2
import numpy as np
from skimage.measure import CircleModel, ransac


class BallDetect:
    """
    ボール検出クラス
    HSV色空間でのマスキングとRANSACによる円検出を行う
    """
    
    def __init__(self, 
                 hsv_lower=None, 
                 hsv_upper=None,
                 morph_kernel=(7, 7),
                 ransac_residual_threshold=3.0,
                 ransac_max_trials=300,
                 min_radius=10.0,
                 max_radius=1000.0,
                 min_contour_points=30,
                 debug=False,
                 ball_diameter_mm=70.0,
                 image_width_px=None):
        """
        Args:
            hsv_lower: HSV下限値 (numpy array [H, S, V])
            hsv_upper: HSV上限値 (numpy array [H, S, V])
            morph_kernel: モルフォロジー演算のカーネルサイズ (tuple)
            ransac_residual_threshold: RANSAC残差閾値 (float)
            ransac_max_trials: RANSAC最大試行回数 (int)
            min_radius: 検出する最小半径 (float)
            max_radius: 検出する最大半径 (float)
            min_contour_points: RANSAC処理に必要な最小点数 (int)
            debug: デバッグモード（True: ウィンドウ表示あり）(bool)
            ball_diameter_mm: ボールの実際の直径 (mm) (float)
            image_width_px: 画像幅 (ピクセル) - Noneの場合は実際の画像から自動取得
        """
        # デフォルトパラメータ（黄色ボール用）
        self.hsv_lower = hsv_lower if hsv_lower is not None else np.array([15, 80, 80])
        self.hsv_upper = hsv_upper if hsv_upper is not None else np.array([36, 255, 255])
        self.morph_kernel = morph_kernel
        self.ransac_residual_threshold = ransac_residual_threshold
        self.ransac_max_trials = ransac_max_trials
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_contour_points = min_contour_points
        self.debug = debug
        
        # 距離推定用パラメータ
        self.ball_diameter_mm = ball_diameter_mm
        self.image_width_px = image_width_px
        
        # デバッグウィンドウ用
        if self.debug:
            cv2.namedWindow("BallDetect - Debug", cv2.WINDOW_NORMAL)
    
    def detect(self, bgr_frame):
        """
        BGR画像からボールを検出
        
        Args:
            bgr_frame: BGRフォーマットのOpenCV画像 (numpy array)
        
        Returns:
            dict or None: 検出結果
                - center: (x, y) 中心座標
                - radius: 半径 (float)
                - circularity: 円形度 (float)
                - area_ratio: 面積比 (float)
                - confidence: 信頼度 (float)
                - method: 検出手法名 (str)
            None: 検出失敗時
        """
        # 画像サイズを取得（初回のみ）
        if self.image_width_px is None:
            height, width = bgr_frame.shape[:2]
            self.image_width_px = width
        
        # HSVに変換
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        
        # マスク作成
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # モルフォロジー処理（ノイズ除去と穴埋め）
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, self.morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 輪郭抽出
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        # 最大輪郭を選択
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        cnt = contours[0]
        area = cv2.contourArea(cnt)
        
        # 面積が小さすぎる場合は無視
        if area < 50:
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        # 輪郭点を取得
        pts = cnt.reshape(-1, 2)
        if pts.shape[0] < self.min_contour_points:
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        # RANSACで円をフィッティング
        pts_float = pts.astype(np.float64)
        
        try:
            model_robust, inliers = ransac(
                pts_float,
                CircleModel,
                min_samples=3,
                residual_threshold=self.ransac_residual_threshold,
                max_trials=self.ransac_max_trials
            )
        except Exception:
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        if model_robust is None:
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        # 円のパラメータを取得
        xc, yc, r = model_robust.center[0], model_robust.center[1], model_robust.radius
        
        # 半径チェック
        if not (self.min_radius <= r <= self.max_radius):
            if self.debug:
                self._show_debug(bgr_frame, None, mask)
            return None
        
        # 円形度（インライア比）
        inlier_count = np.count_nonzero(inliers)
        circularity = float(inlier_count) / float(pts.shape[0])
        
        # 面積比（マスク面積 / 円面積）
        mask_area = np.count_nonzero(mask)
        circle_area = np.pi * r * r
        area_ratio = min(1.0, mask_area / (circle_area + 1e-6))
        
        # 信頼度を算出
        confidence = 0.7 * circularity + 0.3 * area_ratio
        confidence = float(np.clip(confidence, 0.0, 1.0))
        
        result = {
            "center": (int(round(xc)), int(round(yc))),
            "radius": float(r),
            "circularity": float(circularity),
            "area_ratio": float(area_ratio),
            "confidence": float(confidence),
            "method": "HSV+RANSAC"
        }
        
        # デバッグ表示
        if self.debug:
            self._show_debug(bgr_frame, result, mask)
        
        return result
    
    def calculate_distance_from_camera(self, radius_px):
        """
        検出されたボールの画像上の半径から、カメラからボールまでの距離を計算
        
        公式: distance = 29.84 × (画像の幅) / 円の直径
        
        Args:
            radius_px: 画像上のボールの半径 (ピクセル) (float)
        
        Returns:
            float: 推定距離 (mm)、推定不可の場合はNone
        """
        if self.image_width_px is None or radius_px <= 0:
            return None
        
        # 円の直径 = 2 × radius_px
        diameter_px = 2.0 * radius_px
        
        # 距離 = 29.84 × 画像の幅 / 円の直径
        distance_mm = 29.84 * self.image_width_px / diameter_px

        return float(distance_mm)
    
    def _show_debug(self, bgr_frame, result, mask):
        """
        デバッグ用の画像表示
        
        Args:
            bgr_frame: 元のBGR画像
            result: 検出結果（None可）
            mask: マスク画像
        """
        disp = bgr_frame.copy()
        
        if result is not None:
            cx, cy = result["center"]
            r = int(round(result["radius"]))
            conf = result["confidence"]
            circ = result["circularity"]
            area_ratio = result["area_ratio"]
            
            # 円と中心を描画
            cv2.circle(disp, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(disp, (cx, cy), 3, (0, 0, 255), -1)
            
            # 情報をテキスト表示
            text = f"r={r} conf={conf:.2f} circ={circ:.2f} area={area_ratio:.2f}"
            cv2.putText(disp, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 255), 2)
            cv2.putText(disp, f"center=({cx}, {cy})", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 距離推定を表示
            distance = self.calculate_distance_from_camera(result["radius"])
            if distance is not None:
                cv2.putText(disp, f"distance={distance:.1f}mm ({distance/10:.1f}cm)", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        else:
            cv2.putText(disp, "No ball detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        cv2.imshow("BallDetect - Debug", disp)
        cv2.waitKey(1)
    
    def set_debug(self, debug):
        """
        デバッグモードの切り替え
        
        Args:
            debug: True=デバッグモードON, False=OFF
        """
        self.debug = debug
        if self.debug:
            cv2.namedWindow("BallDetect - Debug", cv2.WINDOW_NORMAL)
        else:
            cv2.destroyWindow("BallDetect - Debug")
    
    def cleanup(self):
        """
        リソースのクリーンアップ（デバッグウィンドウを閉じる）
        """
        if self.debug:
            cv2.destroyWindow("BallDetect - Debug")


def main():
    """
    デモ用メイン関数
    """
    # カメラ初期化
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (640, 360), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    
    # BallDetectクラスのインスタンス作成（デバッグモード有効）
    detector = BallDetect(debug=True)
    
    print("Camera started (press ESC to quit).")
    
    try:
        while True:
            # フレーム取得
            frame_rgb = picam2.capture_array()
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            # ボール検出
            result = detector.detect(frame)
            
            # 結果を表示（デバッグモードなら画像も表示される）
            if result is not None:
                print(f"Detected: center={result['center']}, "
                      f"radius={result['radius']:.1f}, "
                      f"confidence={result['confidence']:.2f}")
            
            # ESCキーで終了
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    finally:
        detector.cleanup()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
