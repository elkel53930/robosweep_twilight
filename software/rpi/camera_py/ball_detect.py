#!/usr/bin/env python3
# ball_detect.py

from picamera2 import Picamera2
import cv2
import numpy as np
from skimage.measure import CircleModel, ransac
from typing import Tuple


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

    def calc_angle_distance(self, diameter: int, x: int, y: int) -> Tuple[float, float]:
        """
        画像上のボール情報からロボット中心からの角度と距離を計算
        
        Args:
            diameter: 画像上のボールの直径 [pixels]
            x: ボール中心のx座標 [pixels] (0が左端)
            y: ボール中心のy座標 [pixels] (0が上端)
        
        Returns:
            (angle, distance): 
                angle - ロボット正面からの角度 [degrees] (右回りが正)
                distance - ロボット中心からボール中心までの水平距離 [mm]
        """
        
        # ===== カメラ・センサーの仕様 =====
        # IMX708センサーサイズ
        SENSOR_WIDTH = 7.4   # mm
        SENSOR_HEIGHT = 5.6  # mm
        
        # 画像解像度
        IMG_WIDTH = 640      # pixels
        IMG_HEIGHT = 360     # pixels
        
        # 画角
        FOV_H = 102  # degrees (horizontal)
        FOV_V = 67   # degrees (vertical)
        
        # ボールの実際の直径
        BALL_DIAMETER = 70   # mm
        BALL_RADIUS = 35     # mm
        
        # カメラの取り付け位置(ロボット座標系)
        CAM_POS_X = 0        # mm (中心から右方向)
        CAM_POS_Y = 28       # mm (中心から前方向)
        CAM_POS_Z = 90       # mm (中心から上方向)
        
        # カメラの傾き
        TILT_ANGLE = 25      # degrees (下向き)
        
        # ===== 焦点距離の計算(ピクセル単位) =====
        # 画角から焦点距離を逆算
        fx = (IMG_WIDTH / 2) / np.tan(np.radians(FOV_H / 2))
        fy = (IMG_HEIGHT / 2) / np.tan(np.radians(FOV_V / 2))
        
        # 画像中心
        cx = IMG_WIDTH / 2
        cy = IMG_HEIGHT / 2
        
        # ===== Step 1: 画像座標をカメラ座標系の方向ベクトルに変換 =====
        # 正規化座標
        x_norm = (x - cx) / fx
        y_norm = (y - cy) / fy
        
        # カメラ座標系の方向ベクトル(正規化前)
        # カメラ座標系: X=右, Y=下, Z=光軸方向
        v_cam = np.array([x_norm, y_norm, 1.0])
        # 正規化
        v_cam = v_cam / np.linalg.norm(v_cam)
        
        # ===== Step 2: カメラ座標系からロボット座標系への変換 =====
        # ロボット座標系: X=右, Y=前, Z=上
        # カメラはY軸(前方)から下向き25度回転
        
        # 回転角(ラジアン)
        theta = np.radians(TILT_ANGLE)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # カメラ座標系の各軸をロボット座標系で表現
        # X_cam: 右方向(ロボットと同じ)
        X_cam = np.array([1, 0, 0])
        # Z_cam: 光軸(前方から下向き25度)
        Z_cam = np.array([0, cos_theta, -sin_theta])
        # Y_cam: Z_cam × X_cam
        Y_cam = np.cross(Z_cam, X_cam)
        
        # 回転行列(カメラ→ロボット)
        R = np.column_stack([X_cam, Y_cam, Z_cam])
        
        # カメラ座標系の方向ベクトルをロボット座標系に変換
        v_robot = R @ v_cam
        
        # ===== Step 3: 地面(ボール中心の高さ)との交点を計算 =====
        # 光線の方程式: P(t) = P_camera + t * v_robot
        # ボールの中心の高さ = BALL_RADIUS (地面から35mm)
        # P_camera[2] + t * v_robot[2] = BALL_RADIUS
        
        if v_robot[2] >= 0:
            # カメラが上向き→ボールは見えないはず
            return (0.0, 0.0)
        
        t = (BALL_RADIUS - CAM_POS_Z) / v_robot[2]
        
        # ボールの位置(ロボット座標系)
        P_ball = np.array([CAM_POS_X, CAM_POS_Y, CAM_POS_Z]) + t * v_robot
        
        # ===== Step 4: 距離と角度の計算 =====
        # 水平距離(XY平面)
        distance = np.sqrt(P_ball[0]**2 + P_ball[1]**2)
        
        # 角度(度) - 正面が0度、右回りが正
        angle = np.degrees(np.arctan2(P_ball[0], P_ball[1]))
        
        # ===== オプション: 直径による距離検証 =====
        # カメラからボールまでの3D距離
        d_3d = t
        
        # 期待される画像上の直径(pixels)
        # 投影された直径は角度によって変わるが、近似的に計算
        pixel_size = SENSOR_WIDTH / IMG_WIDTH  # mm/pixel
        expected_diameter = (fx * BALL_DIAMETER) / d_3d
        
        # 実際の直径と期待値の比較(デバッグ用)
        # print(f"Expected diameter: {expected_diameter:.1f} px, Actual: {diameter} px")
        
        return (angle, distance)

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
                angle, distance = detector.calc_angle_distance(
                    diameter=int(round(result['radius'] * 2)),
                    x=result['center'][0],
                    y=result['center'][1]
                )
                print(f"Detected: pos={result['center']}, "
                      f"r={result['radius']:.1f}, "
                      f"conf={result['confidence']:.2f}",
                      f"ang={angle:.1f} deg, dist={distance:.1f} mm")
            
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
