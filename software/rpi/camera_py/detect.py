#!/usr/bin/env python3
# detect_yellow_ball_ransac.py

from picamera2 import Picamera2
import cv2
import numpy as np
from skimage.measure import CircleModel, ransac

# --- パラメータ調整用 ---

HSV_LOWER = np.array([15, 80, 80])   # H,S,V lower for yellow (V low to catch shadows)
HSV_UPPER = np.array([36, 255, 255]) # H,S,V upper for yellow
MORPH_KERNEL = (7, 7)                # closing kernel size
RANSAC_RESIDUAL_THRESHOLD = 3.0      # px 単位：円周点からの距離許容
RANSAC_MAX_TRIALS = 300
MIN_RADIUS = 10.0                    # px
MAX_RADIUS = 1000.0                  # px
MIN_CONTOUR_POINTS = 30              # RANSAC に渡す最小点数

# --- ボール検出関数 ---
def detect_yellow_ball_by_ransac(bgr_frame):
    """
    入力: BGR OpenCV イメージ
    出力: dict or None, mask
    dict keys: center (x,y), radius (float), circularity, confidence, method
    """
    # HSV に変換
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

    # マスク（影を拾いやすいよう V を低めに）
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # モルフォロジーで穴埋め（影で欠けがある場合の改善）
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # 輪郭抽出（全ての外郭点を得る）
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None, mask

    # 最大輪郭を選択（ただし面積が小さい場合は無視）
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    cnt = contours[0]
    area = cv2.contourArea(cnt)
    if area < 50:  # ノイズで小さいなら無視
        return None, mask

    pts = cnt.reshape(-1, 2)
    if pts.shape[0] < MIN_CONTOUR_POINTS:
        # 点が少なすぎると ransac が安定しない
        return None, mask

    # skimage ransac expects float coordinates
    pts_float = pts.astype(np.float64)

    # RANSAC で CircleModel をフィット
    try:
        model_robust, inliers = ransac(
            pts_float,
            CircleModel,
            min_samples=3,
            residual_threshold=RANSAC_RESIDUAL_THRESHOLD,
            max_trials=RANSAC_MAX_TRIALS
        )
    except Exception as e:
        # ransac が収束しないなど
        # print("RANSAC failed:", e)
        return None, mask

    if model_robust is None:
        return None, mask

    xc, yc, r = model_robust.params  # (xc, yc, radius)

    # 半径チェック
    if not (MIN_RADIUS <= r <= MAX_RADIUS):
        return None, mask

    # 円形度 = inlier 比
    inlier_count = np.count_nonzero(inliers)
    circularity = float(inlier_count) / float(pts.shape[0])

    # 面積比 = マスク面積 / 円面積（円にどれだけ塗られているか）
    mask_area = np.count_nonzero(mask)
    circle_area = np.pi * r * r
    area_ratio = min(1.0, mask_area / (circle_area + 1e-6))

    # confidence を circularity と area_ratio の組合せで算出（重みはお好みで調整）
    confidence = 0.7 * circularity + 0.3 * area_ratio
    confidence = float(np.clip(confidence, 0.0, 1.0))

    return {
        "center": (int(round(xc)), int(round(yc))),
        "radius": float(r),
        "circularity": float(circularity),
        "area_ratio": float(area_ratio),
        "confidence": float(confidence),
        "method": "HSV+RANSAC"
    }, mask

# --- メイン: カメラ読み出しと可視化 ---
def main():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (1280, 720), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    print("Camera started (press q to quit).")

    try:
        while True:
            frame_rgb = picam2.capture_array()
            # Picamera2 は RGB888 で受け取り、OpenCV は BGR を期待するので変換
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            result, mask = detect_yellow_ball_by_ransac(frame)

            # 描画
            disp = frame.copy()
            if result is not None:
                cx, cy = result["center"]
                r = int(round(result["radius"]))
                conf = result["confidence"]
                circ = result["circularity"]
                area_ratio = result["area_ratio"]

                # 円と中心
                cv2.circle(disp, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(disp, (cx, cy), 3, (0, 0, 255), -1)

                # 情報表示
                text = f"r={r} conf={conf:.2f} circ={circ:.2f} area={area_ratio:.2f}"
                cv2.putText(disp, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow("frame", disp)
            cv2.imshow("mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == 27: # Escキー
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
