#!/usr/bin/env python3
"""
A4用紙の中心から放射線を描画するプログラム
"""

from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
import math

def draw_radial_lines(filename="calib_sheet.pdf"):
    """
    A4用紙の中心から放射線と目盛りを描画する
    
    Parameters:
        filename (str): 出力PDFファイル名
    """
    # PDFキャンバスを作成
    c = canvas.Canvas(filename, pagesize=A4)
    width, height = A4
    
    # A4用紙の中心座標
    center_x = width / 2
    center_y = 0
    
    # 放射線のパラメータ
    angle_step_rad = 0.0125 * math.pi  # 0.1πラジアン刻み
    num_lines = int(2 * math.pi / angle_step_rad)  # 2πを0.1πで割った本数
    max_radius = 420 * mm  # 放射線の長さを210mmに固定
    
    # 目盛りのパラメータ
    tick_interval = 10 * mm  # 10mm刻み
    tick_length = 15 * mm  # 目盛りの長さ
    
    # 線の太さを設定
    c.setLineWidth(0.5)
    
    # フォント設定（距離表示用）
    c.setFont("Helvetica", 6)
    
    # 左上に目盛り情報を表示
    c.setFont("Helvetica", 10)
    info_x = 100 * mm
    info_y = height - 8 * mm
    c.drawString(info_x, info_y, f"{tick_interval / mm:.0f}mm")
    c.drawString(info_x, info_y - 5 * mm, f"{angle_step_rad / math.pi:.4f}pi")
    
    # フォントを戻す
    c.setFont("Helvetica", 6)
    
    # 同心円を描画（10mm刻み）
    num_circles = int(max_radius / tick_interval)
    for j in range(1, num_circles + 1):
        radius = j * tick_interval
        if radius > max_radius:
            break
        # 同心円を描画
        c.circle(center_x, center_y, radius, stroke=1, fill=0)
    
    # 放射線を描画
    for i in range(num_lines):
        angle_rad = i * angle_step_rad
        
        # 放射線の終点座標
        end_x = center_x + max_radius * math.cos(angle_rad)
        end_y = center_y + max_radius * math.sin(angle_rad)
        
        # 放射線を描画
        c.line(center_x, center_y, end_x, end_y)
        
        # 放射線と同心円の交点に距離と角度を表示（ひとつ飛ばし）
        if i % 2 == 0:  # 放射線方向
            for j in range(1, num_circles + 1):
                if j % 2 == 0: # 同心円方向
                    radius = j * tick_interval
                    if radius > max_radius:
                        break
                    
                    # 交点の座標
                    point_x = center_x + radius * math.cos(angle_rad)
                    point_y = center_y + radius * math.sin(angle_rad)
                    
                    # 距離（mm）と角度（xπ）を表示
                    distance_mm = int(radius / mm)
                    pi_fraction = angle_rad / math.pi -0.5
#                    text = f"{distance_mm}, {pi_fraction:.2f}"
                    
                    # テキストの位置を少しずらす
                    text_offset = 0 * mm
                    text_x = point_x + text_offset * math.cos(angle_rad)
                    text_y = point_y + text_offset * math.sin(angle_rad)
                    c.drawString(text_x, text_y, f"{distance_mm}")
                    c.drawString(text_x, text_y+3 * mm, f"{pi_fraction:.3f}")
    
    # 中心に小さな円を描画（参照用）
    c.circle(center_x, center_y, 1 * mm, fill=1)
    
    # 中心から左右84mmの位置に縦の線を描画
    vertical_offset = 84 * mm
    c.line(center_x - vertical_offset, 0, center_x - vertical_offset, height)  # 左側の線
    c.line(center_x + vertical_offset, 0, center_x + vertical_offset, height)  # 右側の線

    vertical_offset = (84-35) * mm
    c.line(center_x - vertical_offset, 0, center_x - vertical_offset, height)  # 左側の線
    c.line(center_x + vertical_offset, 0, center_x + vertical_offset, height)  # 右側の線
    
    # PDFを保存
    c.save()
    print(f"PDFファイルを作成しました: {filename}")
    print(f"- 放射線: {num_lines}本（0.1π刻み）")
    print(f"- 目盛り: {tick_interval / mm:.0f}mm刻み")
    print(f"- 最大半径: {max_radius / mm:.1f}mm")


if __name__ == "__main__":
    draw_radial_lines()
