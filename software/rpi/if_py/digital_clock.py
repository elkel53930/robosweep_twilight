#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2014-18 Richard Hull and contributors
# See LICENSE.rst for details.
# PYTHON_ARGCOMPLETE_OK

"""
A digital clock display showing time and date.
"""

import sys
import time
import datetime
from luma.core import cmdline
from luma.core.render import canvas
from PIL import ImageFont


def main(device):
    # フォントの設定（64x96のディスプレイに最適化）
    try:
        # 時刻用フォント（小さめに調整）
        font_time = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
        # 日付用フォント
        font_date = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 8)
    except IOError:
        # フォントが見つからない場合はデフォルトフォントを使用
        font_time = ImageFont.load_default()
        font_date = ImageFont.load_default()

    today_last_time = "Unknown"
    
    while True:
        now = datetime.datetime.now()
        today_time = now.strftime("%H:%M:%S")
        
        if today_time != today_last_time:
            today_last_time = today_time
            
            with canvas(device) as draw:
                # 現在時刻と日付を取得
                now = datetime.datetime.now()
                current_time = now.strftime("%H:%M:%S")
                current_date = now.strftime("%m-%d")  # 短縮形式
                day_of_week = now.strftime("%a")  # 短縮形式（例: Mon）
                
                # 画面の中央に配置するための計算
                # 時刻の表示（中央）
                time_bbox = draw.textbbox((0, 0), current_time, font=font_time)
                time_width = time_bbox[2] - time_bbox[0]
                time_x = (device.width - time_width) // 2
                time_y = (device.height // 2) - 7
                
                draw.text((time_x, time_y), current_time, fill="white", font=font_time)
                
                # 日付の表示（上部）
                date_bbox = draw.textbbox((0, 0), current_date, font=font_date)
                date_width = date_bbox[2] - date_bbox[0]
                date_x = (device.width - date_width) // 2
                date_y = 5
                
                draw.text((date_x, date_y), current_date, fill="yellow", font=font_date)
                
                # 曜日の表示（下部）
                day_bbox = draw.textbbox((0, 0), day_of_week, font=font_date)
                day_width = day_bbox[2] - day_bbox[0]
                day_x = (device.width - day_width) // 2
                day_y = device.height - 15
                
                draw.text((day_x, day_y), day_of_week, fill="cyan", font=font_date)
        
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        # 固定オプションでデバイスを作成
        parser = cmdline.create_parser(description='Digital clock display')
        args = parser.parse_args([
            '--interface', 'spi',
            '--display', 'ssd1331',
            '--width', '96',
            '--height', '64',
            '--rotate', '2'
        ])
        
        # デバイスを作成
        device = cmdline.create_device(args)
        main(device)
    except KeyboardInterrupt:
        pass
