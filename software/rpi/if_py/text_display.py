#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ディスプレイに文字列を表示するためのクラス
メニューシステムなどに使用可能
"""

from luma.core.render import canvas
from PIL import ImageFont


class TextDisplay:
    """
    ディスプレイに文字列を表示するためのクラス
    フォントサイズ10、色と背景色をカスタマイズ可能
    """
    
    def __init__(self, device, font_size=10, text_color="white", bg_color="black"):
        """
        TextDisplayクラスの初期化
        
        Args:
            device: luma.coreのデバイスオブジェクト
            font_size: フォントサイズ（デフォルト: 10）
            text_color: 文字色（デフォルト: "white"）
            bg_color: 背景色（デフォルト: "black"）
        """
        self.device = device
        self.font_size = font_size
        self.text_color = text_color
        self.bg_color = bg_color
        
        # フォントの読み込み
        try:
            self.font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 
                font_size
            )
        except IOError:
            # フォントが見つからない場合はデフォルトフォントを使用
            self.font = ImageFont.load_default()
    
    def display_text(self, text, x=None, y=None, align="left"):
        """
        指定された位置にテキストを表示
        
        Args:
            text: 表示するテキスト
            x: X座標（Noneの場合は自動配置）
            y: Y座標（Noneの場合は自動配置）
            align: テキストの配置（"left", "center", "right"）
        """
        with canvas(self.device) as draw:
            # 背景色で塗りつぶし
            draw.rectangle(
                [(0, 0), (self.device.width, self.device.height)], 
                fill=self.bg_color
            )
            
            # テキストのバウンディングボックスを取得
            bbox = draw.textbbox((0, 0), text, font=self.font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
            
            # X座標の計算
            if x is None:
                if align == "center":
                    x = (self.device.width - text_width) // 2
                elif align == "right":
                    x = self.device.width - text_width
                else:  # left
                    x = 0
            
            # Y座標の計算（Noneの場合は中央）
            if y is None:
                y = (self.device.height - text_height) // 2
            
            # テキストを描画
            draw.text((x, y), text, fill=self.text_color, font=self.font)
    
    def display_menu(self, items, selected_index=0, y_start=0):
        """
        メニュー形式でアイテムのリストを表示
        
        Args:
            items: メニューアイテムのリスト
            selected_index: 選択されているアイテムのインデックス
            y_start: メニューの開始Y座標
        """
        with canvas(self.device) as draw:
            # 背景色で塗りつぶし
            draw.rectangle(
                [(0, 0), (self.device.width, self.device.height)], 
                fill=self.bg_color
            )
            
            y = y_start
            line_height = self.font_size + 2  # 行間を考慮
            
            for i, item in enumerate(items):
                # 選択されているアイテムは反転表示
                if i == selected_index:
                    # 選択された行の背景を描画
                    draw.rectangle(
                        [(0, y), (self.device.width, y + line_height)],
                        fill=self.text_color
                    )
                    # テキストは背景色で表示（反転）
                    draw.text((2, y), item, fill=self.bg_color, font=self.font)
                else:
                    # 通常のテキスト表示
                    draw.text((2, y), item, fill=self.text_color, font=self.font)
                
                y += line_height
                
                # 画面からはみ出す場合は停止
                if y >= self.device.height:
                    break
    
    def display_multi_line(self, lines, y_start=0, line_spacing=2):
        """
        複数行のテキストを表示
        
        Args:
            lines: テキスト行のリスト
            y_start: 開始Y座標
            line_spacing: 行間のスペース
        """
        with canvas(self.device) as draw:
            # 背景色で塗りつぶし
            draw.rectangle(
                [(0, 0), (self.device.width, self.device.height)], 
                fill=self.bg_color
            )
            
            y = y_start
            line_height = self.font_size + line_spacing
            
            for line in lines:
                draw.text((0, y), line, fill=self.text_color, font=self.font)
                y += line_height
                
                # 画面からはみ出す場合は停止
                if y >= self.device.height:
                    break
    
    def clear(self):
        """
        画面をクリア（背景色で塗りつぶし）
        """
        with canvas(self.device) as draw:
            draw.rectangle(
                [(0, 0), (self.device.width, self.device.height)], 
                fill=self.bg_color
            )
    
    def set_colors(self, text_color=None, bg_color=None):
        """
        テキスト色と背景色を変更
        
        Args:
            text_color: 新しいテキスト色（Noneの場合は変更しない）
            bg_color: 新しい背景色（Noneの場合は変更しない）
        """
        if text_color is not None:
            self.text_color = text_color
        if bg_color is not None:
            self.bg_color = bg_color


# 使用例
if __name__ == "__main__":
    import time
    from luma.core import cmdline
    
    try:
        # デバイスの作成
        parser = cmdline.create_parser(description='Text display example')
        args = parser.parse_args([
            '--interface', 'spi',
            '--display', 'ssd1331',
            '--width', '96',
            '--height', '64',
            '--rotate', '2'
        ])
        
        device = cmdline.create_device(args)
        
        # TextDisplayインスタンスの作成
        text_display = TextDisplay(device, font_size=10)
        
        # テキスト表示のデモ
        text_display.display_text("Hello!", align="center")
        time.sleep(2)
        
        # メニュー表示のデモ
        menu_items = ["Item 1", "Item 2", "Item 3", "Item 4"]
        for i in range(len(menu_items)):
            text_display.display_menu(menu_items, selected_index=i)
            time.sleep(1)
        
        # 複数行表示のデモ
        text_display.set_colors(text_color="yellow", bg_color="blue")
        lines = ["Line 1", "Line 2", "Line 3"]
        text_display.display_multi_line(lines)
        time.sleep(2)
        
        # クリア
        text_display.clear()
        
    except KeyboardInterrupt:
        pass
