#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
階層構造メニューシステム
text_displayとdirection_buttonsを使用して、複数の選択肢から選択
"""

from text_display import TextDisplay
from direction_buttons import DirectionButtons, Direction
import time


class MenuItem:
    """メニューアイテムを表すクラス"""
    
    def __init__(self, label, action=None, submenu=None):
        """
        Args:
            label (str): メニューに表示されるラベル
            action (callable, optional): 選択時に実行される関数
            submenu (list[MenuItem], optional): サブメニューのリスト
        """
        self.label = label
        self.action = action
        self.submenu = submenu
    
    def has_submenu(self):
        """サブメニューを持っているかを返す"""
        return self.submenu is not None and len(self.submenu) > 0
    
    def has_action(self):
        """アクションを持っているかを返す"""
        return self.action is not None


class MenuSelector:
    """階層構造メニューセレクタークラス
    
    上下ボタン: 選択項目の移動
    右ボタン: 決定/サブメニューへ進む
    左ボタン: キャンセル/上の階層に戻る
    """
    
    def __init__(self, device, menu_items, font_size=10, 
                 text_color="white", bg_color="black",
                 down_pin=2, up_pin=3, right_pin=17, left_pin=4):
        """
        Args:
            device: luma.coreのデバイスオブジェクト
            menu_items (list[MenuItem]): ルートメニューのアイテムリスト
            font_size (int): フォントサイズ
            text_color (str): テキスト色
            bg_color (str): 背景色
            down_pin (int): DOWNボタンのGPIOピン
            up_pin (int): UPボタンのGPIOピン
            right_pin (int): RIGHTボタンのGPIOピン
            left_pin (int): LEFTボタンのGPIOピン
        """
        self.display = TextDisplay(device, font_size, text_color, bg_color)
        self.buttons = DirectionButtons(down_pin, up_pin, right_pin, left_pin)
        
        # メニュー階層を管理するスタック
        self.menu_stack = []
        self.current_menu = menu_items
        self.selected_index = 0
        
        # 実行結果
        self.result = None
        self.running = False
    
    def _get_menu_labels(self):
        """現在のメニューからラベルのリストを取得"""
        labels = []
        for item in self.current_menu:
            label = item.label
            # サブメニューがある場合は矢印を追加
            if item.has_submenu():
                label += " >"
            labels.append(label)
        return labels
    
    def _render_menu(self):
        """現在のメニューを画面に描画"""
        labels = self._get_menu_labels()
        self.display.display_menu(labels, self.selected_index)
    
    def _move_up(self):
        """選択を上に移動（循環）"""
        if self.selected_index > 0:
            self.selected_index -= 1
        else:
            # 一番上から一番下へ
            self.selected_index = len(self.current_menu) - 1
        self._render_menu()
    
    def _move_down(self):
        """選択を下に移動（循環）"""
        if self.selected_index < len(self.current_menu) - 1:
            self.selected_index += 1
        else:
            # 一番下から一番上へ
            self.selected_index = 0
        self._render_menu()
    
    def _select_item(self):
        """現在の選択項目を決定"""
        if not self.current_menu:
            return
        
        item = self.current_menu[self.selected_index]
        
        # サブメニューがある場合
        if item.has_submenu():
            self._enter_submenu(item.submenu)
        # アクションがある場合
        elif item.has_action():
            result = item.action()
            # 特別な結果が返された場合は終了
            if result in ("exit", "power_off_yes"):
                self.result = result
                self.running = False
            elif result is None and self.menu_stack:
                # Noneが返された場合、サブメニュー内なら親メニューに戻る
                self._go_back()
            else:
                # それ以外はメニューを再描画
                self._render_menu()
        else:
            # 何もない場合はメニューを再描画
            self._render_menu()
    
    def _enter_submenu(self, submenu):
        """サブメニューに入る"""
        # 現在の状態をスタックに保存
        self.menu_stack.append((self.current_menu, self.selected_index))
        # サブメニューに移動
        self.current_menu = submenu
        self.selected_index = 0
        self._render_menu()
    
    def _go_back(self):
        """前の階層に戻る"""
        if self.menu_stack:
            # スタックから前の状態を復元
            self.current_menu, self.selected_index = self.menu_stack.pop()
            self._render_menu()
        # ルートメニューの場合は何もしない（終了しない）
    
    def run(self, poll_interval=0.01):
        """メニュー選択を実行
        
        Args:
            poll_interval (float): ボタンチェックのポーリング間隔
            
        Returns:
            選択された結果（キャンセルの場合None）
        """
        self.running = True
        self.result = None
        
        # 初期画面を描画
        self._render_menu()
        
        try:
            while self.running:
                # ボタン入力を待つ
                pressed = self.buttons.wait_for_any_press(poll_interval=poll_interval)
                
                if pressed == Direction.UP:
                    self._move_up()
                elif pressed == Direction.DOWN:
                    self._move_down()
                elif pressed == Direction.RIGHT:
                    self._select_item()
                elif pressed == Direction.LEFT:
                    self._go_back()
                
                # 短い遅延（連続入力防止）
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            self.result = None
        
        return self.result
    
    def close(self):
        """リソースを解放"""
        self.buttons.close()
    
    def __enter__(self):
        """コンテキストマネージャーのエントリポイント"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャーの終了処理"""
        self.close()


# 使用例
if __name__ == "__main__":
    from luma.core import cmdline
    
    # アクション関数の例
    def action_option1():
        print("Option 1 selected!")
        return "option1"
    
    def action_option2():
        print("Option 2 selected!")
        return "option2"
    
    def action_setting1():
        print("Setting 1 changed!")
        return "setting1"
    
    def action_setting2():
        print("Setting 2 changed!")
        return "setting2"
    
    def action_exit():
        print("Exit selected! Powering off...")
        import os
        os.system("sudo poweroff")
        return "exit"
    
    # 階層構造のメニューを定義
    menu_items = [
        MenuItem("Menu Option 1", action=action_option1),
        MenuItem("Menu Option 2", action=action_option2),
        MenuItem("Settings", submenu=[
            MenuItem("Setting 1", action=action_setting1),
            MenuItem("Setting 2", action=action_setting2),
            MenuItem("Advanced", submenu=[
                MenuItem("Adv Setting 1", action=lambda: "adv1"),
                MenuItem("Adv Setting 2", action=lambda: "adv2"),
            ]),
        ]),
        MenuItem("Exit", action=action_exit),
    ]
    
    try:
        # デバイスの作成
        parser = cmdline.create_parser(description='Menu selector example')
        args = parser.parse_args([
            '--interface', 'spi',
            '--display', 'ssd1331',
            '--width', '96',
            '--height', '64',
            '--rotate', '2'
        ])
        
        device = cmdline.create_device(args)
        
        # メニューセレクターを実行
        with MenuSelector(device, menu_items) as menu:
            print("Starting menu selector...")
            print("Use direction buttons to navigate:")
            print("  UP/DOWN: Move selection")
            print("  RIGHT: Select/Enter submenu")
            print("  LEFT: Cancel/Go back")
            
            result = menu.run()
            
            print(f"\nMenu result: {result}")
    
    except KeyboardInterrupt:
        print("\nExiting...")
