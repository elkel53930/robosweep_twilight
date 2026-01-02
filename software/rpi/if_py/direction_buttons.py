from gpiozero import Button
from time import sleep
from enum import Enum


class Direction(Enum):
    """方向を表す列挙型"""
    UP = "UP"
    DOWN = "DOWN"
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class DirectionButtons:
    """上下左右のボタン入力を扱うクラス
    
    GPIO pin配置:
    - GPIO 2: DOWN
    - GPIO 3: UP
    - GPIO 17: RIGHT
    - GPIO 4: LEFT
    """
    
    def __init__(self, down_pin=2, up_pin=3, right_pin=17, left_pin=4, 
                 bounce_time=0.05, pull_up=True):
        """
        Args:
            down_pin (int): DOWNボタンのGPIOピン番号
            up_pin (int): UPボタンのGPIOピン番号
            right_pin (int): RIGHTボタンのGPIOピン番号
            left_pin (int): LEFTボタンのGPIOピン番号
            bounce_time (float): チャタリング防止のバウンスタイム（秒）
            pull_up (bool): プルアップ抵抗を使用するか
        """
        self.buttons = {
            Direction.DOWN: Button(down_pin, bounce_time=bounce_time, pull_up=pull_up),
            Direction.UP: Button(up_pin, bounce_time=bounce_time, pull_up=pull_up),
            Direction.RIGHT: Button(right_pin, bounce_time=bounce_time, pull_up=pull_up),
            Direction.LEFT: Button(left_pin, bounce_time=bounce_time, pull_up=pull_up)
        }
    
    def is_pressed(self, direction):
        """指定された方向のボタンが押されているかを返す
        
        Args:
            direction (Direction): 確認する方向
            
        Returns:
            bool: ボタンが押されていればTrue
        """
        return self.buttons[direction].is_pressed
    
    def wait_for_press(self, direction, timeout=None):
        """特定のボタンが押されるまで待つ
        
        Args:
            direction (Direction): 待機する方向
            timeout (float, optional): タイムアウト時間（秒）。Noneの場合は無期限
            
        Returns:
            bool: タイムアウトした場合False、ボタンが押された場合True
        """
        return self.buttons[direction].wait_for_press(timeout=timeout)
    
    def wait_for_release(self, direction, timeout=None):
        """特定のボタンが離されるまで待つ
        
        Args:
            direction (Direction): 待機する方向
            timeout (float, optional): タイムアウト時間（秒）。Noneの場合は無期限
            
        Returns:
            bool: タイムアウトした場合False、ボタンが離された場合True
        """
        return self.buttons[direction].wait_for_release(timeout=timeout)
    
    def wait_for_any_press(self, poll_interval=0.01, timeout=None, wait_for_release=True):
        """いずれかのボタンが押されるまで待ち、押されたボタンを返す
        
        押されていない状態から押された状態への遷移を検出します。
        
        Args:
            poll_interval (float): ポーリング間隔（秒）
            timeout (float, optional): タイムアウト時間（秒）。Noneの場合は無期限
            wait_for_release (bool): Trueの場合、ボタンが離されるまで待ってから返す
            
        Returns:
            Direction or None: 押されたボタンの方向。タイムアウトした場合None
        """
        import time
        start_time = time.time()
        
        # 全てのボタンが離されるまで待つ（最初に押されているボタンを無視）
        while any(button.is_pressed for button in self.buttons.values()):
            if timeout is not None:
                if time.time() - start_time >= timeout:
                    return None
            sleep(poll_interval)
        
        # ボタンが押されるまで待つ
        while True:
            for direction, button in self.buttons.items():
                if button.is_pressed:
                    # ボタンが離されるまで待つ（チャタリング対策）
                    if wait_for_release:
                        while button.is_pressed:
                            sleep(poll_interval)
                    return direction
            
            if timeout is not None:
                if time.time() - start_time >= timeout:
                    return None
            
            sleep(poll_interval)
    
    def get_pressed_buttons(self):
        """現在押されているボタンのリストを返す
        
        Returns:
            list[Direction]: 押されているボタンの方向のリスト
        """
        return [direction for direction, button in self.buttons.items() 
                if button.is_pressed]
    
    def set_on_press(self, direction, callback):
        """特定方向のボタンが押された時のコールバックを設定
        
        Args:
            direction (Direction): コールバックを設定する方向
            callback (callable): 実行する関数
        """
        self.buttons[direction].when_pressed = callback
    
    def set_on_release(self, direction, callback):
        """特定方向のボタンが離された時のコールバックを設定
        
        Args:
            direction (Direction): コールバックを設定する方向
            callback (callable): 実行する関数
        """
        self.buttons[direction].when_released = callback
    
    def set_all_on_press(self, callback):
        """全てのボタンに押下時のコールバックを設定
        
        Args:
            callback (callable): 実行する関数。引数として方向(Direction)を受け取る
        """
        for direction in Direction:
            self.buttons[direction].when_pressed = lambda d=direction: callback(d)
    
    def set_all_on_release(self, callback):
        """全てのボタンに離脱時のコールバックを設定
        
        Args:
            callback (callable): 実行する関数。引数として方向(Direction)を受け取る
        """
        for direction in Direction:
            self.buttons[direction].when_released = lambda d=direction: callback(d)
    
    def close(self):
        """全てのボタンのリソースを解放"""
        for button in self.buttons.values():
            button.close()
    
    def __enter__(self):
        """コンテキストマネージャーのエントリポイント"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """コンテキストマネージャーの終了処理"""
        self.close()


if __name__ == "__main__":
    # 使用例
    print("Direction Buttons Test")
    print("Press any directional button... (Ctrl+C to exit)")
    
    with DirectionButtons() as buttons:
        try:
            while True:
                pressed = buttons.wait_for_any_press()
                if pressed:
                    print(f"{pressed.value} button pressed!")
        except KeyboardInterrupt:
            print("\nExiting...")
