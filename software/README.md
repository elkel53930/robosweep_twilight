# Arduino Nano Arm Board Controller

Arduino Nanoを使用したロボットアーム制御システムです。シリアル通信でサーボモータとDCモータを制御し、センサーデータを取得できます。

## システム構成

### Arduino側（arm_board.ino）
- **D8, D9**: PWMサーボモータ
- **A4, A5**: I2C電流センサ（INA219）
- **A1**: バッテリー電圧監視（10:1分圧回路）
- **D3**: NチャンネルMOSFET経由PWMモータ制御（2kHz）

### PC側（Python）
- `arm_board_controller.py`: メイン制御クラス
- `arm_board_test.py`: テスト用スクリプト

## 通信プロトコル

### 制御コマンド（PC → Arduino）
```
S1<angle>       # サーボ1角度設定（即座に移動）(例: S1090 = 90度)
S2<angle>       # サーボ2角度設定（即座に移動）(例: S2180 = 180度)
S1A<angle>,<speed>  # サーボ1角度と速度設定 (例: S1A090,030 = 90度に30°/secで移動)
S2A<angle>,<speed>  # サーボ2角度と速度設定 (例: S2A180,060 = 180度に60°/secで移動)
S1S<speed>      # サーボ1デフォルト速度設定 (例: S1S045 = 45°/sec)
S2S<speed>      # サーボ2デフォルト速度設定 (例: S2S090 = 90°/sec)
M<speed>        # モータ速度設定 (例: M050 = 50%)
STATUS          # センサーデータ要求
STOP            # 緊急停止
```

**速度指定について:**
- 速度は1-180 degrees/secの範囲で指定
- デフォルト速度は180°/sec（最高速度）
- 速度制御は20ms間隔（50Hz）で更新

### 応答（Arduino → PC）
```
OK:<command>:<value>                    # コマンド成功
ERROR:<message>                         # エラー
DATA:<battery>,<current>,<power>        # センサーデータ（100ms毎）
DEBUG:<message>                         # デバッグメッセージ
```

## セットアップ

### 必要なライブラリ

#### Arduino側
```cpp
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_INA219.h>  // Adafruit INA219ライブラリが必要
```

Arduino IDEのライブラリマネージャーから「Adafruit INA219」をインストールしてください。

#### Python側
```bash
pip install pyserial matplotlib
```

### ハードウェア接続

1. **サーボモータ**: D8, D9ピンに接続
2. **電流センサ（INA219）**: A4(SDA), A5(SCL)に接続
3. **バッテリー監視**: A1ピンに10:1分圧回路で接続
4. **DCモータ**: D3ピンにMOSFET経由で接続

## 使用方法

### 1. 基本的な制御

```python
from arm_board_controller import ArmBoardController

# コントローラー初期化
controller = ArmBoardController('/dev/ttyUSB0')

if controller.connect():
    # サーボ制御（即座に移動）
    controller.set_servo1_angle(90)   # サーボ1を90度に
    controller.set_servo2_angle(45)   # サーボ2を45度に
    
    # サーボ速度制御
    controller.set_servo1_angle_with_speed(180, 30)  # 180度に30°/secで移動
    controller.set_servo2_angle_with_speed(0, 60)    # 0度に60°/secで移動
    
    # デフォルト速度設定
    controller.set_servo1_speed(45)   # サーボ1のデフォルト速度を45°/sec
    controller.set_servo2_speed(90)   # サーボ2のデフォルト速度を90°/sec
    
    # モータ制御
    controller.set_motor_speed(30)    # モータを30%の速度で回転
    
    # センサーデータ取得
    data = controller.get_latest_sensor_data()
    if data:
        print(f"Battery: {data.battery_voltage:.2f}V")
        print(f"Current: {data.current_mA:.1f}mA")
        print(f"Power: {data.power_mW:.1f}mW")
    
    controller.disconnect()
```

### 2. コンテキストマネージャー使用

```python
with ArmBoardController('/dev/ttyUSB0') as controller:
    controller.set_servo1_angle(90)
    controller.set_motor_speed(50)
    time.sleep(5)
    # 自動的に切断される
```

### 3. リアルタイムデータ監視

```python
def sensor_callback(data):
    print(f"Battery: {data.battery_voltage:.2f}V, "
          f"Current: {data.current_mA:.1f}mA")

controller = ArmBoardController('/dev/ttyUSB0')
controller.connect()
controller.set_sensor_callback(sensor_callback)

# データは100ms毎にコールバックで受信される
time.sleep(10)
controller.disconnect()
```

## テストスクリプト

### 基本制御テスト
```bash
python3 arm_board_test.py basic
```

### サーボ速度制御テスト
```bash
python3 servo_speed_test.py
```
様々な速度でのサーボ制御をテストし、速度制御機能を検証します。

### リアルタイムプロットテスト
```bash
python3 arm_board_test.py plot
```

### インタラクティブ制御
```bash
python3 arm_board_test.py interactive
```

インタラクティブモードでは以下のコマンドが使用できます：
- `s1 90`: サーボ1を90度に設定
- `s2 180`: サーボ2を180度に設定
- `m 50`: モータを50%の速度に設定
- `stop`: 緊急停止
- `status`: ステータス要求
- `quit`: 終了

## 安全機能

- 角度制限: サーボ角度は0-180度に自動制限
- 速度制限: モータ速度は0-100%に自動制限
- 緊急停止: `STOP`コマンドですべてのモータを停止
- 接続監視: 通信エラー時の自動復旧

## トラブルシューティング

### INA219が見つからない場合
```
Failed to find INA219 chip - continuing without current sensor
```
- 配線を確認（VCC、GND、SDA=A4、SCL=A5）
- I2Cアドレスが0x40であることを確認
- `scanI2CDevices()`でI2Cデバイスをスキャン

### シリアル通信エラー
- ポート名を確認（`/dev/ttyUSB0`, `/dev/ttyACM0`など）
- ボーレートが115200であることを確認
- 他のプログラムがポートを使用していないか確認

### PWM周波数の変更
現在2kHzに設定されています。変更する場合は`setupMotorPWM()`関数を編集してください。

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。