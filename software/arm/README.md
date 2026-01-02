# Arduino Nano Arm Board Controller

Arduino Nanoを使用したロボットアーム制御システムです。シリアル通信でサーボモータとDCモータを制御し、INA219センサーによるリアルタイム電流監視とmatplotlibによるグラフ表示機能を提供します。

## システム構成

### Arduino側（arm_board.ino）
- **D8, D9**: PWMサーボモータ（個別デフォルト角度: Servo1=80°, Servo2=180°）
- **A4, A5**: I2C電流センサ（INA219）
- **A1**: バッテリー電圧監視（10:1分圧回路）
- **D3**: NチャンネルMOSFET経由PWMモータ制御（2kHz）
- **センサーデータ送信**: 25ms間隔（40Hz）

### PC側（Python）
- `arm_board_controller.py`: メイン制御クラス
- `test.py`: インタラクティブ制御とファイル実行機能
- `throw_test.py`: 複雑な自動シーケンス実行
- `arm_board_test.py`: 基本機能テスト

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
- 速度制御は5ms間隔（200Hz）で更新

### 応答（Arduino → PC）
```
OK:<command>:<value>                    # コマンド成功
ERROR:<message>                         # エラー
DATA:<battery>,<current>,<power>        # センサーデータ（25ms毎、40Hz）
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

### 3. リアルタイムデータ監視とグラフ表示

```python
def sensor_callback(data):
    print(f"Battery: {data.battery_voltage:.2f}V, "
          f"Current: {data.current_mA:.1f}mA")

controller = ArmBoardController('/dev/ttyUSB0')
controller.connect()
controller.set_sensor_callback(sensor_callback)

# データは25ms毎にコールバックで受信される（40Hz）
time.sleep(10)
controller.disconnect()
```

### 4. ファイル実行でのリアルタイムプロット

ファイル実行モード時は自動的にmatplotlibグラフが表示されます：
- 上段：電流の生データ（青線）
- 下段：10ポイント移動平均（赤線）
- 時間軸：HH:MM:SS形式
- 自動スケーリング対応

## 新機能

### 1. 高速センサーデータ取得
- センサーデータ送信間隔：25ms（40Hz）
- サーボ制御更新間隔：5ms（200Hz）
- より細かい制御とデータ取得が可能

### 2. ファイル実行システム
- テキストファイルからコマンド読み込み
- ステップバイステップ実行
- 自動ループ機能
- コメント対応（#で始まる行）

### 3. リアルタイムグラフ監視
- matplotlib による電流値の可視化
- 移動平均によるノイズ除去
- 非ブロッキングプロット表示

### 4. 複雑な自動化シーケンス
- 電流値による条件判定
- 移動平均による電流変化の監視
- タイムアウト保護機能

## テストスクリプト

### インタラクティブ制御
```bash
python3 test.py
```

インタラクティブモードでは以下のコマンドが使用できます：
- `s1 90`: サーボ1を90度に設定（即座）
- `s2 180`: サーボ2を180度に設定（即座）
- `s1s 90 30`: サーボ1を90度に30°/secで移動
- `s2s 180 60`: サーボ2を180度に60°/secで移動
- `m 50`: モータを50%の速度に設定
- `stop`: 緊急停止
- `status`: センサーデータ表示
- `file <filename>`: テキストファイルからコマンド実行
- `sample`: サンプルコマンドファイル作成
- `help`: ヘルプ表示
- `quit`: 終了

### ファイル実行機能
```bash
# サンプルファイル作成
>>> sample
# ファイル実行（ステップバイステップ）
>>> file sample_commands.txt
```

ファイル実行モードでは：
- エンターキーで次のコマンドを実行
- 最後まで実行すると自動的にループ
- リアルタイム電流監視グラフ表示
- `q`で終了

### 自動化テストシーケンス
```bash
python3 throw_test.py
```

複雑な動作シーケンスを自動実行：
- サーボの段階的制御
- 電流監視と閾値判定
- 移動平均による電流変化の検出
- タイムアウト保護機能

### 基本制御テスト
```bash
python3 arm_board_test.py
```

## 安全機能

- 角度制限: サーボ角度は0-180度に自動制限
- 速度制限: モータ速度は0-100%に自動制限  
- 緊急停止: `STOP`コマンドですべてのモータを停止
- デフォルト位置復帰: 緊急停止時にServo1=80°、Servo2=180°に復帰
- 接続監視: 通信エラー時の自動復旧
- タイムアウト保護: 長時間処理での自動継続

## ファイルフォーマット

### コマンドファイル例（sample_commands.txt）
```
# Comment lines start with #
s1 180          # Set servo1 to 180 degrees instantly  
s2s 0 10        # Move servo2 to 0 degrees at 10°/sec
s1 45           # Set servo1 to 45 degrees
m 50            # Set motor to 50% speed
m 0             # Stop motor
stop            # Emergency stop
```

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