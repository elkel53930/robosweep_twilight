# マイクロマウス迷路データ コンパクト形式 仕様書

## 概要

本ドキュメントでは、マイクロマウス迷路データの圧縮保存・転送を目的とした2つのコンパクト形式について説明します。

- **コンパクト形式**: 16進数ベースで中程度の圧縮率を実現し、デバッグ時にも内容を確認できる形式
- **ウルトラコンパクト形式**: バイナリ+Base64でMaximum圧縮を実現し、ネットワーク転送に最適化された形式

## 従来形式との比較

| 形式 | サイズ例（16x16迷路） | 圧縮率 | 可読性 | 用途 |
|------|---------------------|--------|--------|------|
| 従来ASCII | 1122 bytes | 100% | ★★★ | 人間による編集・確認 |
| コンパクト | 528 bytes | 47.1% | ★☆☆ | デバッグ可能な圧縮保存 |
| ウルトラコンパクト | 352 bytes | 31.4% | ☆☆☆ | 最大圧縮・ネットワーク転送 |

---

## 1. コンパクト形式（Compact Format）

### 仕様

**対象サイズ**: 最大99x99の迷路まで対応

**フォーマット構造**:
```
[サイズ2桁][壁データ][|][観測データ][|G座標4桁][|S座標5桁]
```

### エンコード詳細

#### ヘッダー部
- **サイズ**: 2桁の10進数（ゼロパディング）
  - 例: `16` → 16x16迷路

#### 壁データ部
各セルの壁情報を4ビットの16進数1文字で表現：
```
ビット配置: [WEST|SOUTH|EAST|NORTH]
```
- bit 0 (0x1): 北の壁
- bit 1 (0x2): 東の壁  
- bit 2 (0x4): 南の壁
- bit 3 (0x8): 西の壁

セルの走査順序: `y=0,x=0` → `y=0,x=1` → ... → `y=size-1,x=size-1`

#### 観測データ部
壁データと同じ形式で、各方向が観測済みかどうかを記録

#### オプション部
- **ゴール**: `|G` + X座標2桁 + Y座標2桁
- **スタート**: `|S` + X座標2桁 + Y座標2桁 + 向き1桁

### エンコード例

```python
# 16x16迷路の例
"16c555545555455456ac55695556ad7a|eaaac515556a8542aaaa955447abafaaaaa|G0707|S00011"
```

分解すると:
- `16`: 16x16迷路
- `c555...`: 壁データ（256文字）
- `|eaaa...`: 観測データ（256文字） 
- `|G0707`: ゴール座標(7,7)
- `|S0001`: スタート座標(0,0)、向き1（東）

### 使用関数

```python
# エンコード
compact_str = write_maze_compact(known_walls, observed, goal, start_pose)

# デコード  
known_walls, observed, goal, start_pose = read_maze_compact(compact_str)

# Explorer経由
explorer = AdachiExplorer()
compact_data = explorer.export_compact()
goal = explorer.import_compact(compact_data)
```

---

## 2. ウルトラコンパクト形式（Ultra-Compact Format）

### 仕様

**対象サイズ**: 最大255x255の迷路まで対応

**エンコード**: バイナリデータをBase64エンコード

### バイナリ構造

```
[サイズ1byte][フラグ1byte][壁データ][観測データ][オプション]
```

#### ヘッダー部
1. **サイズ**: 1バイト（0-255）
2. **フラグ**: 1バイト
   - bit 0 (0x01): ゴール情報有り
   - bit 1 (0x02): スタート情報有り（デフォルトでない場合）
   - bit 2-7: 予約

#### データ部
**ビットパッキング**: 各セル4ビットを効率的にバイト境界で詰め込み

セル毎の壁情報（4ビット）:
```
[bit3:西|bit2:南|bit1:東|bit0:北]
```

パッキング例（16x16 = 256セル）:
- 256セル × 4ビット = 1024ビット = 128バイト（壁）
- 256セル × 4ビット = 1024ビット = 128バイト（観測）

#### オプション部
フラグに応じて追加:
- **ゴール** (2バイト): X座標 + Y座標
- **スタート** (3バイト): X座標 + Y座標 + 向き

### エンコード例

```python
# Base64エンコード結果例
"EANcVUVVVVRFZcpVllVl2qeuqlxRVaZYJKqqWUV0uvqqqopFpspRcaqrqq+qqlxVgiWapqqqy2Srr1..."
```

バイナリ構造:
```
0x10: サイズ（16）
0x03: フラグ（ゴール+スタート有り）
0x5c, 0x55, ...: ビットパックされた壁データ（128バイト）
0xea, 0xaa, ...: ビットパックされた観測データ（128バイト）  
0x07, 0x07: ゴール座標(7,7)
0x00, 0x00, 0x01: スタート座標(0,0)、向き1
```

### 使用関数

```python
# エンコード
ultra_str = write_maze_ultracompact(known_walls, observed, goal, start_pose)

# デコード
known_walls, observed, goal, start_pose = read_maze_ultracompact(ultra_str)

# Explorer経由
explorer = AdachiExplorer()
ultra_data = explorer.export_ultracompact()
goal = explorer.import_ultracompact(ultra_data)
```

---

## 3. 利用シーン別推奨事項

### コンパクト形式を推奨する場面
- ✅ ログファイルの圧縮保存
- ✅ デバッグ時に内容を確認したい場合
- ✅ ヒューマンリーダブルな圧縮が必要
- ✅ 中程度の圧縮で十分な場合

### ウルトラコンパクト形式を推奨する場面  
- ✅ ネットワーク経由での迷路データ転送
- ✅ 大量の迷路データのアーカイブ
- ✅ メモリ制約が厳しい組み込みシステム
- ✅ 最大限の圧縮率が必要な場合
- ✅ データベースでの大量保存

### 従来ASCII形式を推奨する場面
- ✅ 人間が直接編集する迷路ファイル
- ✅ 迷路の視覚的確認が必要
- ✅ 既存ツールとの互換性が必要
- ✅ 学習・デバッグ目的での利用

---

## 4. 実装上の注意点

### エラーハンドリング
```python
try:
    data = write_maze_compact(walls, obs, goal, pose)
    walls, obs, goal, pose = read_maze_compact(data)
except ValueError as e:
    print(f"フォーマットエラー: {e}")
```

### サイズ制限
- **コンパクト形式**: 99x99まで
- **ウルトラコンパクト形式**: 255x255まで
- 制限を超えるとValueErrorが発生

### パフォーマンス特性
- **エンコード**: O(size²) 
- **デコード**: O(size²)
- **メモリ使用量**: O(size²)

---

## 5. テスト・検証

テストスクリプトでの検証:
```bash
cd software/search
python3 test_step_solver.py --compact-test maze_data/file.txt --verbose
```

出力例:
```
[INFO] Compact encoded (528 bytes): 16c555545555455456ac...
[INFO] Ultra-compact encoded (352 bytes): EANcVUVVVVRFZcpV...
[INFO] Size comparison: original=1122 bytes, compact=528 bytes (47.1%), ultra-compact=352 bytes (31.4%)
```

### 検証内容
1. エンコード/デコードの可逆性
2. 壁データの完全性
3. 観測データの完全性  
4. ゴール・スタート位置の保持
5. 圧縮率の測定

---

## 6. 今後の拡張可能性

### 追加可能な機能
- 距離情報の埋め込み
- 探索履歴の圧縮保存
- カスタムメタデータの追加
- より高度な圧縮アルゴリズムの適用

### 互換性の維持
- フォーマットバージョニング
- 下位互換性の保証
- 段階的な機能追加

---

## リファレンス

### 関数一覧

| 関数名 | 説明 | 戻り値 |
|--------|------|--------|
| `write_maze_compact()` | コンパクト形式エンコード | str |
| `read_maze_compact()` | コンパクト形式デコード | tuple |
| `write_maze_ultracompact()` | ウルトラコンパクト形式エンコード | str |
| `read_maze_ultracompact()` | ウルトラコンパクト形式デコード | tuple |
| `BaseExplorer.export_compact()` | エクスプローラーからエクスポート | str |
| `BaseExplorer.import_compact()` | エクスプローラーへインポート | tuple[int,int]\|None |
| `BaseExplorer.export_ultracompact()` | ウルトラコンパクトエクスポート | str |
| `BaseExplorer.import_ultracompact()` | ウルトラコンパクトインポート | tuple[int,int]\|None |

### ファイル構成
- `micromouse_algorithms.py`: メイン実装
- `test_step_solver.py`: テスト・検証機能
- このドキュメント: `COMPACT_FORMAT_DOC.md`