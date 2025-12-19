# SpresenseマルチIMU Add-onボード（通称：がいためIMU） ライブラリ



**SpresenseIMU** は、Sony Spresense メインボード上で
マルチIMU Add-onボードを利用するためのラッパーライブラリです。

サブコアを活用した高速処理・データ保存・姿勢推定（AHRS）・ジャイロコンパスなど、
複数の高度なサンプルを提供します。

-------------------------
## SpresenseマルチIMU Add-onボード（通称：がいためIMU）とは

"Spresense Multi-IMU”とは、ソニーセミコンダクタソリューションズ株式会社 から発売の Spresense Mainボードに載せる高精度IMUのAdd-onボードです。

詳細は、下記商品サイトをご覧ください。

![](https://wpp-cdn.developer.sony.com/dp-images/image/1200xAUTO/2vkl4atWlo/uploads/sites/19/2025/02/250226-IMU-Spresenseboard-with-Sony-Logo-new-1-1.png.webp?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1cmwiOiIvMnZrbDRhdFdsby91cGxvYWRzL3NpdGVzLzE5LzIwMjUvMDIvMjUwMjI2LUlNVS1TcHJlc2Vuc2Vib2FyZC13aXRoLVNvbnktTG9nby1uZXctMS0xLnBuZyIsImZvY3VzIjp7IngiOjAuNSwieSI6MC41fSwiYWx0IjoiIiwiYWx0VGV4dCI6IiIsIm1pbWUiOiJpbWFnZS9wbmciLCJfX2lzaW1hZ2UiOnRydWUsIndpZHRoIjoxMjAwLCJoZWlnaHQiOjY4NiwicmF0aW8iOjEuNzQ5MjcxMTM3MDI2MjM5LCJhbGxvd2VkU2l6ZXMiOlsiMXgxIiwiN3gzIiwiN3g0IiwiN3g1IiwiN3g2IiwiN3g3IiwiN3g4IiwiN3g5IiwiN3gxMCIsIjd4MTEiLCI3eDEyIiwiN3gxNiIsIjQ4eDQ4IiwiMzJ4MzIiLCIxNngxNiIsIjk2eDk2IiwiMTAweEFVVE8iLCIxMDB4NjUiLCIxMTB4ODIiLCIxODB4MTgwIiwiMTkyeDE5MiIsIjIwMHgxNTAiLCIyMjV4QVVUTyIsIjIyNXgxNzAiLCIyMjl4MzAwIiwiMzAweEFVVE8iLCIzNjh4QVVUTyIsIjQ1MHgzNDAiLCI0ODB4MjUwIiwiNTAweEFVVE8iLCI1MTJ4NTEyIiwiNjAweDQwMCIsIjYwMHhBVVRPIiwiNzUweEFVVE8iLCI3NTB4NTAwIiwiOTYweDUwMCIsIjk2MHhBVVRPIiwiMTAwMHhBVVRPIiwiMTIwMHhBVVRPIiwiMTIwMHg4MDAiLCIxOTIweEFVVE8iLCIxOTIweDEwMDAiXSwiaW1hZ2VWZXJzaW9uIjoidjYiLCJleHAiOjE5MDQ2ODgwMDAwMDB9.xXpF0pDx4D_LOkt85xx2er0kCf23tFOLnZLwlaqQhLQ)

SpresenseマルチIMU Add-onボード : https://developer.sony.com/ja/spresense/products/spresense-imu-add-on-board

Spresense開発サイト ：https://developer.sony.com/ja/spresense/

## 🧩 構造体の詳細

### `struct pwbImuData`

IMUボードから取得される1サンプル分のデータを保持する構造体です。
内部で `cxd5602pwbimu_data_t` を利用しており、加速度・ジャイロ・温度情報を格納します。

| フィールド | 型 | 説明 |
|-------------|----|------|
| `data.timestamp` | `uint32_t` | 取得時刻（IMU内部タイマ値） |
| `data.temp` | `float` | 温度（摂氏） |
| `data.gx, gy, gz` | `float` | ジャイロ角速度（rad/s） |
| `data.ax, ay, az` | `float` | 加速度（m/s²） |

#### 主なメソッド
| 関数名 | 機能 |
|---------|------|
| `operator+=(...)` | 他のIMUデータと加算（平均化に使用） |
| `operator/=(int)` | データを除算して平均化 |
| `print()` | タイムスタンプ付きで全データをCSV出力 |
| `printImu()` | 加速度＋ジャイロの6軸データを出力 |
| `printAccelerometer()` | 加速度データのみ出力 |
| `printGyro()` | ジャイロデータのみ出力 |

---

### `struct pwbGyroData`

ジャイロセンサの角速度ベクトルを扱う構造体。
`pwbImuData` からジャイロ成分だけを抽出して利用します。

| フィールド | 型 | 説明 |
|-------------|----|------|
| `x, y, z` | `double` | ジャイロ角速度（rad/s） |

#### 主なメソッド
| 関数名 | 機能 |
|---------|------|
| `operator=(const pwbImuData&)` | IMUデータからジャイロ成分をコピー |
| `operator+=(...)` | 他のジャイロデータと加算 |
| `print()` | X, Y, Z をCSV出力 |

---

## 🧠 クラス `SpresenseImuClass`

このクラスがIMUボード全体を制御し、
`pwbImuData` および `pwbGyroData` を用いてセンサーデータを取得・解析します。


-------------------------
### API

API一覧

| 関数名          | 説明 |
|-----------------|------|
| `begin()`       | ライブラリの初期化 |
| `end()`         | ライブラリの終了処理 |
| `initialize()`  | IMU センサーを初期化 |
| `finalize()`    | IMU センサーの終了処理 |
| `start()`       | データ取得開始 |
| `stop()`        | データ取得停止 |
| `get()`         | 最新のセンサーデータを取得 |
| `getAvarage()`  | 最新のセンサーデータのN個の平均値を取得 |
| `calcEarthsRotation()` | 地球自転による角速度を計算 |
| `calcAngleFrX()` | X軸方向との角度を算出 |

---

### `bool SpresenseIMU::begin(IMUConfig config = IMUConfig())`

- **説明**: ライブラリを初期化し、IMUConfig で指定された設定を有効にします。
- **引数**:
 - `IMUConfig config` : 初期化パラメータ（デフォルトコンストラクタ可）
- **戻り値**:
 - `true` : 初期化成功
 - `false` : 初期化失敗

---

### `void SpresenseIMU::end()`

- **説明**: ライブラリを終了させ、リソースを解放します。
- **引数**: なし
- **戻り値**: なし

---

### `bool SpresenseIMU::initialize()`

- **説明**: 複数 IMU センサーを個別に初期化する処理を実行します（下位レベルの初期化）。
- **引数**: なし
- **戻り値**:
 - `true` : 成功
 - `false` : 失敗

---

### `void SpresenseIMU::finalize()`

- **説明**: `initialize()` で確保されたリソースや設定を元に戻す／終了処理を行います。
- **引数**: なし
- **戻り値**: なし

---

### `bool SpresenseIMU::start()`

- **説明**: IMU データの取得（計測）を開始します。
- **引数**: なし
- **戻り値**:
 - `true` : 正常に開始
 - `false` : 開始失敗

---

### `void SpresenseIMU::stop()`

- **説明**: IMU データ取得を停止します。
- **引数**: なし
- **戻り値**: なし

---

### `bool SpresenseIMU::get(IMURead &dat)`

- **説明**: 最新の IMU センサー読み出しデータを取得します。
- **引数**:
 - `IMURead &dat` : センサー読み出し結果を格納する参照（構造体型）
- **戻り値**:
 - `true` : データ取得成功
 - `false` : データ取得失敗（未初期化など）

---

### `float SpresenseIMU::calcEarthsRotation(float lat)`

- **説明**: 地球自転による角速度（ジャイロバイアスに相当）を計算します。
- **引数**:
- **戻り値**:

---

### `float SpresenseIMU::calcAngleFrX(float x, float y)`

- **説明**: X軸成分からの傾きを算出します
- **引数**:
- **戻り値**:

---

### 補足：使用される型・構造体

- `IMUConfig`
 - 初期設定値をまとめた構造体またはクラス（動作モード、サンプルレート、フィルタ設定などを含む）
- `IMURead`
 - 加速度、ジャイロ、磁気などの生データまたは物理量を格納する構造体

---

### ⚠  注意点 & 推奨

- 各関数が失敗を返す状況（初期化されていない、IMU が応答しないなど）を呼び出し側でチェックすること。
- サブコア処理との同期や競合制御（ミューテックスなど）に注意。
- 高頻度取得／複数 IMU の同時制御時にはオーバーヘッドやタイミングズレに注意。

---

### 備考
- `getAverage()` はセンサーのノイズ低減に有効ですが、応答遅延が生じるのでリアルタイム性とのトレードオフがあります。


-------------------------


## 💾 サンプル一覧

### Spresense 単体でのサンプル

| サンプル | 内容 |
|-----------|------|
| **compass** | ジャイロコンパスで方位角を表示 |
| **compassMulti** | サブコアで動作するコンパスサンプル |
| **sample** | Rawデータを取得してシリアル出力 |
| **sample_multi** | サブコアでRawデータを並列取得 |
| **rawStored** | 1920Hzでの高速Rawデータ保存（SDカード対応）|
| **Orientation** | AHRSによる姿勢推定 |
| **tilt** | 加速度による傾き検出 |

### **Processing連携** でのサンプル
 | PC上のProcessingで波形／姿勢／位置を可視化 |

| サンプル | 内容 |
|-----------|------|
| **sample** | Raw データの波形をPC上のProcessingで表示するサンプル |
| **orientation** | 姿勢データをPC上のProcessingで可視化するサンプル|
| **posithin** | 位置データをPC上のProcessingで可視化するサンプル|

## 🪶 保存データの表示ツール

IMUボードのサンプル「rawStored」で保存されたバイナリデータは、
付属の Python スクリプト **`imu_viewer.py`** を使用して人間が読める形式に変換できます。

### 🔧 使用方法
```bash
python imu_viewer.py <データファイル名>
```

### 📄 出力形式
スクリプトは各サンプルを1行のCSVとして出力します。

| 項目 | 内容 |
|------|------|
| `timestamp` | 取得時刻（内部タイマ値 / 19.2MHz 換算） |
| `temp` | 温度（℃） |
| `gx, gy, gz` | ジャイロ角速度（rad/s） |
| `ax, ay, az` | 加速度（m/s²） |

出力例:
```
0.000052,24.31,0.02,-0.01,0.01,0.01,0.00,9.80
0.000104,24.31,0.01,-0.02,0.01,0.00,0.00,9.80
...
```
