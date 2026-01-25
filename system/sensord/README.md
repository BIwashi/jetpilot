# sensord - Sensor Daemon

openpilotのセンサーデーモン。加速度センサー、ジャイロスコープ、温度センサー、磁気センサーのデータを取得し、メッセージングシステムに公開します。

## サポートするセンサー

### ハードウェアセンサー (I2C)
- **LSM6DS3**: 加速度センサー + ジャイロスコープ (104Hz)
- **LSM6DS3_Temp**: 温度センサー
- **MMC5603NJ**: 磁気センサー (tiziデバイスのみ)

### ネットワークセンサー
- **Conduit IMU**: iPhone IMU via Zenoh (Conduit app)

---

## Conduit IMU センサー

iPhoneのConduitアプリを使用してIMUデータを取得します。Jetson Orin Nano SuperなどハードウェアIMUがないデバイスで使用できます。

### 必要なもの

1. **iPhone** (iOS 16以上)
2. **Conduit app**: [App Store](https://apps.apple.com/us/app/conduit-powered-by-ros/id6757171237)
3. **zenoh-python**: `uv sync --extra conduit`

### セットアップ

1. iPhoneにConduitアプリをインストール
2. Conduitアプリでルーターアドレスを設定 (例: `tcp/192.168.1.xxx:7447`)
3. Jetson上でZenohルーターを起動するか、iPhoneと同じネットワークに接続

### 起動方法

```bash
# 基本的な起動
CONDUIT_IMU=1 CONDUIT_ROUTER=tcp/192.168.1.xxx:7447 ./launch_openpilot.sh

# 静的キャリブレーション付き（初回セットアップ時に推奨）
CONDUIT_IMU=1 CONDUIT_ROUTER=tcp/192.168.1.xxx:7447 CONDUIT_STATIC_CALIBRATION=1 ./launch_openpilot.sh

# 取り付け向きを指定（MagSafe縦マウント、画面が運転者向き、充電ポートが右）
CONDUIT_IMU=1 CONDUIT_MOUNT_ORIENTATION="backward,left" ./launch_openpilot.sh
```

### 環境変数

| 変数 | デフォルト | 説明 |
|------|-----------|------|
| `CONDUIT_IMU` | 0 | 1でConduit IMU有効化 |
| `CONDUIT_ROUTER` | tcp/192.168.1.100:7447 | Zenohルーターアドレス |
| `CONDUIT_DOMAIN_ID` | 0 | ROS2ドメインID |
| `CONDUIT_IMU_TOPIC` | conduit/imu | IMUトピック名 |
| `CONDUIT_MOUNT_ORIENTATION` | up,forward | iPhoneの取り付け向き |
| `CONDUIT_ENABLE_CALIBRATION` | 1 | キャリブレーション有効化 |
| `CONDUIT_STATIC_CALIBRATION` | 0 | 起動時に静的キャリブレーション開始 |

---

## iPhoneの取り付け向き

`CONDUIT_MOUNT_ORIENTATION` の形式: `"画面の向き,ノッチの向き"`

### よく使う設定

| 設定値 | 説明 | 図 |
|--------|------|-----|
| `up,forward` | ダッシュボードに平置き、ノッチ前方 | 画面↑ ノッチ→前 |
| `up,backward` | ダッシュボードに平置き、ノッチ後方 | 画面↑ ノッチ→後 |
| `forward,up` | フロントガラスマウント、ノッチ上 | 画面→前 ノッチ↑ |
| `backward,left` | MagSafe縦マウント、充電ポート右 | 画面→後 ノッチ←左 |
| `backward,right` | MagSafe縦マウント、充電ポート左 | 画面→後 ノッチ→右 |

### MagSafe縦マウントの例 (`backward,left`)

```
【上から見た図】
          車両前方
             ↑
        ┌─────────┐
        │  背面   │ ← 背面カメラが前方
        │(カメラ) │
        └─────────┘
             ↓
          車両後方（画面は見えない）


【運転席から見た図】
            車両上
               ↑
    ┌──────────────────┐
    │                  │
    │ノッチ←   画面    │→ 充電ポート（右）
    │ (左)             │
    │                  │
    └──────────────────┘
```

---

## キャリブレーション

iPhoneの取り付け角度の微調整を行います。

### 1. 静的キャリブレーション（推奨）

車両を水平な場所に停車し、約2秒間データを収集してキャリブレーションを行います。

```bash
# 方法1: 起動時に自動実行
CONDUIT_IMU=1 CONDUIT_STATIC_CALIBRATION=1 ./launch_openpilot.sh

# 方法2: テストスクリプトで確認
python -m openpilot.system.sensord.sensors.conduit_calibration --static
```

**検出する項目:**
- ジャイロバイアス (静止時のジャイロ出力のずれ)
- ロール/ピッチオフセット (重力ベクトルからの傾き)

### 2. オンラインキャリブレーション（自動）

走行中に自動的にキャリブレーションを更新します。

- 直進走行時のデータを使用
- 車速 > 5 m/s で有効
- 時間とともに精度が向上

### キャリブレーションの確認

```bash
python -m openpilot.system.sensord.sensors.conduit_calibration --show
```

出力例:
```
Current calibration:
  Static calibrated: True
  Online calibrated: True
  Gyro bias: [0.001234, -0.000567, 0.000890]
  Roll offset: 2.34 deg
  Pitch offset: -1.23 deg
  Online samples: 1234
```

### キャリブレーションデータ

保存先: `/data/params/d/ConduitIMUCalibration`

---

## テスト

```bash
# Conduit IMUセンサーの単体テスト
python -m openpilot.system.sensord.sensors.conduit_imu --router tcp/192.168.1.xxx:7447

# キャリブレーションのテスト
python -m openpilot.system.sensord.sensors.conduit_calibration --static

# CDRパーサーの確認
python -m openpilot.system.sensord.sensors.cdr_parser
```

---

## トラブルシューティング

### Conduitに接続できない

1. iPhoneとJetsonが同じネットワークにいるか確認
2. Conduitアプリの設定でルーターアドレスが正しいか確認
3. ファイアウォールでポート7447が開いているか確認

### IMUデータが届かない

1. Conduitアプリでセンサーが有効になっているか確認
2. `CONDUIT_IMU_TOPIC`が正しいか確認
3. ログを確認: `journalctl -u sensord`

### キャリブレーションが失敗する

1. 静的キャリブレーション中は車両を完全に静止させる
2. 水平な場所で行う
3. iPhoneがしっかり固定されているか確認

---

## 関連リンク

- [Conduit App](https://apps.apple.com/us/app/conduit-powered-by-ros/id6757171237)
- [Conduit Support](https://github.com/youtalk/conduit-support)
- [Zenoh](https://zenoh.io/)
