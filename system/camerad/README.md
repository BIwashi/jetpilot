# camerad - Camera Daemon

openpilot のカメラキャプチャデーモン。VisionIPC 経由でフレームを他のプロセス（modeld, encoderd, ui など）に配信します。

## アーキテクチャ

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Camera Sensor  │────▶│    camerad      │────▶│   VisionIPC     │
│  (IMX219 etc)   │     │  (this daemon)  │     │   Consumers     │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                              │                        │
                              │                        ├── modeld
                              │                        ├── encoderd
                              ▼                        ├── ui
                        ┌───────────┐                  └── loggerd
                        │  cereal   │
                        │ (pubsub)  │
                        └───────────┘
```

## 対応プラットフォーム

| プラットフォーム | 実装ファイル | 環境変数 |
|------------------|--------------|----------|
| QCOM/Tici (Comma 3X) | `cameras/camera_qcom2.cc` | なし（デフォルト） |
| Jetson Orin Nano | `cameras/camera_jetson.cc` | `USE_JETSON_CAMERA=1` |

## Jetson での使用方法

### 必要な環境変数

```bash
# 必須: Jetson実装を有効化
export USE_JETSON_CAMERA=1

# カメラ設定（デフォルト値あり）
export ROAD_CAM=0              # センサーID（0, 1, 2...）またはGStreamerパイプライン
export CAMERA_WIDTH=1280       # フレーム幅（デフォルト: 1280）
export CAMERA_HEIGHT=720       # フレーム高さ（デフォルト: 720）
export CAMERA_FPS=20           # フレームレート（デフォルト: 20）

# オプション: 追加カメラ
export WIDE_CAM=1              # ワイドカメラ（センサーID）
export DRIVER_CAM=2            # ドライバーカメラ（センサーID）

# カメラ無効化
export DISABLE_ROAD=1          # ロードカメラを無効化
```

### カスタムGStreamerパイプライン

`ROAD_CAM` に数字以外を指定すると、カスタムパイプラインとして解釈されます：

```bash
# カスタムパイプライン例
export ROAD_CAM='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=NV12 ! appsink name=sink emit-signals=true max-buffers=2 drop=true sync=false'
```

### 起動

```bash
# 単体テスト
USE_JETSON_CAMERA=1 ROAD_CAM=0 ./system/camerad/camerad

# フルシステム（manager経由）
export USE_JETSON_CAMERA=1
export ROAD_CAM=0
export DISPLAY=:0
./system/manager/manager.py
```

## トラブルシューティング

### "Failed to create CaptureSession" エラー

nvargus-daemon が不正な状態です。回復方法：

```bash
# 状態確認
./scripts/check_nvargus.sh

# 回復
sudo systemctl restart nvargus-daemon
```

### カメラが検出されない

```bash
# カメラデバイス確認
ls -la /dev/video*

# センサー確認
v4l2-ctl --list-devices

# GStreamerで直接テスト
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink
```

### UIが真っ暗

1. nvargus-daemon の状態を確認
2. `GST_ARGUS: Setup Complete` がログに表示されているか確認
3. VisionIPC が正しく動作しているか確認

## ファイル構成

```
system/camerad/
├── main.cc                    # エントリーポイント（プラットフォーム分岐）
├── SConscript                 # ビルド設定
├── cameras/
│   ├── camera_common.h        # 共通インターフェース定義
│   ├── camera_common.cc       # 共通実装（QCOM用）
│   ├── camera_qcom2.cc        # QCOM/Spectra ISP 実装
│   ├── camera_jetson.cc       # Jetson GStreamer 実装
│   ├── spectra.cc/h           # Spectra ISP ドライバ
│   └── ...
├── sensors/                   # センサードライバ（QCOM用）
│   ├── sensor.h
│   ├── ox03c10.cc
│   └── os04c10.cc
└── test/
    └── ...
```

## 技術詳細

### Jetson 実装 (camera_jetson.cc)

- **GStreamer C API** を使用（OpenCV経由ではない）
- **nvarguscamerasrc** プラグインでCSIカメラをキャプチャ
- **NV12形式** で直接出力（BGR変換なし）
- **appsink** コールバックでフレーム処理
- **ExitHandler** による適切なシグナル処理とクリーンアップ

### GStreamerパイプライン

```
nvarguscamerasrc sensor-id=0
    ↓
video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=20/1
    ↓
nvvidconv
    ↓
video/x-raw, format=NV12
    ↓
appsink (max-buffers=2, drop=true, sync=false)
```

### 制限事項

- **NVENC非対応**: Jetson Orin Nano Super はハードウェアエンコーダーがないため、ソフトウェアエンコード（libx264）を使用
- **nvargus-daemon依存**: デーモンが不正な状態になると再起動が必要

## 関連ドキュメント

- [NVIDIA Argus Camera API](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)
- [GStreamer nvarguscamerasrc](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/Multimedia/AcceleratedGstreamer.html)
