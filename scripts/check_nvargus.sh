#!/bin/bash
# nvargus-daemon 状態確認スクリプト
# Usage: ./scripts/check_nvargus.sh

echo "=== nvargus-daemon プロセス確認 ==="
if pgrep -x nvargus-daemon > /dev/null; then
    echo "✓ nvargus-daemon は起動中"
    ps aux | grep nvargus-daemon | grep -v grep
else
    echo "✗ nvargus-daemon が起動していません"
    echo "  → sudo systemctl start nvargus-daemon"
    exit 1
fi

echo ""
echo "=== カメラセッションテスト ==="
result=$(timeout 3 gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! fakesink 2>&1)

if echo "$result" | grep -q "Done Success"; then
    echo "✓ カメラセッション: 正常"
    exit 0
elif echo "$result" | grep -q "Failed to create CaptureSession"; then
    echo "✗ カメラセッション: 不正な状態（スタックしたセッションあり）"
    echo ""
    echo "回復方法:"
    echo "  sudo systemctl restart nvargus-daemon"
    exit 1
elif echo "$result" | grep -q "No cameras available"; then
    echo "✗ カメラが検出されません"
    echo "  → カメラの物理接続を確認してください"
    exit 1
else
    echo "? 不明な状態:"
    echo "$result" | tail -5
    exit 1
fi
