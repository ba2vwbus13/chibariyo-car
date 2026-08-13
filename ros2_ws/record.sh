#!/bin/bash
# シミュレーション画面を録画するスクリプト(コンテナ内で実行)
# 使い方: ./record.sh [録画秒数]   (デフォルト30秒)
# 出力: ~/ros2_ws/videos/wheelchair_YYYYmmdd_HHMMSS.mp4
#       (Mac側の ros2_ws/videos/ にそのまま現れる)
set -e

DUR=${1:-30}
DISP=${DISPLAY:-:1}
OUT_DIR="$HOME/ros2_ws/videos"
OUT="$OUT_DIR/wheelchair_$(date +%Y%m%d_%H%M%S).mp4"
mkdir -p "$OUT_DIR"

# ffmpeg / xdpyinfo が無ければインストール
if ! command -v ffmpeg >/dev/null || ! command -v xdpyinfo >/dev/null; then
    echo "ffmpegをインストールします..."
    sudo apt-get update -qq && sudo apt-get install -y -qq ffmpeg x11-utils
fi

RES=$(xdpyinfo -display "$DISP" | awk '/dimensions/{print $2}')
echo "録画開始: ${DUR}秒 (画面 $RES, display $DISP)"
echo "出力先: $OUT"

ffmpeg -y -loglevel warning -f x11grab -video_size "$RES" -framerate 15 \
    -i "$DISP" -t "$DUR" \
    -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' \
    -c:v libx264 -preset veryfast -pix_fmt yuv420p \
    "$OUT"

echo "録画完了: $OUT"
echo "(Macからは ros2_ws/videos/ フォルダで開けます)"
