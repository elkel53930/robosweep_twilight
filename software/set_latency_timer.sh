#!/bin/bash

if [ $# -lt 2 ]; then
    echo "Usage: $0 <latency_value> <device_pattern1> [device_pattern2] ..."
    echo "Example:"
    echo "  $0 1 ttyUSB0 ttyUSB1"
    echo "  $0 1 ttyUSB*"
    exit 1
fi

LATENCY=$1
shift
DEVICES=()

for PATTERN in "$@"; do
    # パターンにマッチするデバイス名を取得
    MATCHED_DEVICES=($(ls /sys/bus/usb-serial/devices/ | grep "$PATTERN" 2>/dev/null))

    if [ ${#MATCHED_DEVICES[@]} -eq 0 ]; then
        echo "Warning: No devices matching '$PATTERN' found."
    else
        DEVICES+=("${MATCHED_DEVICES[@]}")
    fi
done

for DEVICE in "${DEVICES[@]}"; do
    LATENCY_PATH="/sys/bus/usb-serial/devices/$DEVICE/latency_timer"
    if [ ! -e "$LATENCY_PATH" ]; then
        echo "Error: Device $DEVICE does not exist or does not support latency_timer."
        continue
    fi

    # 書き込み権限を一時的に設定
    sudo chmod a+w "$LATENCY_PATH"
    # 値を設定
    echo "$LATENCY" | sudo tee "$LATENCY_PATH" > /dev/null
    # 書き込み権限をもとに戻す
    sudo chmod a-w "$LATENCY_PATH"
    # 設定値を確認
    NEW_VALUE=$(cat "$LATENCY_PATH")
    echo "Latency timer for $DEVICE set to $NEW_VALUE ms."
done