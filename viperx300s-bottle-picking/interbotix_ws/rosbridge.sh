#!/usr/bin/env bash
#
# 檔名：run_rosbridge.sh
# 功能：快速啟動 rosbridge_server 的 WebSocket 服務
# 使用方式：在終端機中執行 ./run_rosbridge.sh

# 1. 若有需要，可先設定 ROS_DOMAIN_ID，或其他環境變數
#    export ROS_DOMAIN_ID=0

# 2. 切換到 launch 檔所在目錄（視專案結構調整）
#    這裡假設你已經在包含 rosbridge_websocket_launch.xml 的目錄中執行此腳本
# cd /path/to/your/ros2_ws/src/rosbridge_suite/launch

echo "🟢 開始啟動 rosbridge_server WebSocket 服務..."
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
