#!/bin/bash
xhost +local:root

# 自動建置映像檔
docker build -t amazing_hand:humble .

# 啟動並進入容器 (掛載當前目錄)
docker run --rm -it \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=${DISPLAY} \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v $(pwd):/root/amazing_hand \
  --name amazing_hand_container \
  amazing_hand:humble /bin/bash
