FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 1. 安裝系統開發工具、序列通訊、OpenGL 以及 Qt6/XCB 圖形依賴
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    python3-serial \
    python3-colcon-common-extensions \
    python3-rosdep \
    udev \
    git \
    nano \
    usbutils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxkbcommon-x11-0 \
    libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

# 2. 升級 pip 與 build 工具
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel build

# 3. 安裝手掌馬達控制與 Paxini 視覺化所需的 Python 相依套件
RUN pip3 install --no-cache-dir \
    numpy \
    pyyaml \
    pyserial \
    PyQt5 \
    PyQt6 \
    pyqtgraph \
    dynamixel-sdk \
    rustypot

# 4. 設定 ROS 2 工作空間與全域 Python 搜尋路徑
ENV WORKSPACE=/root/amazing_hand
ENV PYTHONPATH="${WORKSPACE}/amazing_hand/midas_hand_api:${WORKSPACE}/midas_hand_api:${WORKSPACE}:${WORKSPACE}/amazing_hand:${PYTHONPATH}"
WORKDIR $WORKSPACE


RUN echo "/root/amazing_hand/amazing_hand/midas_hand_api" > /usr/local/lib/python3.10/dist-packages/midas_hand_api.pth && \
    echo "/root/amazing_hand/midas_hand_api" >> /usr/local/lib/python3.10/dist-packages/midas_hand_api.pth

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/amazing_hand/install/setup.bash ]; then source /root/amazing_hand/install/setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /root/amazing_hand/amazing_hand/install/setup.bash ]; then source /root/amazing_hand/amazing_hand/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
