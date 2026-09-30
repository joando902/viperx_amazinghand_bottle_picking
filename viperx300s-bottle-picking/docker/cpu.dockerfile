################################################################################################
# - Base stage
#   - This stage serves as the foundational stage for all other stages.
#   - Base image: Ubuntu 22.04
################################################################################################

FROM ros:humble AS base

LABEL org.opencontainers.image.title="Docker Image of Interbotix ROS XSarms (Ubuntu 22.04)"
LABEL org.opencontainers.image.authors="yoseph.huang@gmail.com"
LABEL org.opencontainers.image.licenses="MIT"

ENV TZ=Asia/Taipei
ENV LANG=en_US.UTF-8

ENV ROS_DISTRO=humble

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]


################################################################################################
# - User stage
#   - Add a non-root user with sudo privileges.
################################################################################################

FROM base AS user

ARG USERNAME=hrc
ARG USER_UID=1000
ARG USER_GID=1000

ENV USERNAME=${USERNAME}
ENV USERPATH=/home/${USERNAME}
ENV ROOTPATH=/home

## Create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    rm -rf /var/lib/apt/lists/*


################################################################################################
# - Interbotix stage
#   - Install Interbotix dependencies and Arm packages.
#   - ref: https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh.
################################################################################################

FROM user AS interbotix

# linux tools
RUN apt-get update && \
    apt-get install -y \
        git \
        gnupg \
        locales \
        lsb-release \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        wget && \
    rm -rf /var/lib/apt/lists/*

# python packages
RUN python3 -m pip install \
    transforms3d \
    modern_robotics \
    pyserial

# Keep the YOLO runtime in the image so recreating the container does not
# require reinstalling it. Use CPU PyTorch wheels to avoid CUDA dependencies.
RUN python3 -m pip install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision && \
    python3 -m pip install --no-cache-dir \
        "numpy<2" \
        "opencv-python<4.12" \
        ultralytics

# ros dependencies
WORKDIR /tmp

COPY apriltag_ws /tmp/apriltag_ws
COPY interbotix_ws /tmp/interbotix_ws

RUN cd /tmp/interbotix_ws && \
    apt update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    cd /tmp/apriltag_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/apriltag_ws /tmp/interbotix_ws 

################################################################################################
# - Release stage
#   - Set working directory.
################################################################################################

FROM interbotix AS release

WORKDIR /home/interbotix_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${ROOTPATH}/interbotix_ws/install/setup.bash" >> ~/.bashrc && \
    echo "source ${ROOTPATH}/apriltag_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

##################### 安裝 rosbridge #####################
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-rosbridge-server && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
