##########################################################################
##########################################################################
# This is a concise version of viperX300s ROS 2 Humble installation script.  
# For more details, please refer to the original script
# at https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh.
##########################################################################

#!/usr/bin/env bash

set -e

echo "===== 環境設定: amd64, ROS 2 Humble, Python 3, 安裝 MATLAB 與 Perception 套件 ====="

# 1. 建立 Interbotix 工作區
INSTALL_PATH=~/interbotix_ws
mkdir -p $INSTALL_PATH/src
cd $INSTALL_PATH/src

# 2. 下載 Interbotix 套件
echo "下載 Interbotix ROS 2 套件..."
git clone -b humble https://github.com/Interbotix/interbotix_ros_core.git
git clone -b humble https://github.com/Interbotix/interbotix_ros_manipulators.git
git clone -b humble https://github.com/Interbotix/interbotix_ros_toolboxes.git

# 3. 移除 CATKIN_IGNORE（ROS 2 預設使用 COLCON）
rm -f interbotix_ros_core/interbotix_ros_xseries/COLCON_IGNORE
rm -f interbotix_ros_manipulators/interbotix_ros_xsarms/COLCON_IGNORE
rm -f interbotix_ros_toolboxes/interbotix_xs_toolbox/COLCON_IGNORE
rm -f interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE

# 4. 安裝 Perception 模組
echo "啟用 Perception 模組..."
rm -f interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/COLCON_IGNORE

# 5. 安裝 Python 套件
pip3 install modern_robotics transforms3d

# 6. 安裝 MATLAB
echo "啟用 Matlab 模組..."
 cd interbotix_ros_toolboxes
git submodule update --init third_party_libraries/ModernRobotics

# 7. 編譯工作區
cd $INSTALL_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc

echo "✅ 安裝完成：Interbotix ROS 2 Humble + Perception 模組"