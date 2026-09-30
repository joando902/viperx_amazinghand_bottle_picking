# VX300s AprilTag continuous detection

This workspace contains a ready-to-run `apriltag_ros` configuration for the
RealSense topics used by the VX300s project. It detects a `tag36h11` tag,
publishes its pose as TF, and opens RViz2 with TF and the annotated image shown.

## 1. Build once inside the development container

```bash
cd /home/apriltag_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2. Start the camera

In terminal A inside the container:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

Confirm that the paired topics exist:

```bash
ros2 topic list | grep /camera/camera/color
ros2 topic hz /camera/camera/color/image_raw
```

## 3. Start AprilTag detection and RViz2

In terminal B inside the container, replace `tag_size` with the measured side
length of the black/white boundary on the printed tag (metres):

```bash
source /opt/ros/humble/setup.bash
source /home/apriltag_ws/install/setup.bash
ros2 launch apriltag_ros vx300s_apriltag.launch.py tag_id:=0 tag_size:=0.040
```

When tag 0 is visible, RViz2's TF display shows this transform:

```text
camera_color_optical_frame -> tag_0
```

Verify it independently from RViz2:

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag_0
ros2 topic echo /apriltag/detector/tag_detections --once
```

If the camera uses different topics, override both together:

```bash
ros2 launch apriltag_ros vx300s_apriltag.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  tag_id:=0 tag_size:=0.040
```

The defaults live in
`src/apriltag_ros/apriltag_ros/config/vx300s_tags.param.yaml`. The tag family,
ID, and physical size must match the printed card. Incorrect size produces an
incorrect translation scale even when detection succeeds.
