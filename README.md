# ViperX 300S + AmazingHand Bottle Picking

This project integrates the **ViperX 300S robotic arm**, **AmazingHand**, **Intel RealSense camera**, and **YOLOv8** to detect and pick bottles.

The system uses:

- ViperX 300S robotic arm
- AmazingHand robotic hand
- Intel RealSense camera
- YOLOv8 object detection
- ROS 2
- MoveIt
- Docker

---

# 1. AmazingHand

## Start AmazingHand Docker

Start the AmazingHand Docker container:

```bash
docker start amazing_hand_container
```

Enter the container and set the ROS domain ID:

```bash
docker exec -it -e ROS_DOMAIN_ID=77 amazing_hand_container bash
```

## Start AmazingHand

Inside the AmazingHand Docker container, launch the hand:

```bash
ros2 launch paxini_tactile amazing_hand.launch.py
```

Keep this terminal running.

---

# 2. ViperX 300S

Go to the ViperX ROS 2 project directory and start the Docker development environment:

```bash
./cpu_run.sh dev
```

Then open another terminal and enter the ViperX Docker container:

```bash
docker exec -it dev bash
```

All of the following ViperX commands should be executed inside the `dev` Docker container.

---

# Terminal 1 — RealSense Camera

Start the Intel RealSense camera with aligned depth:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true
```

Keep this terminal running.

---

# Terminal 2 — ViperX 300S + MoveIt

Start the ViperX 300S robotic arm and MoveIt:

```bash
ros2 launch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  use_moveit_rviz:=true \
  use_moveit_interface_gui:=false
```

RViz will open and display the ViperX 300S robot.

Keep this terminal running.

---

# Terminal 3 — YOLOv8 Bottle Detection

Start YOLOv8 bottle detection:

```bash
ros2 launch interbotix_xsarm_perception bottle_detection.launch.py \
  model_path:=/home/interbotix_ws/src/yolov8n.pt
```

This node detects bottles from the RealSense camera image.

Keep this terminal running.

---

# Terminal 4 — Coordinate Transformation

Start the hand-eye coordinate transformation:

```bash
ros2 launch interbotix_xsarm_perception hand_eye.launch.py
```

This node transforms the detected bottle position from the camera coordinate frame to the robot coordinate frame.

Keep this terminal running.

---

# Terminal 5 — Bottle Picking

Run the bottle picking program:
(The bottle can be positioned 10 cm up, 10 cm to the left, and 10 cm down. )
```bash
ros2 run interbotix_xsarm_perception viperx_amazing_hand_bottle_pick.py --ros-args \
  -p dry_run:=false \
  -p pregrasp_only:=false \
  -p grasp_pitch:=0.0 \
  -p lift_distance:=0.10 \
  -p place_left_distance:=0.10 \
  -p place_lower_distance:=0.10 \
  -p post_grasp_velocity_scale:=0.25
```

The robot will use the detected bottle position to perform the picking motion with the AmazingHand.

---

# Recommended Startup Order

Start the system in the following order:

1. AmazingHand Docker
2. AmazingHand ROS 2 node
3. ViperX Docker
4. RealSense Camera
5. ViperX 300S + MoveIt
6. YOLOv8 Bottle Detection
7. Coordinate Transformation
8. Bottle Picking Program

---

# ROS Domain ID

The AmazingHand container uses:

```bash
ROS_DOMAIN_ID=77
```

Make sure that ROS 2 nodes that need to communicate with AmazingHand are configured to use the same ROS domain ID when required.
