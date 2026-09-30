# ViperX300S Bottle Picking

This repository contains a bottle detection and robotic grasping system for the **Interbotix ViperX 300S** robotic arm.

The system uses:

- ViperX 300S
- Intel RealSense RGB-D Camera
- ROS 2
- MoveIt 2
- YOLOv8
- Hand-Eye Calibration
- TF Coordinate Transformation
- Gripper Control

The system detects a bottle using the RealSense camera, obtains the bottle position, transforms the position into the robot coordinate frame, and controls the ViperX 300S to grasp the bottle.

---

# 1. Download the Repository

Clone the repository from GitHub:

```bash
git clone https://github.com/joando902/viperx300s-bottle-picking.git
```

Enter the project folder:

```bash
cd viperx300s-bottle-picking
```

---

# 2. Build

Build the ROS 2 environment:

```bash
./cpu_run.sh build
```

---

# 3. Start Docker

Start the Docker development environment:

```bash
./cpu_run.sh dev
```

Open another terminal and enter the Docker container:

```bash
docker exec -it dev bash
```

---

# 4. Motor Calibration

Before using the robotic arm, confirm that the motor positions are calibrated correctly.

Use **DYNAMIXEL Wizard 2.0** to check the Present Position of each motor and set the required Homing Offset.

The goal is to make the physical ViperX 300S position match the robot position shown in RViz.

After calibration, confirm that:

```text
Physical Robot Position
        ≈
RViz Robot Position
```

---

# 5. Camera / Hand-Eye Calibration

Before running the bottle picking system, the RealSense camera position relative to the ViperX 300S must be calibrated.

The hand-eye calibration is used to obtain the transformation between:

```text
Camera Coordinate Frame
        ↓
Robot Base Coordinate Frame
```

After calibration, the transformation result is used by the system to convert the detected bottle position into the ViperX 300S coordinate frame.

Make sure the camera position has not changed after calibration.

---

# 6. Run the System

The complete system uses five terminals.

Recommended startup order:

```text
Terminal 1 → RealSense Camera
Terminal 2 → ViperX 300S + MoveIt
Terminal 3 → YOLOv8 Bottle Detection
Terminal 4 → Coordinate Transformation
Terminal 5 → Bottle Picking
```

Each terminal should first enter the Docker container:

```bash
docker exec -it dev bash
```

---

## Terminal 1 — RealSense Camera

Start the RealSense camera.

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true
```

---

## Terminal 2 — ViperX 300S + MoveIt

Start the ViperX 300S robotic arm and MoveIt.

```bash
ros2 launch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  use_moveit_rviz:=true \
  use_moveit_interface_gui:=false
```

---

## Terminal 3 — YOLOv8 Bottle Detection

Start YOLOv8 bottle detection.

```bash
ros2 launch interbotix_xsarm_perception bottle_detection.launch.py \
  model_path:=/home/interbotix_ws/src/yolov8n.pt
```

---

## Terminal 4 — Coordinate Transformation

Start the hand-eye coordinate transformation.

```bash
ros2 launch interbotix_xsarm_perception hand_eye.launch.py
```

---

## Terminal 5 — Bottle Picking

Start the bottle picking program.

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

After grasping the bottle, the end effector rises 10 cm, moves 10 cm to the
robot's left (`+Y` in `vx300s/base_link`), lowers 10 cm, and sends `open` to
the Amazing Hand controller. The post-grasp Cartesian motion runs at 25% of
the configured joint velocity.

---

# 7. System Flow

```text
Download Repository
        ↓
Build
        ↓
Start Docker
        ↓
Motor Calibration
        ↓
Camera / Hand-Eye Calibration
        ↓
Start RealSense Camera
        ↓
Start ViperX 300S + MoveIt
        ↓
Start YOLOv8
        ↓
Start Coordinate Transformation
        ↓
Start Bottle Picking
        ↓
Robot Grasps Bottle
```

---

# Terminal Summary

| Terminal | Function |
|---|---|
| Terminal 1 | Start RealSense camera |
| Terminal 2 | Start ViperX 300S and MoveIt |
| Terminal 3 | Start YOLOv8 bottle detection |
| Terminal 4 | Start coordinate transformation |
| Terminal 5 | Start bottle picking |

---

# Author

GitHub: `joando902`
