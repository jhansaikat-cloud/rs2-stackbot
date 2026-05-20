# SS1 — Perception & Pose Estimation

**Subsystem:** SS1  
**Student:** Arjun Harish (25533997)  
**Project:** Precision Robotic Assembly with UR3e — Stackbot  
**Course:** 41069 Robotics Studio 2, UTS  

---

## Overview

SS1 detects coloured cubes in the robot workspace and publishes their 3D poses in the `base_link` frame. It uses an Intel RealSense D435i camera mounted on the UR3e wrist, OpenCV HSV colour segmentation, and a hand-eye calibration transform to convert camera-frame detections into robot-frame coordinates.

The pyramid uses 6 cubes across 3 colours:

| Layer  | Colour | Count |
|--------|--------|-------|
| Bottom | Red    | 3     |
| Middle | Yellow | 2     |
| Top    | Blue   | 1     |

---

## Hardware

| Component | Details |
|-----------|---------|
| Robot     | UR3e — IP: 192.168.0.194 |
| Camera    | Intel RealSense D435i — wrist mounted (eye-in-hand) |
| Mount     | Between tool0 flange and RG2-FT gripper |
| OS        | Ubuntu 22.04 |
| Framework | ROS2 Humble |

> **Camera serial changes every session.** Check serial at start of each session and redo hand-eye calibration if it has changed (see below).

---

## Dependencies

```bash
# RealSense ROS2 driver
sudo apt install ros-humble-realsense2-camera

# ArUco marker detection
sudo apt install ros-humble-aruco-opencv

# TF2 geometry messages
sudo apt install ros-humble-tf2-geometry-msgs

# CV bridge
sudo apt install ros-humble-cv-bridge

# easy_handeye2 (hand-eye calibration)
cd ~/rs2-stackbot/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ~/rs2-stackbot && colcon build --packages-select easy_handeye2 easy_handeye2_msgs
```

Set ROS domain ID in `~/.bashrc` on every laptop:

```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
cd ~/rs2-stackbot
colcon build --packages-select perception_pkg
source install/setup.bash
```

---

## Full System Run Order

Open terminals in this order. Do not skip steps.

| Terminal | Command |
|----------|---------|
| T1 | `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false` |
| T2 | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 kinematics_params:=${HOME}/ur3e_calibration_194.yaml launch_rviz:=false` |
| T3 | `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true` |
| T4 | `ros2 launch easy_handeye2 publish.launch.py name:=ur3e_eye_on_hand` |
| T5 | `ros2 run perception_pkg pose_estimator` |

> **Important:** Always use `publish_tf:=false` in T1. Always use `ur3e_calibration_194.yaml` in T2 (not `ur3e_calibration_new.yaml`).

---

## SS1 Nodes

| Node | Command | Purpose |
|------|---------|---------|
| pose_estimator | `ros2 run perception_pkg pose_estimator` | Main node — detects cubes and publishes 3D poses |
| cube_detector | `ros2 run perception_pkg cube_detector` | Standalone colour detection debug |
| hsv_tuner | `ros2 run perception_pkg hsv_tuner` | Interactive HSV mask tuning |
| camera_viewer | `ros2 run perception_pkg camera_viewer` | Raw camera feed viewer |
| webcam_hsv_tuner | `python3 ~/rs2-stackbot/src/perception_pkg/perception_pkg/webcam_hsv_tuner.py` | Standalone HSV tuner (no ROS) |
| webcam_detector | `python3 ~/rs2-stackbot/src/perception_pkg/perception_pkg/webcam_detector.py` | Standalone detector (no ROS) |

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ss1/raw_detected_objects` | `geometry_msgs/PoseArray` | 3D poses of detected cubes in `base_link` frame |
| `/object_labels` | `std_msgs/String` | Comma-separated colour names matching pose indices |

**`/ss1/raw_detected_objects`**
- `frame_id`: `base_link`
- Poses published in consistent order: red → yellow → blue
- Within each colour, sorted by Y position
- `orientation.z` and `orientation.w` encode yaw (rotation about Z axis)
- Last known pose held for 2 seconds after cube leaves camera view

**`/object_labels`**
- Example for full pyramid: `red,red,red,yellow,yellow,blue`

> SS3 subscribes to both these topics. SS3 validates that exactly 6 poses are present (3 red, 2 yellow, 1 blue) before forwarding to SS2.

---

## Camera Intrinsics

Check at the start of every session:

```bash
# Check serial number
rs-enumerate-devices | grep Serial

# Check intrinsics
ros2 topic echo /camera/camera/color/camera_info --once
```

If the serial has changed from last session, update `FX`, `FY`, `CX`, `CY` in `pose_estimator.py` and redo hand-eye calibration.

**Current camera (Session 5):**

| Parameter | Value |
|-----------|-------|
| Serial | 027322070904 |
| FX | 917.625244140625 |
| FY | 915.7077026367188 |
| CX | 636.296875 |
| CY | 351.4231872558594 |

---

## HSV Tuning Values

Tuned under warm indoor lab lighting at UTS:

| Colour | H min | H max | S min | S max | V min | V max | Notes |
|--------|-------|-------|-------|-------|-------|-------|-------|
| Red (low) | 0 | 10 | 160 | 255 | 150 | 255 | Two ranges needed — red wraps at 0/180 |
| Red (high) | 165 | 180 | 160 | 255 | 150 | 255 | Warm lighting shifts background toward red |
| Yellow | 18 | 33 | 80 | 255 | 120 | 255 | |
| Blue | 98 | 108 | 180 | 255 | 90 | 200 | |

Retune at the start of each session if lighting has changed:

```bash
ros2 run perception_pkg hsv_tuner
```

---

## Detection Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| MIN_AREA | 2000 px | Filters small noise detections |
| MIN_SQUARENESS | 0.45 | Lowered from 0.65 — perspective distortion at frame edges makes cubes appear non-square |
| BUFFER_SIZE | 3 frames | Smoothing buffer per cluster |
| DEPTH_WINDOW | 20 px | Robust median depth sampling |
| POSE_TIMEOUT | 2.0 s | Holds last known pose after cube leaves view |
| CLUSTER_DIST | 5 cm | Spatial distance to separate multiple cubes of same colour |

---

## Hand-Eye Calibration

Calibration file location:

```
~/.ros2/easy_handeye2/calibrations/ur3e_eye_on_hand.calib
```

**Must redo calibration if the camera serial changes between sessions.**

### Calibration Run Order

| Terminal | Command |
|----------|---------|
| T1 | `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false` |
| T2 | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 kinematics_params:=${HOME}/ur3e_calibration_194.yaml launch_rviz:=false` |
| T3 | `ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=camera/camera/color/image_raw -p marker_dict:=6X6_250 -p marker_size:=0.021 -p publish_tf:=true -p aruco.cornerRefinementMethod:=2` |
| T4 | `ros2 launch easy_handeye2 calibrate.launch.py name:=ur3e_eye_on_hand` |

### Charuco Board Specs

| Parameter | Value |
|-----------|-------|
| Columns x Rows | 9 x 6 |
| Checker size | 28mm |
| Marker size | 21mm |
| Dictionary | 6X6_250 |
| Board image | `~/charuco_board_9x6.png` |

### Sampling Procedure

1. Place Charuco board flat on table — do not move it during calibration
2. Use teach pendant freedrive to move arm so camera sees the board
3. Wait for stable green boxes in ArUco debug view
4. Click **Take Sample** in the easy_handeye2 rqt GUI
5. Repeat 15–20 times with varied arm positions and wrist rotations
6. Click **Compute** — verify result looks reasonable (rotation `w > 0.95`)
7. Click **Save**

### Key Calibration Rules

- Use `marker_11` (central marker) not `marker_0` — more stable
- Use `camera_color_optical_frame` as tracking_base_frame not `camera_link`
- Use `aruco.cornerRefinementMethod:=2` to prevent marker pose flipping
- `base` and `base_link` are NOT the same frame — 180° rotation between them

---

## TF Transform Chain

```
base_link → tool0                       (UR driver — live joint states)
tool0 → camera_color_optical_frame      (hand-eye calibration — easy_handeye2 publisher)
camera_color_optical_frame → cube       (pose_estimator depth calculation)
──────────────────────────────────────────────────────────────────────────
base_link → cube                        (published on /ss1/raw_detected_objects)
```

---

## Diagnostic Commands

```bash
# Check poses publishing
ros2 topic echo /ss1/raw_detected_objects
ros2 topic echo /object_labels
ros2 topic hz /ss1/raw_detected_objects

# Check TF chain is connected
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# View full TF tree
ros2 run tf2_tools view_frames && evince frames.pdf

# List all active topics
ros2 topic list

# Open image viewer
ros2 run rqt_image_view rqt_image_view
```

---

## base vs base_link Frame

The pendant reads TCP position in `base` frame. ROS uses `base_link`. They are NOT the same.

```
base_link.x = -base.x
base_link.y = -base.y
base_link.z =  base.z
```

Use this when comparing pendant TCP position to pose_estimator output during accuracy testing.

---

## Common Issues

| Issue | Cause | Fix |
|-------|-------|-----|
| "no TF" shown on cubes | TF tree disconnected | Run `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` — relaunch T2/T4 |
| Two disconnected TF trees | RealSense publishing its own TF | Always launch camera with `publish_tf:=false` |
| UR driver crash — speed_slider_mask error | Another RTDE client connected | Stop pendant program, run `pkill -f ur_ros2_control_node`, relaunch |
| TF extrapolation warning | Timestamp mismatch | Use `rclpy.time.Time().to_msg()` not `colour_msg.header.stamp` |
| Take Sample button flickering | Marker pose unstable | Use `aruco.cornerRefinementMethod:=2`, use marker_11 |
| Detection flickering | HSV mask unstable | Retune with `ros2 run perception_pkg hsv_tuner` |
| `poses: []` even when detecting | Buffer not filled yet | Wait for 3 consecutive frames or reduce BUFFER_SIZE temporarily |
| Duplicate package build error | Two copies of perception_pkg | `rm -rf ~/rs2-stackbot/src/ss1_perception` then rebuild |
| Cube not detected at frame edges | Perspective distortion | MIN_SQUARENESS already lowered to 0.45 — lower further if needed |
| Red detection unreliable | Warm lighting shifts background toward red | Raise S min, keep dual H range (0–10 and 165–180) |

---

## Grading Accuracy Targets

| Grade | Position Accuracy | Notes |
|-------|------------------|-------|
| Pass | ±15mm | Manual calibration allowed |
| C | ±10mm | ±10° orientation, stable lighting |
| D | ±8mm | Moderate lighting variation |
| HD | ±5mm | Handles partial occlusions |
| Perfect | ±2mm | Continuous tracking, auto re-detection |
