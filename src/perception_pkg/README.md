# SS1 — Perception & Pose Estimation

**Subsystem:** SS1
**Student:** Arjun Harish (25533997)
**Project:** Precision Robotic Assembly with UR3e — Stackbot
**Course:** 41069 Robotics Studio 2, UTS

---

## Overview

SS1 detects coloured cubes in the robot workspace and publishes their 3D poses in the `base_link` frame. It uses an Intel RealSense D435i camera mounted on the UR3e wrist, OpenCV HSV colour segmentation, IoU-NMS filtering, and a hand-eye calibration transform to convert camera-frame detections into robot-frame coordinates.

The pyramid uses 6 cubes across 3 colours:

| Layer | Colour | Count |
|---|---|---|
| Bottom | Red | 3 |
| Middle | Yellow | 2 |
| Top | Blue | 1 |

---

## Hardware

| Component | Details |
|---|---|
| Robot | UR3e — IP: 192.168.0.192 |
| Camera | Intel RealSense D435i — wrist mounted (eye-in-hand) |
| Gripper | OnRobot RG2-FT — mounted below camera |
| OS | Ubuntu 22.04 |
| Framework | ROS2 Humble, Python 3.10, OpenCV 4.5.4 |

> Camera serial changes every session. Check serial at start of each session and redo hand-eye calibration if it has changed (see below).

---

## Dependencies

```bash
# TF transformations
sudo apt install ros-humble-tf-transformations

# TF2 geometry messages
sudo apt install ros-humble-tf2-geometry-msgs

# CV bridge
sudo apt install ros-humble-cv-bridge

# RealSense ROS2 driver
sudo apt install ros-humble-realsense2-camera
```

---

## Build

```bash
cd ~/ros2_ws_backup
colcon build --packages-select perception_pkg
source install/setup.bash
```

---

## Full System Run Order

Open terminals in this order. Do not skip steps.

| Terminal | Command |
|---|---|
| T1 | `ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30 depth_module.profile:=640x480x30 enable_depth:=true align_depth.enable:=true publish_tf:=false` |
| T2 | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.192 kinematics_params_file:=/home/arjun/my_robot_calibration_192.yaml launch_rviz:=false` |
| T3 | `ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py ur_type:=ur3e onrobot_type:=rg2` |
| T4 | `ros2 run perception_pkg publish_handeye_tf --method park` |
| T5 | `ros2 run perception_pkg pose_estimator_v2` |

> **Important:** Always use `publish_tf:=false` in T1. Always use the full path `/home/arjun/` not `~/` for `kinematics_params_file` in T2. T3 is only needed for the full team pipeline.

---

## SS1 Nodes

| Node | Command | Purpose |
|---|---|---|
| pose_estimator_v2 | `ros2 run perception_pkg pose_estimator_v2` | Main node — detects cubes and publishes 3D poses |
| publish_handeye_tf | `ros2 run perception_pkg publish_handeye_tf --method park` | Publishes hand-eye calibration TF to the system |
| charuco_handeye_calibration | `ros2 run perception_pkg charuco_handeye_calibration` | Run once to calibrate camera to robot |
| charuco_handeye_validate | `ros2 run perception_pkg charuco_handeye_validate --method park` | Validates calibration accuracy |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/raw_detected_objects` | `geometry_msgs/PoseArray` | 3D poses of detected cubes in `base_link` frame |
| `/object_labels` | `std_msgs/String` | Comma-separated colour names matching pose indices |

### /raw_detected_objects
- `frame_id: base_link`
- Poses published in consistent order: red → yellow → blue
- Within each colour, sorted by X position (left → right)
- `orientation.z` and `orientation.w` encode yaw (rotation about Z axis)
- Last known pose held for 2 seconds after cube leaves camera view

### /object_labels
- Example for full pyramid: `red,red,red,yellow,yellow,blue`

> **SS3 note (Saikat):** Subscribe to `/raw_detected_objects` — topic was renamed from `/ss1/raw_detected_objects` in Session 7. SS3 validates that exactly 6 poses are present (3 red, 2 yellow, 1 blue) before forwarding to SS2.

---

## Camera Intrinsics

Check at the start of every session:

```bash
# Check serial number
rs-enumerate-devices | grep Serial

# Check intrinsics
ros2 topic echo /camera/camera/color/camera_info --once
```

If the serial has changed from last session, update `FX`, `FY`, `CX`, `CY` in `pose_estimator_v2.py` and redo hand-eye calibration.

Current camera (Session 7):

| Parameter | Value |
|---|---|
| Serial | 017322075373 |
| FX | 917.7413940429688 |
| FY | 917.1574096679688 |
| CX | 645.986572265625 |
| CY | 349.41644287109375 |

---

## HSV Tuning Values

Tuned under warm indoor lab lighting at UTS:

| Colour | H min | H max | S min | S max | V min | V max | Notes |
|---|---|---|---|---|---|---|---|
| Red (low) | 0 | 10 | 120 | 255 | 100 | 255 | Two ranges needed — red wraps at 0/180 |
| Red (high) | 165 | 180 | 120 | 255 | 100 | 255 | |
| Yellow | 18 | 33 | 80 | 255 | 120 | 255 | |
| Blue | 98 | 108 | 180 | 255 | 90 | 200 | |

---

## Detection Parameters

| Parameter | Value | Purpose |
|---|---|---|
| `MIN_AREA` | 2000 px | Filters small noise detections |
| `MIN_SQUARENESS` | 0.40 | Rejects non-square shapes (perspective distortion at frame edges) |
| `IOU_THRESHOLD` | 0.30 | Suppresses overlapping same-object detections |
| `DEPTH_WINDOW` | 20 px | Primary depth sample patch half-size |
| `DEPTH_WINDOW_FB` | 40 px | Fallback depth sample patch half-size |
| `EMA_ALPHA` | 0.4 | Smoothing factor (higher = faster response, noisier) |
| `POSE_TIMEOUT` | 2.0 s | Drops stale pose after cube leaves view |

---

## Hand-Eye Calibration

Calibration files location:
```
~/ros2_ws_backup/src/perception_pkg/calibration/
├── handeye_charuco_park.json       ← USE THIS (1.84mm std)
├── handeye_charuco_tsai.json
├── handeye_charuco_daniilidis.json
├── handeye_charuco_horaud.json
└── handeye_charuco_andreff.json
```

Must redo calibration if the camera serial changes between sessions.

### Calibration Run Order

| Terminal | Command |
|---|---|
| T1 | Camera launch (same as Full System T1) |
| T2 | Robot driver launch (same as Full System T2) |
| T3 | `ros2 run perception_pkg charuco_handeye_calibration` |
| T4 | `ros2 run perception_pkg charuco_handeye_validate --method park` |

### ChArUco Board Specs

| Parameter | Value |
|---|---|
| Columns x Rows | 9 x 6 |
| Square size | 28mm |
| Marker size | 21mm |
| Dictionary | 6X6_250 |

### Sampling Procedure

1. Place ChArUco board flat on table — do not move it during calibration
2. Move arm so camera sees the board from different angles
3. Press `SPACE` to capture a sample
4. Repeat 15–20 times with varied arm positions and wrist rotations
5. Press `S` to solve — pick the method with lowest std deviation
6. Validate: Z must be positive and between 50–500mm

### Session 7 Calibration Results

| Method | Std Dev |
|---|---|
| Park | 1.84mm ✅ |
| Tsai | 1.84mm |
| Daniilidis | 1.84mm |
| Horaud | 1.84mm |
| Andreff | 1.86mm |

Validation result: `X=144.4mm  Y=404.5mm  Z=24.3mm` — variation < 0.1mm ✅

---

## TF Transform Chain

```
base_link → tool0                       (UR driver — live joint states)
tool0 → camera_color_optical_frame      (publish_handeye_tf --method park)
camera_color_optical_frame → cube       (pose_estimator_v2 depth projection)
──────────────────────────────────────────────────────────────────────────
base_link → cube                        (published on /raw_detected_objects)
```

---

## Diagnostic Commands

```bash
# Check poses publishing
ros2 topic echo /raw_detected_objects
ros2 topic echo /object_labels
ros2 topic hz /raw_detected_objects

# Check TF chain is connected
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# View full TF tree
ros2 run tf2_tools view_frames && evince frames.pdf

# List all active topics
ros2 topic list
```

---

## Testing Without a Robot

Run a fake TF instead of the real robot driver (Terminal 2):

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.5 \
  --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id base_link \
  --child-frame-id tool0
```

Then run Terminals 1, 4, and 5 as normal. Positions won't be physically accurate but detection and labels will work.

---

## Common Issues

| Issue | Cause | Fix |
|---|---|---|
| `no TF` shown on cubes | TF tree disconnected | Run `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` — relaunch T2/T4 |
| Two disconnected TF trees | RealSense publishing its own TF | Always launch camera with `publish_tf:=false` |
| Camera window not opening | `DISPLAY` not set | `export DISPLAY=:1` then rerun |
| Missing yellow cube | Depth invalid at that pixel (USB bandwidth) | Plug into USB 3.0 (blue) port; increase `DEPTH_WINDOW_FB` to 60 |
| Poses have negative Z | Hand-eye calibration wrong | Redo calibration — Z must be positive |
| `tilde not expanding` in launch | Shell not expanding `~/` | Use `/home/arjun/` not `~/` for `kinematics_params_file` |
| TF extrapolation warning | Timestamp mismatch | Use `rclpy.time.Time().to_msg()` not `colour_msg.header.stamp` |

---

## Grading Accuracy Targets

| Grade | Position Accuracy |
|---|---|
| Pass | ±15mm |
| Credit | ±10mm |
| Distinction | ±8mm |
| High Distinction | ±5mm |

Calibration internal consistency: **1.84mm** — within HD range ✅
