# SS1 — Perception & Pose Estimation

**Subsystem:** SS1  
**Student:** Arjun Harish (25533997)  
**Project:** Precision Robotic Assembly with UR3e — Stackbot  
**Course:** 41069 Robotics Studio 2, UTS  

---

## Overview

SS1 is responsible for detecting coloured cubes in the robot workspace and publishing their 3D poses in the `base_link` frame. It uses an Intel RealSense D435i camera mounted on the UR3e wrist, OpenCV HSV colour segmentation, and a hand-eye calibration transform to convert camera-frame detections into robot-frame coordinates.

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
| Robot | UR3e — IP: `192.168.0.194` |
| Camera | Intel RealSense D435i — Serial: `244622072177` |
| Mount | Wrist mounted (eye-in-hand) |
| OS | Ubuntu 22.04 |
| Framework | ROS2 Humble |

---

## Dependencies

Install the following before building:

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
cd ~/ros2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
cd ~/ros2_ws && colcon build --packages-select easy_handeye2 easy_handeye2_msgs
```

Also ensure `ROS_DOMAIN_ID=42` is set in `~/.bashrc`:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select perception_pkg
source ~/.bashrc
```

---

## SS1 Nodes

| Node | Command | Purpose |
|---|---|---|
| `pose_estimator` | `ros2 run perception_pkg pose_estimator` | Main node — detects cubes and publishes 3D poses |
| `cube_detector` | `ros2 run perception_pkg cube_detector` | Standalone colour detection debug |
| `hsv_tuner` | `ros2 run perception_pkg hsv_tuner` | Interactive HSV mask tuning |
| `camera_viewer` | `ros2 run perception_pkg camera_viewer` | Raw camera feed viewer |
| `webcam_hsv_tuner` | `python3 ~/ros2_ws/src/perception_pkg/perception_pkg/webcam_hsv_tuner.py` | Standalone HSV tuner (no ROS) |
| `webcam_detector` | `python3 ~/ros2_ws/src/perception_pkg/perception_pkg/webcam_detector.py` | Standalone detector (no ROS) |

---

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/detected_objects` | `geometry_msgs/PoseArray` | 3D poses of detected cubes in `base_link` frame |
| `/object_labels` | `std_msgs/String` | Comma-separated colour names matching pose indices e.g. `red,yellow,blue` |

### Topic Details

**`/detected_objects`**
- `frame_id`: `base_link`
- Poses published in consistent order: red first, yellow second, blue third
- Within each colour, sorted left-to-right by Y position
- `orientation.z` and `orientation.w` encode yaw (rotation about Z axis)
- Last known pose is held for 2 seconds after cube leaves camera view

**`/object_labels`**
- Comma-separated string matching the pose indices in `/detected_objects`
- Example: `red,red,red,yellow,yellow,blue` for full pyramid

---

## HSV Tuning Values

Tuned under lab lighting at UTS:

| Colour | H min | H max | S min | S max | V min | V max |
|---|---|---|---|---|---|---|
| Red | 0 | 10 | 150 | 255 | 110 | 210 |
| Yellow | 10 | 35 | 100 | 255 | 80 | 255 |
| Blue | 98 | 115 | 170 | 255 | 40 | 255 |

To retune under different lighting:
```bash
ros2 run perception_pkg hsv_tuner
```

---

## Camera Intrinsics

Read from `/camera/camera/color/camera_info` at 1280x720:

| Parameter | Value |
|---|---|
| fx | 908.159 |
| fy | 907.398 |
| cx | 647.846 |
| cy | 366.289 |

Launch command:
```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false
```

> **Important:** Always use `publish_tf:=false` to avoid conflicting with the hand-eye calibration TF tree.

---

## Hand-Eye Calibration

### Calibration file location
```
~/.ros2/easy_handeye2/calibrations/ur3e_eye_on_hand.calib
```

### Calibration result (current)
```yaml
transform:
  translation:
    x: -0.022325774896365072
    y: -0.08846779863561084
    z: 0.0074559732797811785
  rotation:
    x: -0.06915750778858583
    y: -0.012806931075589945
    z: 0.021085754620804593
    w: 0.9973006630825871
```

### To redo calibration

**Step 1 — Launch all required nodes:**

| Terminal | Command |
|---|---|
| T1 | `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false` |
| T2 | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 kinematics_params:=${HOME}/ur3e_calibration_194.yaml launch_rviz:=false` |
| T3 | `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true` |
| T4 | `ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=camera/camera/color/image_raw -p marker_dict:=6X6_250 -p marker_size:=0.021 -p publish_tf:=true -p aruco.cornerRefinementMethod:=2` |
| T5 | Verify TF: `ros2 run tf2_ros tf2_echo camera_color_optical_frame marker_11` |
| T6 | `ros2 launch easy_handeye2 calibrate.launch.py name:=ur3e_eye_on_hand calibration_type:=eye_in_hand robot_base_frame:=base_link robot_effector_frame:=tool0 tracking_base_frame:=camera_color_optical_frame tracking_marker_frame:=marker_11` |

**Step 2 — Charuco board specs:**
| Parameter | Value |
|---|---|
| Columns x Rows | 9 x 6 |
| Checker size | 28mm |
| Marker size | 21mm |
| Dictionary | 6X6_250 |

**Step 3 — Sampling procedure:**
1. Place Charuco board flat on table — do not move it during calibration
2. Use teach pendant freedrive to move arm to position where camera sees the board
3. Wait for stable green boxes in ArUco debug view
4. Click **Take Sample** in the easy_handeye2 rqt GUI
5. Repeat 15–20 times with varied arm positions and wrist rotations
6. Click **Compute**, verify result looks reasonable (rotation w > 0.95)
7. Click **Save**

---

## Full System Run Order

| Terminal | Command |
|---|---|
| T1 | `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 enable_depth:=true publish_tf:=false` |
| T2 | `ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 kinematics_params:=${HOME}/ur3e_calibration_194.yaml launch_rviz:=false` |
| T3 | `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true` |
| T4 | `ros2 launch easy_handeye2 publish.launch.py name:=ur3e_eye_on_hand` |
| T5 | `ros2 run perception_pkg pose_estimator` |

---

## Verify Topics

```bash
# Check poses are publishing
ros2 topic echo /detected_objects

# Check labels are publishing
ros2 topic echo /object_labels

# Check topic rates
ros2 topic hz /detected_objects
ros2 topic hz /camera/camera/color/image_raw

# Check TF tree is connected
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

---

## TF Transform Chain

```
base_link → tool0          (UR driver — live joint states)
tool0 → camera_color_optical_frame   (hand-eye calibration — easy_handeye2 publisher)
camera_color_optical_frame → cube    (pose_estimator depth calculation)
─────────────────────────────────────────────────────────────
base_link → cube           (published on /detected_objects)
```

---

## Detection Parameters

| Parameter | Value | Purpose |
|---|---|---|
| MIN_AREA | 2000 px | Filters small noise detections |
| MIN_SQUARENESS | 0.65 | Rejects non-square (side face) detections |
| BUFFER_SIZE | 3 frames | Smoothing buffer per cluster |
| DEPTH_WINDOW | 20 px | Robust median depth sampling |
| POSE_TIMEOUT | 2.0 s | Holds last known pose after cube leaves view |
| CLUSTER_DIST | 5 cm | Spatial distance to separate multiple cubes of same colour |

---

## Interface with SS2

SS1 publishes to these topics consumed by SS2 (Ha Khoa Vu):

| Topic | Type | Notes |
|---|---|---|
| `/detected_objects` | `geometry_msgs/PoseArray` | Poses in `base_link` frame, ordered red→yellow→blue |
| `/object_labels` | `std_msgs/String` | Colour names matching pose indices |

