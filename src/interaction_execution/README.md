# SS3 – Interaction & Execution Subsystem

## Overview

SS3 acts as the coordination layer between the perception subsystem (SS1) and the motion planning/execution subsystem (SS2).

The subsystem is responsible for:

* Moving the UR3e cobot to a predefined search position for camera-based cube detection
* Receiving raw cube poses and colour labels from SS1
* Validating all 6 detected cubes before execution
* Sorting cubes using nearest-first logic
* Generating the final ordered pyramid-building sequence
* Publishing ordered cube poses to SS2
* Supporting targeted cube retrieval after pyramid completion
* Managing START and RESET workflow

---

# Final System Architecture

```text
SS1
/raw_detected_objects
/object_labels
        ↓
SS3
search-position + validation + ordering
        ↓
/detected_objects
        ↓
SS2
```

---

# Main Features

* ROS2 Humble compatible
* Implemented fully in C++
* MoveIt2 integration
* UR3e search-position movement
* Full 6-cube validation
* Nearest-first cube ordering
* Pyramid-ready pose sequencing
* Retrieval-ready architecture
* START/RESET workflow
* Topic-based subsystem integration

---

# Dependencies

## Hardware

* Universal Robots UR3e
* OnRobot RG2 Gripper
* RGB-D Camera

## Software

* Ubuntu 22.04
* ROS2 Humble
* MoveIt2
* RViz2
* colcon

---

# Required ROS2 Packages

```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-planning-interface
```

---

# Workspace Setup

```bash
cd ~

mkdir -p rs2-stackbot/src

cd rs2-stackbot/src

git clone https://github.com/jhansaikat-cloud/rs2-stackbot.git
```

---

# Build Instructions

```bash
cd ~/rs2-stackbot

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
```

---

# Running the Full System

## Terminal 1 – Start UR3e Driver

```bash
cd ~/rs2-stackbot
source install/setup.bash

ros2 launch ur_onrobot_control start_robot.launch.py \
ur_type:=ur3e \
onrobot_type:=rg2 \
robot_ip:=192.168.0.194
```

---

## Terminal 2 – Start MoveIt + RViz

```bash
cd ~/rs2-stackbot
source install/setup.bash

ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py \
ur_type:=ur3e \
onrobot_type:=rg2
```

---

## Terminal 3 – Run SS3 Coordinator

```bash
cd ~/rs2-stackbot
source install/setup.bash

ros2 launch interaction_execution ss3_task_coordinator.launch.py
```

---

# Starting the Task

```bash
ros2 topic pub --once /client/start std_msgs/msg/Bool "
data: true
"
```

---

# Resetting the Task

```bash
ros2 topic pub --once /client/reset std_msgs/msg/Bool "
data: true
"
```

---

# Retrieval Request

After pyramid completion:

```bash
ros2 topic pub --once /client/retrieve_cube std_msgs/msg/String "
data: 'cube_2'
"
```

SS3 forwards the retrieval request to SS2 using:

```text
/retrieve_cube
```

---

# SS1 Integration Topics

| Topic                   | Type                      | Purpose            |
| ----------------------- | ------------------------- | ------------------ |
| `/raw_detected_objects` | `geometry_msgs/PoseArray` | Raw cube poses     |
| `/object_labels`        | `std_msgs/String`         | Cube colour labels |

---

# SS3 Published Topics

| Topic               | Type                      | Purpose                           |
| ------------------- | ------------------------- | --------------------------------- |
| `/detected_objects` | `geometry_msgs/PoseArray` | Ordered cube sequence for SS2     |
| `/retrieve_cube`    | `std_msgs/String`         | Retrieval target forwarded to SS2 |

---

# Detection Validation Logic

SS3 validates the incoming cube data before publishing to SS2.

Validation requirements:

```text
Total poses = 6
Total labels = 6

Colour distribution:
Red    = 3
Yellow = 2
Blue   = 1
```

If validation fails:

* SS3 does NOT publish `/detected_objects`
* SS2 execution does NOT start
* SS3 waits for updated perception data from SS1

---

# Ordering Logic

The cubes are grouped by colour and sorted using nearest-first planar distance ordering.

Final pyramid sequence:

| Layer  | Colour | Count |
| ------ | ------ | ----- |
| Bottom | Red    | 3     |
| Middle | Yellow | 2     |
| Top    | Blue   | 1     |

Published sequence order:

```text
Red base cubes
↓
Yellow middle cubes
↓
Blue top cube
```

---

# Search Position Logic

When `/client/start` is received:

1. SS3 moves UR3e to a predefined search pose
2. SS1 performs cube detection
3. SS3 validates the full cube set
4. SS3 publishes ordered cube poses to SS2

The search position is executed using MoveIt2:

```cpp
moveToSearchPosition()
```

---

# Search Position Calibration

The search position joint values inside:

```cpp
std::vector<double> search_joint_values
```

must be replaced using real UR3e joint values.

## Calibration Procedure

1. Move the robot to the desired camera-search pose in RViz
2. Read the robot joint values:

```bash
ros2 topic echo /joint_states
```

3. Replace the placeholder joint values in:

```cpp
moveToSearchPosition()
```

---

# Known Limitations

* Placeholder search pose values require calibration on the real robot
* SS3 assumes SS1 provides accurate colour labels
* Dynamic re-planning is not implemented
* Retrieval obstacle-removal sequencing is handled by SS2

---

# Troubleshooting

| Problem                    | Likely Cause             | Solution                             |
| -------------------------- | ------------------------ | ------------------------------------ |
| `robot_description` error  | MoveIt not running       | Launch MoveIt before SS3             |
| Search position failed     | Invalid joint values     | Recalibrate search pose              |
| No cube sequence published | Incomplete detections    | Verify SS1 topics                    |
| SS2 does not move          | Wrong topic subscription | Confirm SS2 uses `/detected_objects` |
| Retrieval ignored          | Pyramid not complete     | Complete build before retrieval      |

---

# Final Topic Flow

```text
/client/start
        ↓
SS3 moves robot to search position
        ↓
SS1 publishes:
/raw_detected_objects
/object_labels
        ↓
SS3 validates:
3 red
2 yellow
1 blue
        ↓
SS3 nearest-first ordering
        ↓
SS3 publishes:
/detected_objects
        ↓
SS2 builds pyramid
        ↓
Client sends retrieval request
        ↓
SS3 forwards:
/retrieve_cube
        ↓
SS2 performs targeted retrieval
```

---

# Author

**Saikat Jhan**
SS3 – Interaction & Execution Subsystem
Robotics Studio 2 – UTS
