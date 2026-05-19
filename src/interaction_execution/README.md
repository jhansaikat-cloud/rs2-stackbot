# SS3 – Interaction & Execution Subsystem

## Overview

SS3 acts as the coordination layer between the perception subsystem (SS1) and the motion planning/execution subsystem (SS2).

The subsystem is responsible for:

* Moving the UR3e cobot to a predefined search position for camera-based cube detection
* Receiving raw cube poses and colour labels from SS1
* Validating detected cube information
* Sorting cubes using nearest-first logic
* Managing staged pyramid construction:

  * Base layer (3 red cubes)
  * Middle layer (2 yellow cubes)
  * Top layer (1 blue cube)
* Publishing ordered cube poses to SS2
* Coordinating stage transitions
* Supporting targeted cube retrieval after pyramid completion

---

# System Architecture

```text
SS1
/raw_detected_objects
/object_labels
        ↓
SS3
/detected_objects
/ss3/current_stage
        ↓
SS2
```

---

# Main Features

* ROS2 Humble compatible
* Implemented fully in C++
* MoveIt2 integration
* UR3e search-position movement
* Stage-based execution coordination
* Nearest-first cube sorting
* Layer-wise pyramid construction
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
* colcon
* rviz2

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

# SS1 Integration Topics

| Topic                   | Type                      | Purpose                             |
| ----------------------- | ------------------------- | ----------------------------------- |
| `/raw_detected_objects` | `geometry_msgs/PoseArray` | Raw detected cube poses             |
| `/object_labels`        | `std_msgs/String`         | Colour labels matching pose indices |

---

# SS3 Published Topics

| Topic                | Type                      | Purpose                       |
| -------------------- | ------------------------- | ----------------------------- |
| `/detected_objects`  | `geometry_msgs/PoseArray` | Ordered cube sequence for SS2 |
| `/ss3/current_stage` | `std_msgs/String`         | Current execution stage       |
| `/retrieve_cube`     | `std_msgs/String`         | Retrieval target handoff      |

---

# SS2 Required Topics

SS2 must subscribe to:

```text
/detected_objects
/ss3/current_stage
```

SS2 must publish:

```text
/ss2/execution_status
```

Supported execution status values:

```text
BASE_COMPLETE
MIDDLE_COMPLETE
TOP_COMPLETE
TASK_COMPLETE
```

---

# Stage Workflow

## Stage 1 – Search for Base

SS3:

* Moves UR3e to search position
* Waits for 3 red cubes
* Sorts nearest-first
* Publishes base layer poses

---

## Stage 2 – Search for Middle Layer

After receiving:

```text
BASE_COMPLETE
```

SS3:

* Returns robot to search position
* Waits for 2 yellow cubes
* Publishes middle layer poses

---

## Stage 3 – Search for Top Layer

After receiving:

```text
MIDDLE_COMPLETE
```

SS3:

* Returns robot to search position
* Waits for 1 blue cube
* Publishes top layer pose

---

## Stage 4 – Retrieval Mode

After receiving:

```text
TOP_COMPLETE
```

or

```text
TASK_COMPLETE
```

SS3 enters:

```text
RETRIEVAL_READY
```

A target cube can then be requested using:

```bash
ros2 topic pub --once /client/retrieve_cube std_msgs/msg/String "
data: 'cube_2'
"
```

---

# Search Position Calibration

The search position joint values inside:

```cpp
moveToSearchPosition()
```

must be replaced using real UR3e joint values.

## Procedure

1. Move robot to desired camera-search pose using RViz
2. Read joint values:

```bash
ros2 topic echo /joint_states
```

3. Replace placeholder values in:

```cpp
std::vector<double> search_joint_values
```

---

# Known Limitations

* Placeholder search joint values must be calibrated on the real robot
* SS3 assumes SS1 provides accurate cube labels
* Retrieval obstacle-removal sequencing is handled by SS2
* Dynamic re-planning is not yet implemented

---

# Troubleshooting

| Problem                    | Cause                        | Solution                             |
| -------------------------- | ---------------------------- | ------------------------------------ |
| `robot_description` error  | MoveIt not running           | Launch MoveIt before SS3             |
| Search position failed     | Invalid joint values         | Recalibrate search pose              |
| No cube sequence published | Insufficient cube detections | Verify SS1 topics                    |
| SS2 does not move          | Wrong topic subscription     | Confirm SS2 uses `/detected_objects` |
| Retrieval ignored          | Pyramid not complete         | Finish pyramid build first           |

---

# Author

**Saikat Jhan**
SS3 – Interaction & Execution Subsystem
Robotics Studio 2 – UTS
