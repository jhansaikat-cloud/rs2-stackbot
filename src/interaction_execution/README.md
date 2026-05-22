# SS3 – Interaction & Execution Coordination

## Overview

SS3 is the coordination layer between SS1 perception and SS2 motion execution.

It receives raw cube detections from SS1, validates the data, orders the cubes for pyramid construction, and publishes the final execution-ready sequence to SS2.

---

# Main Responsibilities

* Receive raw cube poses from SS1
* Receive colour labels from SS1
* Validate cube detections
* Move UR3e to search position before detection
* Order cubes using nearest-first logic
* Publish ordered poses to SS2
* Republish poses every second so SS2 does not miss them
* Provide base-only fallback if full detection fails
* Forward retrieval requests to SS2
* Support reset and recovery

---

# Topic Flow

```text
SS1
/raw_detected_objects
/object_labels
        ↓
SS3
validation + ordering + fallback
        ↓
/detected_objects
        ↓
SS2
```

---

# SS3 Input Topics

| Topic                   | Type                      | Purpose                             |
| ----------------------- | ------------------------- | ----------------------------------- |
| `/raw_detected_objects` | `geometry_msgs/PoseArray` | Raw cube poses from SS1             |
| `/object_labels`        | `std_msgs/String`         | Colour labels matching pose indices |
| `/client/start`         | `std_msgs/Bool`           | Starts SS3 coordination             |
| `/client/reset`         | `std_msgs/Bool`           | Resets SS3 state                    |
| `/client/retrieve_cube` | `std_msgs/String`         | User retrieval request              |

---

# SS3 Output Topics

| Topic               | Type                      | Purpose                            |
| ------------------- | ------------------------- | ---------------------------------- |
| `/detected_objects` | `geometry_msgs/PoseArray` | Ordered cube poses sent to SS2     |
| `/retrieve_cube`    | `std_msgs/String`         | Retrieval request forwarded to SS2 |

---

# Build

```bash
cd ~/rs2-stackbot
colcon build --packages-select interaction_execution
source install/setup.bash
```

---

# Run SS3

```bash
cd ~/rs2-stackbot
source install/setup.bash
ros2 launch interaction_execution ss3_task_coordinator.launch.py
```

Expected:

```text
SS3 ready. Waiting for /client/start.
SS3 state changed to: IDLE
```

---

# Start SS3

```bash
ros2 topic pub --once /client/start std_msgs/msg/Bool "
data: true
"
```

---

# Reset SS3

```bash
ros2 topic pub --once /client/reset std_msgs/msg/Bool "
data: true
"
```

---

# Retrieval Request

```bash
ros2 topic pub --once /client/retrieve_cube std_msgs/msg/String "
data: 'cube_2'
"
```

SS3 forwards this to:

```text
/retrieve_cube
```

---

# Full Pyramid Validation

SS3 publishes a full 6-cube sequence only when:

```text
poses = 6
labels = 6
red = 3
yellow = 2
blue = 1
pose count = label count
```

Output order:

```text
3 red base cubes
2 yellow middle cubes
1 blue top cube
```

---

# Base Fallback Mode

If full pyramid validation fails but at least 3 cubes are detected:

```text
SS3 selects nearest 3 cubes
publishes 3 poses to /detected_objects
SS2 can build base layer
```

This protects the minimum passing requirement.

---

# State Machine

```text
IDLE
↓
MOVING_TO_SEARCH
↓
WAITING_FOR_DETECTION
↓
VALIDATING
↓
SEQUENCE_PUBLISHED
↓
RETRIEVAL_READY
↓
RESETTING
```

---

# No-Robot Laptop Testing

For laptop-only testing, bypass the MoveIt search-position block inside `startCallback()` and set:

```cpp
search_position_reached_ = true;
```

Then test using fake topics.

---

# Test: Full 6-Cube Sequence

```bash
ros2 topic pub --times 3 /raw_detected_objects geometry_msgs/msg/PoseArray "
header:
  frame_id: 'base_link'
poses:
- position: {x: 0.30, y: 0.10, z: 0.05}
- position: {x: 0.20, y: 0.10, z: 0.05}
- position: {x: 0.40, y: 0.10, z: 0.05}
- position: {x: 0.25, y: 0.20, z: 0.05}
- position: {x: 0.35, y: 0.20, z: 0.05}
- position: {x: 0.30, y: 0.30, z: 0.05}
"
```

```bash
ros2 topic pub --times 3 /object_labels std_msgs/msg/String "
data: 'red,red,red,yellow,yellow,blue'
"
```

Check:

```bash
ros2 topic echo /detected_objects
```

Expected: 6 ordered poses, republished every second.

---

# Test: Base Fallback

```bash
ros2 topic pub --times 3 /raw_detected_objects geometry_msgs/msg/PoseArray "
header:
  frame_id: 'base_link'
poses:
- position: {x: 0.30, y: 0.10, z: 0.05}
- position: {x: 0.20, y: 0.10, z: 0.05}
- position: {x: 0.40, y: 0.10, z: 0.05}
"
```

```bash
ros2 topic pub --times 3 /object_labels std_msgs/msg/String "
data: 'red,yellow,blue'
"
```

Expected: 3 ordered poses published to `/detected_objects`.

---

# Real Robot Requirements

Before real robot testing, restore the threaded `moveToSearchPosition()` block.

Required launch order:

```text
1. Robot driver
2. MoveIt + RViz
3. SS1 perception
4. SS3 coordinator
5. /client/start
6. SS2
```

---

# Important Notes

* SS3 publishes to `/detected_objects` because SS2 expects that topic.
* SS3 republishes the last valid sequence every second to prevent SS2 missing data.
* SS3 uses fallback mode only if full pyramid validation fails.
* Retrieval blocker-removal logic is handled by SS2.
* SS3 only forwards retrieval requests.

---

# Author

Saikat Jhan
SS3 – Interaction & Execution Coordination
Robotics Studio 2 – UTS
