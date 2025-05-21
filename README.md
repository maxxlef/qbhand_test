# QBHand ROS 2 Control Test Suite

This repository contains Python scripts designed to explore and test the motion control capabilities of the QBHand via ROS 2 topics exclusively. These tools allow for position and velocity-based control and facilitate the evaluation of the robot hand’s behavior in various command scenarios.

## Prerequisites

- ROS 2 Humble installed
- QBHand properly configured
- launch bringup_qbhand.launch with:
```
cd ~/colcon_ws
source install/setup.bash
ros2 launch qb_hand_description bringup_qbhand.launch standalone:=true activate>


```


## Files Overview

### 1. `trackbar.py`

This script allows real-time position control of the QBHand using a graphical trackbar (slider). The user can:
- Adjust the target position between 0.0 and 1.0 using the slider or arrow keys (left/right).
- Change the execution time for each motion command.

**Limitations and Future Improvements:**
- The hand waits until the slider remains still before executing the motion. This results in a non-fluid movement.
- This delay occurs because the robot pauses briefly each time it receives a new command, even if the change is minimal. This behavior is typical for trajectory controllers expecting discrete goals rather than continuous updates.
- A potential improvement would be to switch from time-based control to velocity-based control for smoother transitions.

---

### 2. `stop_on_command.py`

This script enables discrete position control:
- The user sets a desired position and execution time.
- Upon pressing the **"Stop"** button or a specific key (e.g., space bar), the script sends a new trajectory command using the current actual position of the hand.

**How it works:**
- The controller state is continuously monitored via the `/state` topic.
- The actual position is extracted and, upon trigger, sent back as a trajectory point to effectively "freeze" the hand in place.

---

### 3. `control_speed.py`

This script provides velocity-based control of the hand:
- Instead of specifying an execution time, the user selects a velocity value.
- The script computes the required execution time using the formula:

execution_time = abs(target_pos - current_pos) \ velocity

- A velocity of `1.0` means the hand moves from 0 to 1 (or vice versa) in exactly 1 second.

This method offers more intuitive and dynamic control, particularly for applications requiring consistent motion speed regardless of distance.