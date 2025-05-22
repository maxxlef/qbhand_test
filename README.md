# QBHand ROS 2 Control Test Suite

This repository contains Python scripts designed to explore and test the motion control capabilities of the QBHand via ROS 2 topics exclusively. These tools allow for position and velocity-based control and facilitate the evaluation of the robot hand’s behavior in various command scenarios.

## Prerequisites

- ROS 2 Humble installed
- QBHand properly configured
- launch bringup_qbhand.launch with:
```
cd ~/colcon_ws
colcon build
source install/setup.bash
ros2 launch qb_hand_description bringup_qbhand.launch standalone:=true activate>
```


## Files Overview

### 1. `trackbar.py`

Youtube video: https://youtu.be/Zzw3PEGBysM

This script allows real-time position control of the QBHand using a graphical trackbar (slider). The user can:
- Adjust the target position between 0.0 and 1.0 using the slider or arrow keys (left/right).
- Change the execution time for each motion command.

**Limitations and Future Improvements:**
- The hand waits until the slider remains still before executing the motion. This results in a non-fluid movement.
- This delay occurs because the robot pauses briefly each time it receives a new command, even if the change is minimal. This behavior is typical for trajectory controllers expecting discrete goals rather than continuous updates.
- A potential improvement would be to switch from time-based control to velocity-based control for smoother transitions.

---

### 2. `stop_on_command.py`

NOT USEFUL, SEE THE FOLLOWING PYTHON FILE WHICH DOES THE SAME THING

This script enables discrete position control:
- The user sets a desired position and execution time.
- Upon pressing the **"Stop"** button or a specific key (e.g., space bar), the script sends a new trajectory command using the current actual position of the hand.

**How it works:**
- The controller state is continuously monitored via the `/state` topic.
- The actual position is extracted and, upon trigger, sent back as a trajectory point to effectively "freeze" the hand in place.

---

### 3. `control_speed.py`

Youtube video: https://youtu.be/Zzw3PEGBysM

This script provides **velocity-based control** of the QBHand, with an additional feature to **immediately stop the hand** at its current position.

#### Features:
- The user selects a **target position** and a **velocity** (in position units per second).
- The script computes the execution time using the formula:

"execution_time = abs(target_pos - current_pos) / velocity"

- A velocity of `1.0` means the hand moves from 0 to 1 (or vice versa) in exactly 1 second.
- Pressing the **“Stop”** button sends a new trajectory command using the current position as target, effectively freezing the hand in place.

#### How it works:
- The current hand position is monitored in real-time via the `/state` topic.
- As soon as the user adjusts the slider or clicks send, a new trajectory message is published.
- If the **Stop** button is clicked, the current position is resent as a goal, causing the controller to halt motion.

#### Advantages:
- This control mode offers **dynamic speed regulation** and **real-time responsiveness**.
- The integrated stop feature improves safety and interaction usability, especially during manual tuning or testing.
