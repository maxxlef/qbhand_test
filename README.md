# QBHand Control Suite (ROS 2 Humble)

A collection of Python scripts to control and test the QBHand using ROS 2, including real-time position and velocity-based control modes. Ideal for experimental evaluation, teaching, and development of ROS-based hand interfaces.

---

## 🚀 Quick Start

### Prerequisites

- ROS 2 Humble installed
- QBHand properly configured and connected
- Launch the hand interface with the differents scipts

```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
ros2 launch qb_hand_description bringup_qbhand.launch standalone:=true activate:=true
```

---

## 📦 Included Scripts

### 🔘 `trackbar.py` – Real-Time Position Slider Control  
[▶️ Demo Video](https://youtu.be/Zzw3PEGBysM)

- GUI slider for position command between 0.0 and 1.0
- Adjustable execution time per command
- Uses arrow keys or slider to change position

**Limitations:**
- Commands are only sent when the slider stops moving
- Movement is not fluid due to discrete trajectory commands
- Could be improved by switching to velocity-based control

---

### ⏹️ `stop_on_command.py` – [Deprecated]

**Not recommended** — use `control_speed.py` instead.

- Sends a trajectory to "freeze" the hand at its current position when triggered
- Monitors `/state`, and reuses the current position as a target

---

### 🕹️ `control_speed.py` – Velocity-Based Smooth Control  
[▶️ Demo Video](https://youtu.be/Zzw3PEGBysM)

- Set target position and motion velocity (units: position/s)
- Execution time is auto-computed:
  
  ```
  execution_time = abs(target_pos - current_pos) / velocity
  ```

- Real-time feedback from `/state`
- Stop button immediately halts the hand by sending a new goal with the current position

**Advantages:**
- Smooth, fluid control
- Stop feature improves safety and manual testing usability

---

## 🧠 How It Works

All scripts interact with:
- `/state` (topic): to monitor real-time hand position
- `/command` (topic): to send `trajectory_msgs/JointTrajectory` goals to the controller

---


## 📋 License

This project is licensed under the MIT License.

---

Happy controlling! 🖐️

---

