import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState

import tkinter as tk
import threading
import sys


class QBHandControllerNode(Node):
    def __init__(self):
        super().__init__('qbhand_controller')

        self.joint_name = 'qbhand1_synergy_joint'
        self.topic_name = '/qbhand1/qbhand1_synergy_trajectory_controller/joint_trajectory'
        self.state_topic = '/qbhand1/qbhand1_synergy_trajectory_controller/state'

        self.current_position = 0.0
        self.execution_time = 0.5

        self.publisher_ = self.create_publisher(JointTrajectory, self.topic_name, 10)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, self.state_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        if msg.actual.positions:
            self.current_position = msg.actual.positions[0]

    def send_command(self, position: float, time: float):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(sec=int(time), nanosec=int((time % 1) * 1e9))

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"=> Command sent: position = {position:.2f}, time = {time:.2f} s")


class QBHandControlGUI:
    def __init__(self, node: QBHandControllerNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("QBHand Control")
        self.root.geometry("400x400")

        # Velocity slider
        tk.Label(self.root, text="Velocity (1 pos/s)").pack(pady=(4, 0))
        self.velocity_slider = tk.Scale(self.root, from_=0.05, to=1.0, resolution=0.05,
                                        orient=tk.HORIZONTAL, length=300)
        self.velocity_slider.set(0.5)
        self.velocity_slider.pack()

        # Position slider
        tk.Label(self.root, text="Position (0.0 - 1.0):").pack(pady=(10, 0))
        self.pos_slider = tk.Scale(self.root, from_=0.0, to=1.0, resolution=0.01,
                                   orient=tk.HORIZONTAL, length=300)
        self.pos_slider.set(0.0)
        self.pos_slider.pack()

        # Current position label
        self.current_pos_label = tk.Label(self.root, text="Robot actual position: 0.00")
        self.current_pos_label.pack(pady=(10, 0))
        self.update_current_pos()

        # Buttons
        tk.Button(self.root, text="Send command", command=self.on_send_command).pack(pady=10)
        tk.Button(self.root, text="Stop", command=self.on_stop_command).pack(pady=10)

        # Window close protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def update_current_pos(self):
        pos = self.node.current_position
        self.current_pos_label.config(text=f"Robot actual position: {pos:.3f}")
        self.root.after(100, self.update_current_pos)

    def on_send_command(self):
        pos = float(self.pos_slider.get())
        velocity = float(self.velocity_slider.get())
        if velocity <= 0:
            return
        time = abs(pos - self.node.current_position) / velocity 
        if not (0.0 <= pos <= 1.0) or time <= 0:
            return
        self.node.send_command(pos, time)

    def on_stop_command(self):
        current_pos = self.node.current_position
        self.node.send_command(current_pos, 0.2)

    def on_closing(self):
        print("🛑 Clean shutdown...")
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
        sys.exit(0)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = QBHandControllerNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui = QBHandControlGUI(node)
    gui.run()


if __name__ == '__main__':
    main()
