import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk
import sys
import threading


class QBHandCommander(Node):
    def __init__(self):
        super().__init__('qbhand_commander')

        self.topic_name = '/qbhand1/qbhand1_synergy_trajectory_controller/joint_trajectory'
        self.joint_name = 'qbhand1_synergy_joint'
        self.execution_time = 0.5

        self.publisher_ = self.create_publisher(JointTrajectory, self.topic_name, 10)

    def send_command(self, position: float):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(
            sec=int(self.execution_time),
            nanosec=int((self.execution_time % 1) * 1e9)
        )

        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"=> Command sent: position = {position:.2f}, time = {self.execution_time:.2f} s")



class QBHandGUI:
    def __init__(self, node: QBHandCommander):
        self.node = node

        self.window = tk.Tk()
        self.window.title("QBHand Real-Time Control")
        self.window.geometry("400x200")

        self.setup_widgets()
        self.window.bind('<Left>', self.on_key)
        self.window.bind('<Right>', self.on_key)
        self.window.focus_set()
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_widgets(self):
        # Label + entry for execution time
        tk.Label(self.window, text="Execution time (seconds):").pack()
        self.time_entry = tk.Entry(self.window)
        self.time_entry.insert(0, str(self.node.execution_time))
        self.time_entry.pack()

        # Position slider
        tk.Label(self.window, text="Position (0.0 - 1.0):").pack()
        self.pos_slider = tk.Scale(
            self.window, from_=0.0, to=1.0, resolution=0.01,
            orient=tk.HORIZONTAL, length=300, command=self.on_slider_move
        )
        self.pos_slider.pack()

        self.position_label = tk.Label(self.window, text="Current position: 0.00")
        self.position_label.pack()

    def on_slider_move(self, value):
        try:
            pos = float(value)
            t = float(self.time_entry.get())
            if not 0.0 <= pos <= 1.0 or t <= 0.0:
                return
            self.node.execution_time = t
            self.node.send_command(pos)
            self.position_label.config(text=f"Current position: {pos:.2f}")
        except ValueError:
            pass

    def on_key(self, event):
        current = self.pos_slider.get()
        step = 0.01
        if event.keysym == 'Left':
            self.pos_slider.set(round(max(0.0, current - step), 2))
        elif event.keysym == 'Right':
            self.pos_slider.set(round(min(1.0, current + step), 2))

    def run(self):
        self.window.mainloop()

    def on_close(self):
        print("🛑 Clean shutdown...")
        self.window.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = QBHandCommander()

    # ROS spin thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = QBHandGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        gui.on_close()


if __name__ == '__main__':
    main()
