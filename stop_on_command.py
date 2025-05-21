import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
import tkinter as tk
import threading
import sys


class StopOnCommand(Node):
    def __init__(self):
        super().__init__('stop_on_command')
        self.topic_name = '/qbhand1/qbhand1_synergy_trajectory_controller/joint_trajectory'
        self.state_topic = '/qbhand1/qbhand1_synergy_trajectory_controller/state'
        self.joint_name = 'qbhand1_synergy_joint'

        self.execution_time = 0.5
        self.position_command = 0.0
        self.current_position = 0.0

        self.publisher_ = self.create_publisher(JointTrajectory, self.topic_name, 10)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, self.state_topic, self.listener_callback, 10)

    def listener_callback(self, msg):
        if msg.actual.positions:
            self.current_position = msg.actual.positions[0]
            self.get_logger().info(f"Current position: {self.current_position:.4f}")

    def send_command(self, position: float):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self.joint_name]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(sec=int(self.execution_time),
                                         nanosec=int((self.execution_time % 1) * 1e9))
        traj_msg.points.append(point)
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"=> Command sent: position = {position:.2f}, time = {self.execution_time:.2f} s")


def main():
    rclpy.init()
    node = StopOnCommand()

    # Thread ROS spin
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window = tk.Tk()
    window.title("QBHand Control")
    window.geometry("400x400")

    # Widgets
    tk.Label(window, text="Execution time (seconds):").pack(pady=(10, 0))
    time_slider = tk.Scale(window, from_=0.1, to=10.0, resolution=0.1,
                           orient=tk.HORIZONTAL, length=300)
    time_slider.set(node.execution_time)
    time_slider.pack()

    tk.Label(window, text="Position (0.0 - 1.0):").pack(pady=(10, 0))
    pos_slider = tk.Scale(window, from_=0.0, to=1.0, resolution=0.01,
                          orient=tk.HORIZONTAL, length=300)
    pos_slider.set(0.0)
    pos_slider.pack()

    current_pos_label = tk.Label(window, text="Robot actual position: 0.00")
    current_pos_label.pack(pady=(10, 0))

    def update_current_pos():
        current_pos_label.config(text=f"Robot actual position: {node.current_position:.3f}")
        window.after(100, update_current_pos)

    update_current_pos()

    def on_send_command():
        try:
            pos = float(pos_slider.get())
            t = float(time_slider.get())
            if not 0.0 <= pos <= 1.0 or t <= 0:
                return
            node.execution_time = t
            node.send_command(pos)
        except ValueError:
            pass

    send_button = tk.Button(window, text="Envoyer commande", command=on_send_command)
    send_button.pack(pady=10)

    def on_stop_command():
        pos = node.current_position
        node.send_command(pos)

    send_button = tk.Button(window, text="Arreter la main", command=on_stop_command)
    send_button.pack(pady=10)


    def on_closing():
        print("🛑 Fermeture propre...")
        node.destroy_node()
        rclpy.shutdown()
        window.destroy()
        sys.exit(0)

    window.protocol("WM_DELETE_WINDOW", on_closing)

    try:
        window.mainloop()
    except KeyboardInterrupt:
        on_closing()


if __name__ == '__main__':
    main()
