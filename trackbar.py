import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tkinter as tk


class QBHandCommander(Node):
    def __init__(self):
        super().__init__('qbhand_commander')

        self.topic_name = '/qbhand1/qbhand1_synergy_trajectory_controller/joint_trajectory'
        self.joint_name = 'qbhand1_synergy_joint'
        self.position_command = 0.0
        self.execution_time = 0.5
        self.publisher_ = self.create_publisher(JointTrajectory, self.topic_name, 10)

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
        self.get_logger().info(f"↪️ Command sent: position = {position:.2f}, time = {self.execution_time:.2f} s")


def main(args=None):
    rclpy.init(args=args)
    node = QBHandCommander()

    window = tk.Tk()
    window.title("QBHand Real-Time Control")
    window.geometry("400x200")

    # Temps d'exécution
    tk.Label(window, text="Execution time (seconds):").pack()
    time_entry = tk.Entry(window)
    time_entry.insert(0, "0.5")
    time_entry.pack()

    # Position actuelle
    tk.Label(window, text="Position (0.0 - 1.0):").pack()
    pos_slider = tk.Scale(window, from_=0.0, to=1.0, resolution=0.01,
                          orient=tk.HORIZONTAL, length=300)
    pos_slider.pack()

    position_label = tk.Label(window, text="Current position: 0.00")
    position_label.pack()

    # Callback pour envoi de commande
    def on_slider_move(value):
        try:
            pos = float(value)
            t = float(time_entry.get())
            if not 0.0 <= pos <= 1.0 or t <= 0:
                return
            node.execution_time = t
            position_label.config(text=f"Current position: {pos:.2f}")
            node.send_command(pos)
        except ValueError:
            pass

    pos_slider.config(command=on_slider_move)

    # ⌨️ Gérer les flèches gauche/droite
    def on_key(event):
        current = pos_slider.get()
        step = 0.01
        if event.keysym == 'Left':
            new_val = max(0.0, current - step)
            pos_slider.set(round(new_val, 2))
        elif event.keysym == 'Right':
            new_val = min(1.0, current + step)
            pos_slider.set(round(new_val, 2))

    window.bind('<Left>', on_key)
    window.bind('<Right>', on_key)

    # Focus clavier sur la fenêtre
    window.focus_set()

    try:
        window.mainloop()
    except KeyboardInterrupt:
        window.destroy()
        node.destroy_node()
        pass



if __name__ == '__main__':
    main()
