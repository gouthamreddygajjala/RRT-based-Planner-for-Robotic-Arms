import tkinter as tk
from tkinter import ttk
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RRTGui(Node):
    def __init__(self, yaml_path):
        super().__init__('rrt_gui')

        # --- Load YAML ---
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        params = config['rrt_node']['ros__parameters']

        self.robot_name = params['use_arm']
        robot = params['robots'][self.robot_name]

        self.dof = robot['dof']
        self.lower = robot['joint_limits']['lower']
        self.upper = robot['joint_limits']['upper']

        self.get_logger().info(f"Loaded robot: {self.robot_name}, DOF: {self.dof}")

        # --- Publisher ---
        self.pub = self.create_publisher(Float64MultiArray, '/goal_command', 10)

        # --- GUI ---
        self.root = tk.Tk()
        self.root.title(f"RRT GUI - {self.robot_name}")

        self.sliders = []

        for i in range(self.dof):
            frame = ttk.Frame(self.root)
            frame.pack(fill='x', padx=10, pady=5)

            label = ttk.Label(frame, text=f"Joint {i+1}")
            label.pack(side='left')

            slider = tk.Scale(
                frame,
                from_=self.lower[i],
                to=self.upper[i],
                resolution=0.01,
                orient='horizontal',
                length=400
            )
            slider.set((self.lower[i] + self.upper[i]) / 2.0)
            slider.pack(side='right')

            self.sliders.append(slider)

        # --- Send button ---
        send_btn = ttk.Button(self.root, text="Send Goal", command=self.send_goal)
        send_btn.pack(pady=10)

    def send_goal(self):
        msg = Float64MultiArray()
        msg.data = [slider.get() for slider in self.sliders]

        self.pub.publish(msg)

        self.get_logger().info(f"Sent goal: {msg.data}")

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()

    yaml_path = "/home/gautam/Music/my_ws/rrt_planning/src/rrt_impl/config/robot_config.yaml"

    gui = RRTGui(yaml_path)
    gui.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()