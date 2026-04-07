import tkinter as tk
from tkinter import ttk
import yaml

import rclpy
from rclpy.node import Node

from custom_msgs.msg import GoalCommand


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
        self.pub = self.create_publisher(GoalCommand, '/goal_command', 10)

        # --- GUI ---
        self.root = tk.Tk()
        self.root.title(f"RRT GUI - {self.robot_name}")

        self.sliders = []

        # --- Joint sliders ---
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

        # --- Planner selection ---
        planner_frame = ttk.Frame(self.root)
        planner_frame.pack(pady=10)

        ttk.Label(planner_frame, text="Planner:").pack(side='left')

        self.planner_var = tk.IntVar(value=GoalCommand.RRT)

        ttk.Radiobutton(
            planner_frame,
            text="RRT",
            variable=self.planner_var,
            value=GoalCommand.RRT
        ).pack(side='left')

        ttk.Radiobutton(
            planner_frame,
            text="RRT*",
            variable=self.planner_var,
            value=GoalCommand.RRT_STAR
        ).pack(side='left')

        # --- Buttons ---
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)

        send_btn = ttk.Button(btn_frame, text="Send Goal", command=self.send_goal)
        send_btn.pack(side='left', padx=5)

        reset_btn = ttk.Button(btn_frame, text="Reset", command=self.reset_sliders)
        reset_btn.pack(side='left', padx=5)

        # --- Start ROS spin loop ---
        self.root.after(100, self.spin_ros)

    def send_goal(self):
        msg = GoalCommand()

        msg.goal = [slider.get() for slider in self.sliders]
        msg.planner_type = self.planner_var.get()

        self.pub.publish(msg)

        planner_name = "RRT" if msg.planner_type == GoalCommand.RRT else "RRT*"
        self.get_logger().info(f"Sent goal: {msg.goal} | Planner: {planner_name}")

    def reset_sliders(self):
        for i, slider in enumerate(self.sliders):
            slider.set((self.lower[i] + self.upper[i]) / 2.0)

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(100, self.spin_ros)

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