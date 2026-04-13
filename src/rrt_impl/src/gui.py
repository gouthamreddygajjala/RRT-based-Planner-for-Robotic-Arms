import tkinter as tk
from tkinter import ttk
import yaml

import rclpy
from rclpy.node import Node

from custom_msgs.msg import GoalCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


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

        # --- Publishers ---
        self.pub = self.create_publisher(GoalCommand, '/goal_command', 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # --- GUI ---
        self.root = tk.Tk()
        self.root.title(f"RRT GUI - {self.robot_name}")

        self.sliders = []

        # ================= JOINT SLIDERS =================
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

        # ================= PLANNER SELECTION =================
        planner_frame = ttk.Frame(self.root)
        planner_frame.pack(pady=10)

        ttk.Label(planner_frame, text="Planner:").pack(side='left')

        # Default → RRT-Connect
        self.planner_var = tk.IntVar(value=GoalCommand.RRT_CONNECT)

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

        ttk.Radiobutton(
            planner_frame,
            text="RRT-Connect",
            variable=self.planner_var,
            value=GoalCommand.RRT_CONNECT
        ).pack(side='left')

        # ================= TRAJECTORY TIME =================
        time_frame = ttk.Frame(self.root)
        time_frame.pack(pady=5)

        ttk.Label(time_frame, text="Trajectory Time (sec):").pack(side='left', padx=5)

        self.time_var = tk.DoubleVar(value=2.0)

        time_spinbox = ttk.Spinbox(
            time_frame,
            from_=0.5,
            to=10.0,
            increment=0.5,
            textvariable=self.time_var,
            width=10
        )
        time_spinbox.pack(side='left')

        # ================= STATUS LABEL =================
        self.status_label = ttk.Label(self.root, text="Idle")
        self.status_label.pack(pady=5)

        # ================= BUTTONS =================
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=10)

        send_btn = ttk.Button(btn_frame, text="Send Goal", command=self.send_goal)
        send_btn.pack(side='left', padx=5)

        trajectory_btn = ttk.Button(
            btn_frame,
            text="Send Trajectory",
            command=self.send_trajectory
        )
        trajectory_btn.pack(side='left', padx=5)

        reset_btn = ttk.Button(btn_frame, text="Reset", command=self.reset_sliders)
        reset_btn.pack(side='left', padx=5)

        # ================= PRESETS =================
        preset_frame = ttk.LabelFrame(self.root, text="Preset Positions")
        preset_frame.pack(pady=10, padx=10, fill='x')

        ttk.Button(
            preset_frame,
            text="Home Position",
            command=self.set_home_position
        ).pack(side='left', padx=5, pady=5)

        ttk.Button(
            preset_frame,
            text="Safe Position",
            command=self.set_safe_position
        ).pack(side='left', padx=5, pady=5)

        # --- ROS spin loop ---
        self.root.after(100, self.spin_ros)

    # ================= SEND GOAL =================
    def send_goal(self):
        msg = GoalCommand()

        msg.goal = [slider.get() for slider in self.sliders]
        msg.planner_type = self.planner_var.get()

        self.pub.publish(msg)

        planner_name = self.get_planner_name(msg.planner_type)

        self.get_logger().info(f"Sent goal: {msg.goal} | Planner: {planner_name}")
        self.status_label.config(text=f"Sent {planner_name} goal")

    # ================= SEND TRAJECTORY =================
    def send_trajectory(self):
        msg = JointTrajectory()

        msg.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]

        point = JointTrajectoryPoint()
        point.positions = [slider.get() for slider in self.sliders]

        time_sec = int(self.time_var.get())
        time_nsec = int((self.time_var.get() - time_sec) * 1e9)

        point.time_from_start = Duration(sec=time_sec, nanosec=time_nsec)

        msg.points = [point]

        self.trajectory_pub.publish(msg)

        self.get_logger().info(
            f"Sent trajectory: {point.positions} | Time: {self.time_var.get()}s"
        )

    # ================= HELPERS =================
    def get_planner_name(self, planner_type):
        if planner_type == GoalCommand.RRT:
            return "RRT"
        elif planner_type == GoalCommand.RRT_STAR:
            return "RRT*"
        elif planner_type == GoalCommand.RRT_CONNECT:
            return "RRT-Connect"
        return "Unknown"

    def set_home_position(self):
        for slider in self.sliders:
            slider.set(0.0)

    def set_safe_position(self):
        safe_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        for i, slider in enumerate(self.sliders):
            if i < len(safe_positions):
                slider.set(safe_positions[i])

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