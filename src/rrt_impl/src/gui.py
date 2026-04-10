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

        # --- Time duration input ---
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

        # --- Buttons ---
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

        # --- Preset positions ---
        preset_frame = ttk.LabelFrame(self.root, text="Preset Positions")
        preset_frame.pack(pady=10, padx=10, fill='x')

        preset_btn1 = ttk.Button(
            preset_frame, 
            text="Home Position", 
            command=self.set_home_position
        )
        preset_btn1.pack(side='left', padx=5, pady=5)

        preset_btn2 = ttk.Button(
            preset_frame, 
            text="Safe Position", 
            command=self.set_safe_position
        )
        preset_btn2.pack(side='left', padx=5, pady=5)

        # --- Start ROS spin loop ---
        self.root.after(100, self.spin_ros)

    def send_goal(self):
        msg = GoalCommand()

        msg.goal = [slider.get() for slider in self.sliders]
        msg.planner_type = self.planner_var.get()

        self.pub.publish(msg)

        planner_name = "RRT" if msg.planner_type == GoalCommand.RRT else "RRT*"
        self.get_logger().info(f"Sent goal: {msg.goal} | Planner: {planner_name}")

    def send_trajectory(self):
        """Send current slider positions as a joint trajectory command"""
        msg = JointTrajectory()
        
        # Set joint names (adjust these to match your robot)
        msg.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [slider.get() for slider in self.sliders]
        
        # Set time from start
        time_sec = int(self.time_var.get())
        time_nsec = int((self.time_var.get() - time_sec) * 1e9)
        point.time_from_start = Duration(sec=time_sec, nanosec=time_nsec)
        
        msg.points = [point]
        
        # Publish
        self.trajectory_pub.publish(msg)
        
        self.get_logger().info(
            f"Sent trajectory: {point.positions} | Time: {self.time_var.get()}s"
        )

    def set_home_position(self):
        """Set sliders to home position (all zeros)"""
        for slider in self.sliders:
            slider.set(0.0)

    def set_safe_position(self):
        """Set sliders to a safe collision-free position"""
        safe_positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        for i, slider in enumerate(self.sliders):
            if i < len(safe_positions):
                slider.set(safe_positions[i])

    def reset_sliders(self):
        """Reset sliders to mid-range positions"""
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