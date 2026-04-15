# ONLY STYLE + RANDOM BUTTON + SEPARATE PARAM FIELDS — EVERYTHING ELSE SAME

import tkinter as tk
from tkinter import ttk, messagebox
import yaml
import random

import rclpy
from rclpy.node import Node

from custom_msgs.msg import GoalCommand
from custom_msgs.msg import SceneCommand, PrimitiveObject, MeshObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose


class RRTGui(Node):
    def __init__(self, yaml_path):
        super().__init__('rrt_gui')

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        params = config['rrt_node']['ros__parameters']
        self.robot_name = params['use_arm']
        robot = params['robots'][self.robot_name]
        self.dof = robot['dof']
        self.lower = robot['joint_limits']['lower']
        self.upper = robot['joint_limits']['upper']

        self.pub = self.create_publisher(GoalCommand, '/goal_command', 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.scene_pub = self.create_publisher(SceneCommand, '/scene_command', 10)

        self.scene_objects = []

        # ================= ROOT =================
        self.root = tk.Tk()
        self.root.title(f"RRT GUI — {self.robot_name}")
        self.root.configure(bg="#ffffff")

        # ================= STYLE =================
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("TNotebook", background="#ffffff", borderwidth=0)
        style.configure("TNotebook.Tab",
                        background="#e6e6e6",
                        foreground="#222222",
                        padding=[10, 4],
                        font=("Arial", 10))
        style.map("TNotebook.Tab",
                  background=[("selected", "#ffffff")])

        style.configure("TFrame", background="#ffffff")
        style.configure("TLabel", background="#ffffff",
                        foreground="#222222", font=("Arial", 10))
        style.configure("TButton", background="#e6e6e6",
                        foreground="#222222", font=("Arial", 10),
                        borderwidth=0, padding=6)
        style.map("TButton", background=[("active", "#d6d6d6")])
        style.configure("TRadiobutton", background="#ffffff",
                        foreground="#222222", font=("Arial", 10))
        style.configure("TLabelframe", background="#ffffff",
                        foreground="#333333", font=("Arial", 10, "bold"))
        style.configure("TLabelframe.Label", background="#ffffff",
                        foreground="#333333", font=("Arial", 10, "bold"))
        style.configure("TSpinbox", fieldbackground="#ffffff",
                        foreground="#222222", font=("Arial", 10))
        style.configure("TCombobox", fieldbackground="#ffffff",
                        foreground="#222222", font=("Arial", 10))

        # ================= NOTEBOOK =================
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=10)

        planner_tab = ttk.Frame(notebook)
        notebook.add(planner_tab, text="  Planner  ")
        self._build_planner_tab(planner_tab)

        scene_tab = ttk.Frame(notebook)
        notebook.add(scene_tab, text="  Scene Objects  ")
        self._build_scene_tab(scene_tab)

        self.root.after(100, self.spin_ros)

    # ================= PLANNER TAB =================
    def _build_planner_tab(self, parent):
        self.sliders = []

        slider_frame = ttk.LabelFrame(parent, text="Joint Configuration")
        slider_frame.pack(fill="x", padx=10, pady=8)

        for i in range(self.dof):
            row = ttk.Frame(slider_frame)
            row.pack(fill="x", padx=8, pady=3)

            ttk.Label(row, text=f"J{i+1}", width=3).pack(side="left")

            slider = tk.Scale(
                row,
                from_=self.lower[i], to=self.upper[i],
                resolution=0.01, orient="horizontal", length=420,
                bg="#ffffff", fg="#222222", troughcolor="#dddddd",
                highlightthickness=0, activebackground="#4a90e2",
                font=("Arial", 9)
            )
            slider.set((self.lower[i] + self.upper[i]) / 2.0)
            slider.pack(side="right")
            self.sliders.append(slider)

        planner_frame = ttk.LabelFrame(parent, text="Planner")
        planner_frame.pack(fill="x", padx=10, pady=8)

        self.planner_var = tk.IntVar(value=GoalCommand.RRT_CONNECT)
        for text, val in [("RRT", GoalCommand.RRT),
                          ("RRT*", GoalCommand.RRT_STAR),
                          ("RRT-Connect", GoalCommand.RRT_CONNECT)]:
            ttk.Radiobutton(planner_frame, text=text,
                            variable=self.planner_var, value=val).pack(side="left", padx=10, pady=5)

        time_frame = ttk.LabelFrame(parent, text="Trajectory Time (sec)")
        time_frame.pack(fill="x", padx=10, pady=8)

        self.time_var = tk.DoubleVar(value=2.0)
        ttk.Spinbox(time_frame, from_=0.5, to=10.0, increment=0.5,
                    textvariable=self.time_var, width=10).pack(side="left", padx=10, pady=5)

        self.status_label = ttk.Label(parent, text="● Idle", foreground="green")
        self.status_label.pack(pady=4)

        btn_frame = ttk.Frame(parent)
        btn_frame.pack(pady=8)

        ttk.Button(btn_frame, text="Send Goal", command=self.send_goal).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Send Trajectory", command=self.send_trajectory).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Reset Sliders", command=self.reset_sliders).pack(side="left", padx=5)

        preset_frame = ttk.LabelFrame(parent, text="Presets")
        preset_frame.pack(fill="x", padx=10, pady=8)

        ttk.Button(preset_frame, text="Home",
                   command=self.set_home_position).pack(side="left", padx=5, pady=5)
        ttk.Button(preset_frame, text="Safe",
                   command=self.set_safe_position).pack(side="left", padx=5, pady=5)
        ttk.Button(preset_frame, text="Random",
                   command=self.set_random_position).pack(side="left", padx=5, pady=5)

    def set_random_position(self):
        for i, s in enumerate(self.sliders):
            s.set(random.uniform(self.lower[i], self.upper[i]))

    # ================= SCENE TAB =================
    def _build_scene_tab(self, parent):
        left = ttk.Frame(parent)
        left.pack(side="left", fill="both", expand=True, padx=10, pady=8)

        name_frame = ttk.LabelFrame(left, text="Object Name")
        name_frame.pack(fill="x", pady=4)

        self.obj_name_var = tk.StringVar(value="obstacle_1")
        tk.Entry(name_frame, textvariable=self.obj_name_var,
                 bg="#ffffff", fg="#222222", insertbackground="#222222",
                 font=("Arial", 10), relief="solid", bd=1).pack(fill="x", padx=6, pady=4)

        pose_frame = ttk.LabelFrame(left, text="Pose  (x y z qx qy qz qw)")
        pose_frame.pack(fill="x", pady=4)

        self.pose_vars = []
        defaults = [0.5, 0.0, 0.5, 0, 0, 0, 1]
        pose_inner = ttk.Frame(pose_frame)
        pose_inner.pack(padx=6, pady=4)

        for i, val in enumerate(defaults):
            v = tk.DoubleVar(value=val)
            tk.Entry(pose_inner, textvariable=v, width=6,
                     bg="#ffffff", fg="#222222",
                     insertbackground="#222222",
                     font=("Arial", 9), relief="solid", bd=1).grid(row=0, column=i, padx=2)
            self.pose_vars.append(v)

        prim_frame = ttk.LabelFrame(left, text="Add Primitive")
        prim_frame.pack(fill="x", pady=4)

        self.prim_type_var = tk.StringVar(value="BOX")

        combo = ttk.Combobox(prim_frame, textvariable=self.prim_type_var,
                             values=["SPHERE", "CYLINDER", "BOX", "CAPSULE", "CONE"],
                             state="readonly")
        combo.pack(padx=6, pady=4)
        combo.bind("<<ComboboxSelected>>", self.on_primitive_change)

        self.param_label = ttk.Label(prim_frame, text="Dimensions (x y z)")
        self.param_label.pack()

        # ===== NEW PARAM FIELDS =====
        self.param_container = ttk.Frame(prim_frame)
        self.param_container.pack(pady=4)

        self.param_entries = []
        self.build_param_fields("BOX")

        ttk.Button(prim_frame, text="Add Primitive",
                   command=self.send_add_primitive).pack(pady=6)

        right = ttk.Frame(parent)
        right.pack(side="right", fill="both", padx=10, pady=8)

        self.obj_listbox = tk.Listbox(
            right,
            bg="#ffffff",
            fg="#222222",
            selectbackground="#cce5ff",
            font=("Arial", 10)
        )
        self.obj_listbox.pack(fill="both", expand=True)

        ttk.Button(right, text="Remove",
                   command=self.send_remove_object).pack(pady=3)

    # ===== NEW FUNCTION =====
    def build_param_fields(self, prim_type):
        for widget in self.param_container.winfo_children():
            widget.destroy()

        self.param_entries = []

        def add_field(label, default):
            frame = ttk.Frame(self.param_container)
            frame.pack(side="left", padx=3)

            ttk.Label(frame, text=label).pack()

            var = tk.DoubleVar(value=default)
            entry = tk.Entry(frame,
                             textvariable=var,
                             width=6,
                             bg="#ffffff",
                             fg="#222222",
                             insertbackground="#222222",
                             font=("Arial", 9),
                             relief="solid",
                             bd=1)
            entry.pack()

            self.param_entries.append(var)

        if prim_type == "BOX":
            add_field("x", 1.0)
            add_field("y", 0.8)
            add_field("z", 0.02)
        elif prim_type == "SPHERE":
            add_field("r", 0.5)
        else:
            add_field("r", 0.3)
            add_field("h", 1.0)

    def on_primitive_change(self, event=None):
        prim = self.prim_type_var.get()

        if prim == "BOX":
            self.param_label.config(text="Dimensions (x y z)")
        elif prim == "SPHERE":
            self.param_label.config(text="Radius")
        else:
            self.param_label.config(text="Radius Height")

        self.build_param_fields(prim)

    # ================= METHODS =================
    def _build_pose(self):
        vals = [v.get() for v in self.pose_vars]
        p = Pose()
        p.position.x, p.position.y, p.position.z = vals[:3]
        p.orientation.x, p.orientation.y = vals[3:5]
        p.orientation.z, p.orientation.w = vals[5:7]
        return p

    def send_add_primitive(self):
        msg = SceneCommand()
        msg.action = SceneCommand.ADD_PRIMITIVE
        msg.primitive.name = self.obj_name_var.get()

        prim_map = {
            "BOX": PrimitiveObject.BOX,
            "SPHERE": PrimitiveObject.SPHERE,
            "CYLINDER": PrimitiveObject.CYLINDER,
            "CAPSULE": PrimitiveObject.CAPSULE,
            "CONE": PrimitiveObject.CONE
        }

        msg.primitive.type = prim_map[self.prim_type_var.get()]
        msg.primitive.params = [v.get() for v in self.param_entries]
        msg.primitive.pose = self._build_pose()

        self.scene_pub.publish(msg)
        self.obj_listbox.insert("end", msg.primitive.name)

    def send_remove_object(self):
        sel = self.obj_listbox.curselection()
        if not sel:
            return
        name = self.obj_listbox.get(sel[0])

        msg = SceneCommand()
        msg.action = SceneCommand.REMOVE
        msg.remove_name = name

        self.scene_pub.publish(msg)
        self.obj_listbox.delete(sel[0])

    def send_goal(self):
        msg = GoalCommand()
        msg.goal = [s.get() for s in self.sliders]
        msg.planner_type = self.planner_var.get()
        self.pub.publish(msg)

    def send_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i+1}' for i in range(self.dof)]

        point = JointTrajectoryPoint()
        point.positions = [s.get() for s in self.sliders]

        t = self.time_var.get()
        point.time_from_start = Duration(sec=int(t),
                                         nanosec=int((t - int(t)) * 1e9))

        msg.points = [point]
        self.trajectory_pub.publish(msg)

    def reset_sliders(self):
        for i, s in enumerate(self.sliders):
            s.set((self.lower[i] + self.upper[i]) / 2.0)

    def set_home_position(self):
        for s in self.sliders:
            s.set(0.0)

    def set_safe_position(self):
        for i, s in enumerate(self.sliders):
            s.set([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785][i])

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