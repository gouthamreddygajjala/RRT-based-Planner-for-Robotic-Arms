
import os
import time
import yaml

from platform import node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler,     IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.descriptions
from moveit_configs_utils import MoveItConfigsBuilder




def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    declared_arguments = []
    
    # LOAD_GAZEBO_MODELS
    # os.environ['GAZEBO_MODEL_PATH'] =  '../Data/gazebo_models/'

    #Get URDF via xacro
    robot_description_content = Command(
                [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare("robot_description"), "urdf","panda.urdf.xacro"]),
                # " ",
                # "use_gazebo:=",
                # use_gazebo,
                ]
            )

    world = os.path.join(
            get_package_share_directory('robot_description'),
            'worlds',
            'empty_world.world'
        )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("robot_description"),
            "urdf",
            "panda.urdf.xacro"
        ))
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory("panda_moveit_config"),
            "config",
            "panda.srdf"
        ))
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory("panda_moveit_config"),
            "config",
            "kinematics.yaml"
        ))
        .to_moveit_configs()
    )

    robot_state_pub_node = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(robot_description_content,value_type=str), 'use_sim_time': True}],
            )

    core_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([PathJoinSubstitution(
                        [FindPackageShare('robot_bringup'), 'launch', 'core_nodes.launch.py'])]),

                )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', 'robot_description',
                             '-entity', 'panda_7dof'],
                 output='screen')
    
    
    rrt_node = Node(
        package='rrt_impl',
        executable='rrt_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('rrt_impl'), 'config', 'robot_config.yaml')]
    )   


    servo_params = {
        "moveit_servo": load_yaml("robot_bringup", "config/panda_servo.yaml")["moveit_servo"]["ros__parameters"]
    }

    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    gui = ExecuteProcess(
            cmd=['python3', os.path.join(get_package_share_directory('rrt_impl'), '/home/gautam/Music/my_ws/rrt_planning/src/rrt_impl/src/gui.py')],
            output='screen'
        )

    nodes = [
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
        robot_state_pub_node,   
        core_launch,
        # rrt_node,
        servo_node,
        # gui,
    ]

    return LaunchDescription(declared_arguments + nodes)

# def generate_launch_description():

#     declared_arguments = []
    
#     # Get URDF via xacro
#     robot_description_content = Command([
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " ",
#         PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "panda.urdf.xacro"]),
#     ])
    
#     robot_description = {"robot_description": robot_description_content}

#     world = os.path.join(
#         get_package_share_directory('robot_description'),
#         'worlds',
#         'empty_world.world'
#     )

#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
#     # MoveIt Configuration - FIXED
#     moveit_config = (
#         MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
#         .robot_description(file_path=os.path.join(
#             get_package_share_directory("robot_description"),
#             "urdf",
#             "panda.urdf.xacro"
#         ))
#         .robot_description_semantic(file_path=os.path.join(
#             get_package_share_directory("panda_moveit_config"),
#             "config",
#             "panda.srdf"
#         ))
#         .robot_description_kinematics(file_path=os.path.join(
#             get_package_share_directory("panda_moveit_config"),
#             "config",
#             "kinematics.yaml"
#         ))
#         .to_moveit_configs()
#     )

#     robot_state_pub_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="both",
#         parameters=[robot_description, {'use_sim_time': True}],
#     )

#     core_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([PathJoinSubstitution(
#             [FindPackageShare('robot_bringup'), 'launch', 'core_nodes.launch.py'])]),
#     )

#     gzserver_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
#         ),
#         launch_arguments={'world': world}.items()
#     )

#     gzclient_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
#         )
#     )

#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-topic', 'robot_description', '-entity', 'panda_7dof'],
#         output='screen'
#     )

#     # Load servo YAML properly
#     servo_yaml = load_yaml("robot_bringup", "config/panda_servo.yaml")
#     servo_params = {"moveit_servo": servo_yaml}

#     # Servo node with all required parameters
#     servo_node = Node(
#         package="moveit_servo",
#         executable="servo_node_main",
#         parameters=[
#             servo_params,
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.robot_description_kinematics,
#             {"use_sim_time": True},
#         ],
#         output="screen",
#         emulate_tty=True,  # This helps see error messages
#     )

#     nodes = [
#         gzserver_cmd,
#         gzclient_cmd,
#         robot_state_pub_node,   
#         spawn_entity,
#         core_launch,
#         servo_node,
#     ]

#     return LaunchDescription(declared_arguments + nodes)