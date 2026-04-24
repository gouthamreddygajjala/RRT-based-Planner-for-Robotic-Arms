import os
import time
from platform import node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    timestr = time.strftime("%Y:%m:%d-%H:%M:%S")
    # set ros log file location
    os.environ['ROS_LOG_DIR'] = os.path.join(os.getcwd(),"Logs",timestr)

    config = load_yaml("robot_bringup", "config/robot_config.yaml")


    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            #        output="screen",
            #        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            )


    arm_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "-c", "/controller_manager"],
            )
    
    arm_velocity_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_velocity_controller", "-c", "/controller_manager"],
            )
    

   
    nodes = [
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            #arm_velocity_controller_spawner
    ]

    return LaunchDescription(nodes)
