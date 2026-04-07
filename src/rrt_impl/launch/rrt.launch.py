from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml




def generate_launch_description():

    pkg_share = get_package_share_directory('rrt_impl')

    param_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    # with open(param_file, 'r') as f:
    #     print("YAML CONTENTS:", yaml.safe_load(f))

    # print("PARAM FILE:", param_file)  # 👈 DEBUG

    return LaunchDescription([
        Node(
            package='rrt_impl',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
            parameters=[param_file]
        )
    ])