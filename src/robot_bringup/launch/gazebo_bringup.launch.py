
import os
import time
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
        rrt_node,
        gui,
    ]

    return LaunchDescription(declared_arguments + nodes)