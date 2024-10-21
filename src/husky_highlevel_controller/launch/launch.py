from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import os

def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory("clearpath_gz")

    # Paths
    sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, "launch", "simulation.launch.py"]
    )

    # Include simulation launch file and start rviz
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource([sim_launch]),
            launch_arguments=[
            ('rviz', 'true')]
    )

    # add teleopt_twist_keyboard with launch prefix gnome-terminal --
    teleopt_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix="gnome-terminal --",
        namespace="a200_0000",
    )
    # load params from yaml file
    param_file = os.path.join(
        os.getcwd(),
        'config',
        'config.yaml'
    )

    # create node with params from yaml file
    # add husky_highlevel_controller with launch prefix gnome-terminal --
    high_level = Node(
        package="husky_highlevel_controller",
        executable="subscriber",
        name="husky_highlevel",
        output="screen",
        prefix="gnome-terminal --",
        namespace="a200_0000",
        parameters=[param_file]
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(sim)
    ld.add_action(teleopt_node)
    ld.add_action(high_level)
    return ld
