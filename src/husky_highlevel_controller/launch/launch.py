from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
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
            ('rviz', 'false'),
            ('world', 'single_pillar')]
    )

    # add teleopt_twist_keyboard with launch prefix gnome-terminal --
    #teleopt_node = Node(
    #    package="teleop_twist_keyboard",
    #    executable="teleop_twist_keyboard",
    #    name="teleop",
    #    output="screen",
    #    prefix="gnome-terminal --",
    #    namespace="a200_0000",
    #)
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
        parameters=[param_file],
        namespace="a200_0000",
        # remap tf and tf static to a200_0000
        remappings=[('/tf', '/a200_0000/tf'), ('/tf_static', '/a200_0000/tf_static')]
        
    )
    distance_stop_node = Node(
        package="husky_highlevel_controller",
        executable="distance_stop",
        name="distance_stop",
        output="screen",        
        prefix="gnome-terminal --",
        parameters=[param_file],
        namespace="a200_0000",
        # remap tf and tf static to a200_0000
        remappings=[('/tf', '/a200_0000/tf'), ('/tf_static', '/a200_0000/tf_static')]
        
    )
    crash_stop_node = Node(
        package="husky_highlevel_controller",
        executable="crash_stop",
        name="crash_stop",
        output="screen",        
        prefix="gnome-terminal --",
        parameters=[param_file],
        namespace="a200_0000",
        # remap tf and tf static to a200_0000
        remappings=[('/tf', '/a200_0000/tf'), ('/tf_static', '/a200_0000/tf_static')]
        
    )
    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(sim)
    # ld.add_action(teleopt_node)
    ld.add_action(high_level)
    ld.add_action(distance_stop_node)
    #ld.add_action(crash_stop_node)
    return ld
