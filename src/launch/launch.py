from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Directories
    pkg_clearpath_gz = get_package_share_directory("clearpath_gz")

    # Paths
    sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, "launch", "simulation.launch.py"]
    )

    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource([sim_launch]))

    # add teleopt_twist_keyboard with launch prefix gnome-terminal --
    teleopt_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop",
        output="screen",
        prefix="gnome-terminal --",
        namespace="a200_0000",
    )
    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(sim)
    ld.add_action(teleopt_node)
    return ld
