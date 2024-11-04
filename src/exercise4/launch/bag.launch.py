from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import (Command, FindExecutable,
                                  PathJoinSubstitution, LaunchConfiguration)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os

def generate_launch_description():

    # Launch Configurations
    setup_path = LaunchConfiguration('setup_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    robot_description_command = LaunchConfiguration('robot_description_command')
   
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_manipulation_controllers = LaunchConfiguration('use_manipulation_controllers')
    use_platform_controllers = LaunchConfiguration('use_platform_controllers')

    # Launch Arguments
    arg_setup_path = DeclareLaunchArgument(
        'setup_path',
        default_value= os.path.expanduser("~") + '/clearpath/'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        choices=['true', 'false'],
        default_value='true',
        description='Use simulation time'
    )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='a200_0000',
        description='Robot namespace'
    )

    arg_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware if true'
    )

    arg_use_manipulation_controllers = DeclareLaunchArgument(
        'use_manipulation_controllers',
        default_value='false',
        description='Use manipulation controllers if true'
    )

    arg_use_platform_controllers = DeclareLaunchArgument(
        'use_platform_controllers',
        default_value='true',
        description='Use platform controllers if true'
    )


    # Paths
    dir_platform_config = PathJoinSubstitution([
        setup_path, 'platform/config'])

    # Configs
    config_localization = [
        dir_platform_config,
        '/localization.yaml'
    ]

    # Localization
    node_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, config_localization],
            namespace='a200_0000',
            remappings=[
              ('odometry/filtered', 'platform/odom/filtered'),
              ('/diagnostics', 'diagnostics'),
              ('/tf', 'tf'),
              ('/tf_static', 'tf_static'),
            ]
        )
    

    # Paths
    robot_urdf = PathJoinSubstitution([
        setup_path, 'robot.urdf.xacro'])
    config_control = PathJoinSubstitution([
        dir_platform_config, 'control.yaml'])

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            robot_urdf,
            ' ',
            'is_sim:=',
            use_sim_time,
            ' ',
            'gazebo_controllers:=',
            config_control,
            ' ',
            'namespace:=',
            namespace,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
            ' ',
            'use_manipulation_controllers:=',
            use_manipulation_controllers,
            ' ',
            'use_platform_controllers:=',
            use_platform_controllers,
        ]
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    arg_rviz_config = DeclareLaunchArgument(
        name='config',
        default_value='robot.rviz',
    )

    # Launch Configurations
    rviz_config = LaunchConfiguration('config')
    config_rviz = PathJoinSubstitution(
        [setup_path, rviz_config]
    )


    group_action_state_publishers = GroupAction([
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', config_rviz],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[
               ('odometry/filtered', 'platform/odom/filtered'),
               ('/diagnostics', 'diagnostics'),
               ('/tf', 'tf'),
               ('/tf_static', 'tf_static'),
             ],
             namespace='a200_0000',
             output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
            }],
            namespace='a200_0000',
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('joint_states', 'platform/joint_states')]
                ),
    ])

    ld = LaunchDescription()    
    # Args
    ld.add_action(arg_setup_path)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_namespace)
    ld.add_action(arg_use_fake_hardware)
    ld.add_action(arg_use_manipulation_controllers)
    ld.add_action(arg_use_platform_controllers)
    ld.add_action(arg_robot_description_command)
    ld.add_action(arg_rviz_config)
    # Nodes
    ld.add_action(node_localization)
    ld.add_action(group_action_state_publishers)
    return ld
