#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default_value='true')
    x_pose1 = LaunchConfiguration('x_pose1', default_value='0.0')
    y_pose1 = LaunchConfiguration('y_pose1', default_value='0.0')
    x_pose2 = LaunchConfiguration('x_pose2', default_value='2.0')  # Cambia la posición en x para el segundo TurtleBot
    y_pose2 = LaunchConfiguration('y_pose2', default_value='2.0')  # Cambia la posición en y para el segundo TurtleBot

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
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

    declare_x_position_cmd_1 = DeclareLaunchArgument(
        'x_pose1',
        default_value='0.0',
        description='Specify x position of the first robot'
    )

    declare_y_position_cmd_1 = DeclareLaunchArgument(
        'y_pose1',
        default_value='0.0',
        description='Specify y position of the first robot'
    )

    declare_x_position_cmd_2 = DeclareLaunchArgument(
        'x_pose2',
        default_value='2.0',
        description='Specify x position of the second robot'
    )

    declare_y_position_cmd_2 = DeclareLaunchArgument(
        'y_pose2',
        default_value='2.0',
        description='Specify y position of the second robot'
    )

    spawn_turtlebot_cmd_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',  # Nombre de entidad para el primer TurtleBot
            '-file', urdf_path,
            '-x', x_pose1,
            '-y', y_pose1,
            '-z', '0.01'
        ],
        output='screen',
    )

    spawn_turtlebot_cmd_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_2',  # Nombre de entidad para el segundo TurtleBot
            '-file', urdf_path,
            '-x', x_pose2,
            '-y', y_pose2,
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Agrega los comandos a la descripción de lanzamiento
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(declare_x_position_cmd_1)
    ld.add_action(declare_y_position_cmd_1)
    ld.add_action(declare_x_position_cmd_2)
    ld.add_action(declare_y_position_cmd_2)
    ld.add_action(spawn_turtlebot_cmd_1)
    ld.add_action(spawn_turtlebot_cmd_2)

    return ld

