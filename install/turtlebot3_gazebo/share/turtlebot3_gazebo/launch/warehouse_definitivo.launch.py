#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    
    
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    #TB1
    x_pose1 = LaunchConfiguration('x_pose1', default='0.0')
    y_pose1 = LaunchConfiguration('y_pose1', default='0.0')
    # #TB2
    x_pose2 = LaunchConfiguration('x_pose2', default='4.0')
    y_pose2 = LaunchConfiguration('y_pose2', default='0.0')
    #Namespaces 
    namespace1 = LaunchConfiguration('namespace1', default='tb_b')
    namespace2 = LaunchConfiguration('namespace2', default='tb_w')


    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'warehouse8.world'
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

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Declare the launch arguments TB1
    declare_x_position_cmd_1 = DeclareLaunchArgument(
        'x_pose1', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd_1 = DeclareLaunchArgument(
        'y_pose1', default_value='0.0',
        description='Specify namespace of the robot')
    
    declare_x_position_cmd_2 = DeclareLaunchArgument(
        'x_pose2', default_value='4.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd_2 = DeclareLaunchArgument(
        'y_pose2', default_value='0.0',
        description='Specify namespace of the robot')



    #Spawner TB1
    start_gazebo_ros_spawner_cmd_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # namespace=LaunchConfiguration('namespace1'),  # Usa el argumento de lanzamiento para el espacio de nombres
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-robot_namespace', namespace1,
            '-x', x_pose1,
            '-y', y_pose1,
            '-z', '0.01'
        ],
        output='screen',
    )

    #Spawner TB2
    
    model_folder_waffle = 'turtlebot3_waffle'
    urdf_path_waffle = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder_waffle,
        'model.sdf'
    )
    
    start_gazebo_ros_spawner_cmd_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # namespace=LaunchConfiguration('namespace2'),  # Usa el argumento de lanzamiento para el espacio de nombres
        arguments=[
            '-entity', 'waffle',
            '-file', urdf_path_waffle,
            '-robot_namespace', namespace2,
            '-x', x_pose2,
            '-y', y_pose2,
            '-z', '0.01'
        ],
        output='screen',
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # Declare the launch options
    ld.add_action(declare_x_position_cmd_1)
    ld.add_action(declare_y_position_cmd_1)
    ld.add_action(declare_x_position_cmd_2)
    ld.add_action(declare_y_position_cmd_2)
    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd_1)
    ld.add_action(start_gazebo_ros_spawner_cmd_2)


    return ld
