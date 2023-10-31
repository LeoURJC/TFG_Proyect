import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
        # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    

    use_sim_time = LaunchConfiguration('use_sim_time', default_value='true')
    x_pose1 = LaunchConfiguration('x_pose1', default='0.0')
    y_pose1 = LaunchConfiguration('y_pose1', default='0.0')

    x_pose2 = LaunchConfiguration('x_pose2', default='1.0')  # Adjust the x-coordinate for the second TurtleBot
    y_pose2 = LaunchConfiguration('y_pose2', default='1.0')  # Adjust the y-coordinate for the second TurtleBot

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
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )    

    # Declare launch arguments for the first TurtleBot
    declare_x_position_cmd_1 = DeclareLaunchArgument(
        'x_pose1', default_value='0.0',
        description='Specify x position of the first robot'
    )

    declare_y_position_cmd_1 = DeclareLaunchArgument(
        'y_pose1', default_value='0.0',
        description='Specify y position of the first robot'
    )

    # Declare launch arguments for the second TurtleBot
    declare_x_position_cmd_2 = DeclareLaunchArgument(
        'x_pose2', default_value='1.0',
        description='Specify x position of the second robot'
    )

    declare_y_position_cmd_2 = DeclareLaunchArgument(
        'y_pose2', default_value='1.0',
        description='Specify y position of the second robot'
    )
    
   

    # Create the nodes for spawning both TurtleBots
    start_gazebo_ros_spawner_cmd_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,  # Change entity name for the first TurtleBot
            '-file', urdf_path,
            '-x', LaunchConfiguration('x_pose1'),
            '-y', LaunchConfiguration('y_pose1'),
            '-z', '0.01'
        ],
        output='screen',
    )

    start_gazebo_ros_spawner_cmd_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,  # Change entity name for the second TurtleBot
            '-file', urdf_path,
            '-x', LaunchConfiguration('x_pose2'),
            '-y', LaunchConfiguration('x_pose2'),
            '-z', '0.01'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(declare_x_position_cmd_1)
    ld.add_action(declare_y_position_cmd_1)   
    ld.add_action(declare_x_position_cmd_2)
    ld.add_action(declare_y_position_cmd_2)
    ld.add_action(start_gazebo_ros_spawner_cmd_1)
    ld.add_action(start_gazebo_ros_spawner_cmd_2)

    return ld
