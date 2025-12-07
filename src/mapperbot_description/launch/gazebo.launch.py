import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join

def generate_launch_description():
    
    pkg_ros_gz_rbot = get_package_share_directory('mapperbot_description')
    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'mapperbot.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    gazebo_world = os.path.join(pkg_ros_gz_rbot, 'worlds', 'mazeworld.sdf')

    # Start Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Start Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', gazebo_world],
        output='screen'
    )

    # Mapperbot Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'mapperbot',
            '--no-namespacing',
            '-allow_renaming', 'true',
            '-z', '0.103',
            '-x', '0.0',
            '-y', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # ROS <> GZ Bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
        'config_file': ros_gz_bridge_config,
        }],
        output='screen'
    )

    # ros2_control: controllers
    controller_names = [
        'joint_state_broadcaster',
        'ackermann_steering_controller',
        'telescoping_controller',
    ]

    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
            name='spawner_' + name,
        )
        for name in controller_names
    ]

    return LaunchDescription([
        gazebo,
        spawn,
        start_gazebo_ros_bridge_cmd,
        robot_state_publisher,
        *controller_spawners,
    ])