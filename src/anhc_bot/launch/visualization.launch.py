#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    isaac_sim = LaunchConfiguration('isaac_sim')

    # Paths
    anhc_bot_path = get_package_share_directory('anhc_bot')
    xacro_path = os.path.join(anhc_bot_path, 'urdf', 'anhc_bot.xacro')

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', xacro_path,
                ' wheel_odom_topic:=', 'odom'
            ])
        }],
        condition=UnlessCondition(isaac_sim)  # Only start if isaac_sim is false
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(anhc_bot_path, 'rviz', 'entire_setup.rviz')]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'isaac_sim',
            default_value='false',
            description='Set to true when using Isaac Sim'
        ),
        # Nodes
        robot_state_publisher,
        rviz
    ])
