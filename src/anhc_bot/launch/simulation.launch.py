#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    anhc_bot_path = get_package_share_directory("anhc_bot")
    world_file = LaunchConfiguration(
        "world_file",
        default=join(anhc_bot_path, "worlds", "factory.sdf")
    )

    # Vị trí mặc định dựa trên world
    # Nếu là factory thì spawn ở (2.0, 2.0), nếu không thì (0.0, 0.0)
    default_x = PythonExpression(["'2.0' if 'factory' in '", world_file, "' else '0.0'"])
    default_y = PythonExpression(["'2.0' if 'factory' in '", world_file, "' else '0.0'"])
    default_yaw = PythonExpression(["'1.57' if 'factory' in '", world_file, "' else '0.0'"])

    position_x = LaunchConfiguration("position_x", default=default_x)
    position_y = LaunchConfiguration("position_y", default=default_y)
    orientation_yaw = LaunchConfiguration("orientation_yaw", default=default_yaw)
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    spawn_anhc_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(anhc_bot_path, "launch", "anhc_bot_gz_spawn.launch.py")
        ),
        launch_arguments={
            "position_x": position_x,
            "position_y": position_y,
            "orientation_yaw": orientation_yaw
        }.items()
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(anhc_bot_path, "worlds")),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(anhc_bot_path, "worlds", "models")),

        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=join(anhc_bot_path, "models")),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        DeclareLaunchArgument("position_x", default_value=position_x),
        DeclareLaunchArgument("position_y", default_value=position_y),
        DeclareLaunchArgument("orientation_yaw", default_value=orientation_yaw),

        gz_sim, spawn_anhc_bot_node
    ])
