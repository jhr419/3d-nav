import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("sc_pgo")
    default_config = os.path.join(package_path, "config", "fastlio_sc_pgo.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    save_directory = LaunchConfiguration("save_directory")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="SC-PGO parameter YAML",
        ),
        DeclareLaunchArgument(
            "save_directory",
            default_value="maps/sc_pgo",
            description="Directory for optimized_poses.txt, odom_poses.txt, times.txt and Scans",
        ),
        Node(
            package="sc_pgo",
            executable="sc_pgo_node",
            name="sc_pgo_node",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": use_sim_time},
                {"save_directory": save_directory},
            ],
        ),
    ])
