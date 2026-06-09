import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory("fast_lio")
    sc_pgo_share = get_package_share_directory("sc_pgo")

    fast_lio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    sc_pgo_config = os.path.join(sc_pgo_share, "config", "fastlio_sc_pgo.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    fast_lio_config_file = LaunchConfiguration("fast_lio_config_file")
    rviz = LaunchConfiguration("rviz")
    save_directory = LaunchConfiguration("save_directory")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fast_lio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("save_directory", default_value="maps/sc_pgo"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "config_path": os.path.join(fast_lio_share, "config"),
                "config_file": fast_lio_config_file,
                "rviz": rviz,
            }.items(),
        ),
        Node(
            package="sc_pgo",
            executable="sc_pgo_node",
            name="sc_pgo_node",
            output="screen",
            parameters=[
                sc_pgo_config,
                {"use_sim_time": use_sim_time},
                {"save_directory": save_directory},
            ],
        ),
    ])
