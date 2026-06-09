import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = get_package_share_directory("ego_local_planner")
    default_config_file = os.path.join(
        planner_share, "config", "ego_local_planner.yaml"
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="YAML parameter file for ego_local_planner_node",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    planner_node = Node(
        package="ego_local_planner",
        executable="ego_local_planner_node",
        name="ego_local_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        planner_node,
    ])
