import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_planning_share = get_package_share_directory("local_planning")
    default_config_file = os.path.join(
        local_planning_share, "config", "local_planner.yaml"
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="YAML parameter file for local_planner_node",
    )

    local_planner_node = Node(
        package="local_planning",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_file_arg,
        local_planner_node,
    ])
