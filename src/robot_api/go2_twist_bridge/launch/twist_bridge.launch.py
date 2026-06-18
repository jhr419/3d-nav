import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("go2_twist_bridge")
    default_config_file = os.path.join(package_share, "config", "twist_bridge.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="YAML parameter file for twist_bridge_node",
    )

    twist_bridge_node = Node(
        package="go2_twist_bridge",
        executable="twist_bridge",
        name="twist_bridge_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([
        config_file_arg,
        twist_bridge_node,
    ])
