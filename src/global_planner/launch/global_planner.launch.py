import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _read_debug_mode(config_path):
    try:
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        params = config.get("global_planner_node", {}).get("ros__parameters", {})
        return bool(params.get("debug_mode", True))
    except Exception:
        return True


def generate_launch_description():
    global_planner_share = get_package_share_directory("global_planner")
    config = os.path.join(global_planner_share, "config", "global_planner.yaml")
    debug_mode = _read_debug_mode(config)
    rviz_name = "global_planner.rviz" if debug_mode else "global_planner_localization.rviz"
    rviz_config = os.path.join(global_planner_share, "rviz", rviz_name)

    actions = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the global planner visualization config.",
        ),
        DeclareLaunchArgument(
            "start_livox_driver",
            default_value="false",
            description="Start livox_ros_driver2 when debug_mode is false.",
        ),
        DeclareLaunchArgument(
            "start_fastlio",
            default_value="true",
            description="Start FAST-LIO when debug_mode is false.",
        ),
    ]

    if not debug_mode:
        localization_launch = os.path.join(
            get_package_share_directory("localization_adapter"),
            "launch",
            "fastlio_icp_localization.launch",
        )
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(localization_launch),
                launch_arguments={
                    "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                    "start_fastlio": LaunchConfiguration("start_fastlio"),
                    "start_fastlio_rviz": "false",
                    "start_rviz": "false",
                }.items(),
            )
        )

    actions.extend([
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])

    return LaunchDescription(actions)
