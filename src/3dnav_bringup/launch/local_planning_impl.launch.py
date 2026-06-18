import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "config_file",
            default_value=package_file("ego_local_planner", "config", "ego_local_planner.yaml"),
        ),
        Node(
            package="ego_local_planner",
            executable="ego_local_planner_node",
            name="ego_local_planner_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "global_path_topic": "/planned_path",
                    "cmd_vel_topic": "/cmd_vel_nav",
                },
            ],
        ),
    ])
