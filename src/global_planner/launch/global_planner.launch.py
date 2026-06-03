import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("global_planner")
    localization_share = get_package_share_directory("localization_adapter")
    default_config = os.path.join(package_share, "config", "global_planner.yaml")
    default_rviz_config = os.path.join(package_share, "rviz", "global_planner.rviz")
    localization_launch = os.path.join(
        localization_share,
        "launch",
        "fastlio_icp_localization.launch",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_localization",
            default_value="true",
            description="Start FAST-LIO + ICP localization before global planning.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "start_livox_driver",
            default_value="true",
            description="Start livox_ros_driver2 through the localization launch.",
        ),
        DeclareLaunchArgument(
            "start_fastlio",
            default_value="true",
            description="Start FAST-LIO through the localization launch.",
        ),
        DeclareLaunchArgument(
            "start_fastlio_rviz",
            default_value="false",
            description="Start FAST-LIO RViz through the localization launch.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_config,
            description="Path to the global planner parameter file.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with the global planner visualization config.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Path to the RViz config.",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "start_fastlio_rviz": LaunchConfiguration("start_fastlio_rviz"),
                "start_rviz": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_localization")),
        ),
        Node(
            package="global_planner",
            executable="global_planner_node",
            name="global_planner_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
