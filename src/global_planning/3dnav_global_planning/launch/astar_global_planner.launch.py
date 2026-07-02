import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_share = get_package_share_directory("nav3d_global_planning")
    pct_share = get_package_share_directory("pct_global_planner")
    default_config_file = os.path.join(
        planner_share,
        "config",
        "astar_global_planner.yaml",
    )
    default_rviz_config_file = os.path.join(
        pct_share,
        "rviz",
        "pct_global_planner.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="YAML parameter file for astar_global_planner_node",
            ),
            DeclareLaunchArgument(
                "pcd_file",
                default_value="maps/map_preprocessed.pcd",
                description="PCD map file published on /global_points for RViz.",
            ),
            DeclareLaunchArgument(
                "tomogram_file",
                default_value="map_preprocessed",
                description="PCT tomogram pickle stem published on /tomogram for RViz.",
            ),
            DeclareLaunchArgument(
                "tomogram_dir",
                default_value="maps/tomogram",
                description="Directory containing PCT tomogram pickle files.",
            ),
            DeclareLaunchArgument(
                "launch_map_publisher",
                default_value="true",
                description="Launch PCT map publisher for /global_points and /tomogram.",
            ),
            DeclareLaunchArgument(
                "publish_raw_cloud",
                default_value="true",
                description="Publish raw PCD map on /global_points.",
            ),
            DeclareLaunchArgument(
                "publish_tomogram_cloud",
                default_value="true",
                description="Publish tomogram visualization cloud on /tomogram.",
            ),
            DeclareLaunchArgument(
                "launch_goal_marker",
                default_value="true",
                description="Launch the same interactive 3D goal marker used by PCT.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz for A* global path and debug visualization.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_file,
                description="RViz config file",
            ),
            LogInfo(
                msg=[
                    "astar_global_planner: using PCT RViz/map/goal-marker visualization; A* publishes /planned_path from config ",
                    LaunchConfiguration("config_file"),
                ]
            ),
            Node(
                package="pct_global_planner",
                executable="pct_global_map_publisher_node",
                name="pct_global_map_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "pcd_file": LaunchConfiguration("pcd_file"),
                        "tomogram_file": LaunchConfiguration("tomogram_file"),
                        "tomogram_dir": LaunchConfiguration("tomogram_dir"),
                        "publish_raw_cloud": ParameterValue(
                            LaunchConfiguration("publish_raw_cloud"),
                            value_type=bool,
                        ),
                        "publish_tomogram_cloud": ParameterValue(
                            LaunchConfiguration("publish_tomogram_cloud"),
                            value_type=bool,
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("launch_map_publisher")),
            ),
            Node(
                package="nav3d_global_planning",
                executable="astar_global_planner_node",
                name="astar_global_planner_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="pct_global_planner",
                executable="pct_goal_marker_node",
                name="pct_start_goal_marker",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        )
                    }
                ],
                condition=IfCondition(LaunchConfiguration("launch_goal_marker")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="planning_rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
        ]
    )
