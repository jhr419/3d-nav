import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_planning_share = get_package_share_directory("local_planning")
    default_local_config_file = os.path.join(
        local_planning_share, "config", "local_planner.yaml"
    )

    local_config_file_arg = DeclareLaunchArgument(
        "local_config_file",
        default_value=default_local_config_file,
        description="YAML parameter file for local_planner_node",
    )
    launch_global_planner_arg = DeclareLaunchArgument(
        "launch_global_planner",
        default_value="true",
        description="Start octo_planner/jie_path_node together with the DWA local planner",
    )

    global_planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "goal_pose_topic": "/goal_pose",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "external_preblocked_marker_topic": "/edited_preblocked_cells_markers",
                "edited_occupied_marker_topic": "/edited_occupied_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "robot_radius": 0.25,
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 1,
                "enable_preblocked_costmap": True,
                "preblocked_costmap_radius_cells": 3,
                "preblocked_costmap_weight": 2.5,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_global_planner")),
    )

    local_planner_node = Node(
        package="local_planning",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("local_config_file")],
    )

    return LaunchDescription([
        local_config_file_arg,
        launch_global_planner_arg,
        global_planner_node,
        local_planner_node,
    ])
