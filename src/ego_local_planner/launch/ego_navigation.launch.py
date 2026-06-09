import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    launch_global_planner_arg = DeclareLaunchArgument(
        "launch_global_planner",
        default_value="false",
        description="Optionally launch octo_planner/jie_path_node when no external global planner is running",
    )
    launch_goal_bridge_arg = DeclareLaunchArgument(
        "launch_goal_bridge",
        default_value="false",
        description="Optionally convert RViz /goal_pose into /start_point and /goal_point",
    )
    launch_icp_start_bridge_arg = DeclareLaunchArgument(
        "launch_icp_start_bridge",
        default_value="false",
        description="Also publish /icp_pose as /start_point through octo_planner",
    )
    icp_pose_topic_arg = DeclareLaunchArgument(
        "icp_pose_topic",
        default_value="/icp_pose",
        description="PoseWithCovarianceStamped localization topic used as planner start",
    )

    goal_bridge_node = Node(
        package="ego_local_planner",
        executable="ego_navigation_goal_bridge_node",
        name="ego_navigation_goal_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_goal_bridge")),
        parameters=[
            {
                "map_frame": "map",
                "base_frame": "base_link",
                "base_frame_candidates": "base_link,base_footprint,odin1_base_link",
                "goal_pose_topic": "/goal_pose",
                "goal_point_topic": "/goal_point",
                "start_point_topic": "/start_point",
                "publish_start_from_tf": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    icp_start_bridge_node = Node(
        package="octo_planner",
        executable="icp_pose_to_start_point_node",
        name="icp_pose_to_start_point",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_icp_start_bridge")),
        parameters=[
            {
                "input_pose_topic": LaunchConfiguration("icp_pose_topic"),
                "output_point_topic": "/start_point",
                "frame_id": "map",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    global_planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_global_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "goal_pose_topic": "/goal_pose",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    local_planner_node = Node(
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
        launch_global_planner_arg,
        launch_goal_bridge_arg,
        launch_icp_start_bridge_arg,
        icp_pose_topic_arg,
        goal_bridge_node,
        icp_start_bridge_node,
        global_planner_node,
        local_planner_node,
    ])
