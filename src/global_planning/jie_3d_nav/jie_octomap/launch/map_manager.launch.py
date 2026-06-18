from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="Launch jie_path_node for interactive path planning",
    )
    launch_icp_start_bridge_arg = DeclareLaunchArgument(
        "launch_icp_start_bridge",
        default_value="true",
        description="Publish localization_adapter /icp_pose as /start_point",
    )
    icp_pose_topic_arg = DeclareLaunchArgument(
        "icp_pose_topic",
        default_value="/icp_pose",
        description="PoseWithCovarianceStamped localization topic used as planner start",
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
            }
        ],
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "loaded_map",
                "source_world_file": "",
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
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
    )

    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
    )

    shutdown_when_viewer_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=map_viewer_gui_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason="map_viewer_gui closed")),
            ],
        )
    )

    return LaunchDescription(
        [
            launch_planner_arg,
            launch_icp_start_bridge_arg,
            icp_pose_topic_arg,
            icp_start_bridge_node,
            planner_node,
            map_package_manager_node,
            occupied_marker_node,
            map_viewer_gui_node,
            shutdown_when_viewer_exits,
        ]
    )
