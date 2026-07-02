import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_share = get_package_share_directory("pct_global_planner")
    default_config_file = os.path.join(
        planner_share,
        "config",
        "pct_global_planner.yaml",
    )
    default_rviz_config_file = os.path.join(
        planner_share,
        "rviz",
        "pct_global_planner.rviz",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="YAML parameter file for pct_global_planner_node",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="maps/map_preprocessed.pcd",
        description="Metadata/static map file path. PCT planning uses tomogram_file.",
    )
    pcd_file_arg = DeclareLaunchArgument(
        "pcd_file",
        default_value="maps/map_preprocessed.pcd",
        description="Metadata/source PCD path used when regenerating PCT tomograms.",
    )
    tomogram_file_arg = DeclareLaunchArgument(
        "tomogram_file",
        default_value="map_preprocessed",
        description="PCT tomogram pickle stem or absolute .pickle path",
    )
    tomogram_dir_arg = DeclareLaunchArgument(
        "tomogram_dir",
        default_value="maps/tomogram",
        description="Directory containing PCT tomogram pickle files",
    )
    planner_lib_dir_arg = DeclareLaunchArgument(
        "planner_lib_dir",
        default_value="src/map_process/PCT_planner/planner/lib",
        description="PCT Planner core library directory",
    )
    launch_map_publisher_arg = DeclareLaunchArgument(
        "launch_map_publisher",
        default_value="true",
        description="Launch map publisher for /global_points and /tomogram",
    )
    publish_raw_cloud_arg = DeclareLaunchArgument(
        "publish_raw_cloud",
        default_value="true",
        description="Publish raw PCD map on /global_points",
    )
    publish_tomogram_cloud_arg = DeclareLaunchArgument(
        "publish_tomogram_cloud",
        default_value="true",
        description="Publish tomogram visualization cloud on /tomogram",
    )
    launch_goal_marker_arg = DeclareLaunchArgument(
        "launch_goal_marker",
        default_value="true",
        description="Launch interactive 3D goal marker for RViz",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz for global path and marker visualization",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="RViz config file",
    )

    planner_node = Node(
        package="pct_global_planner",
        executable="pct_global_planner_node",
        name="pct_global_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "map_file": LaunchConfiguration("map_file"),
                "pcd_file": LaunchConfiguration("pcd_file"),
                "tomogram_file": LaunchConfiguration("tomogram_file"),
                "tomogram_dir": LaunchConfiguration("tomogram_dir"),
                "planner_lib_dir": LaunchConfiguration("planner_lib_dir"),
            },
        ],
    )

    map_publisher_node = Node(
        package="pct_global_planner",
        executable="pct_global_map_publisher_node",
        name="pct_global_map_publisher",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
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
            },
        ],
        condition=IfCondition(LaunchConfiguration("launch_map_publisher")),
    )

    goal_marker_node = Node(
        package="pct_global_planner",
        executable="pct_goal_marker_node",
        name="pct_start_goal_marker",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration("launch_goal_marker")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="planning_rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            config_file_arg,
            use_sim_time_arg,
            map_file_arg,
            pcd_file_arg,
            tomogram_file_arg,
            tomogram_dir_arg,
            planner_lib_dir_arg,
            launch_map_publisher_arg,
            publish_raw_cloud_arg,
            publish_tomogram_cloud_arg,
            launch_goal_marker_arg,
            launch_rviz_arg,
            rviz_config_arg,
            LogInfo(
                msg=[
                    "pct_global_planner: publishing map, goal marker, and /planned_path",
                ]
            ),
            map_publisher_node,
            planner_node,
            goal_marker_node,
            rviz_node,
        ]
    )
