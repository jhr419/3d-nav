import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pct_share = get_package_share_directory("pct_global_planner")
    ego_share = get_package_share_directory("ego_local_planner")

    default_pct_config = os.path.join(
        pct_share,
        "config",
        "pct_global_planner.yaml",
    )
    default_ego_config = os.path.join(
        ego_share,
        "config",
        "ego_local_planner.yaml",
    )

    pct_config_arg = DeclareLaunchArgument(
        "pct_config_file",
        default_value=default_pct_config,
        description="YAML parameter file for pct_global_planner_node",
    )
    ego_config_arg = DeclareLaunchArgument(
        "ego_config_file",
        default_value=default_ego_config,
        description="YAML parameter file for ego_local_planner_node",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="/home/jhr/3dnav_ws/maps/result_cleaned.bt",
        description="Metadata/static map file path. PCT planning uses tomogram_file.",
    )
    tomogram_file_arg = DeclareLaunchArgument(
        "tomogram_file",
        default_value="map_leveled",
        description="PCT tomogram pickle stem or absolute .pickle path",
    )
    tomogram_dir_arg = DeclareLaunchArgument(
        "tomogram_dir",
        default_value="/home/jhr/.ros/pct_planner/tomogram",
        description="Directory containing PCT tomogram pickle files",
    )
    planner_lib_dir_arg = DeclareLaunchArgument(
        "planner_lib_dir",
        default_value="/home/jhr/3dnav_ws/src/PCT_planner/planner/lib",
        description="PCT Planner core library directory",
    )

    pct_node = Node(
        package="pct_global_planner",
        executable="pct_global_planner_node",
        name="pct_global_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("pct_config_file"),
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "map_file": LaunchConfiguration("map_file"),
                "tomogram_file": LaunchConfiguration("tomogram_file"),
                "tomogram_dir": LaunchConfiguration("tomogram_dir"),
                "planner_lib_dir": LaunchConfiguration("planner_lib_dir"),
                "publish_path_topic": "/planned_path",
                "publish_alias_path_topic": "/path",
                "map_frame": "map",
            },
        ],
    )

    ego_node = Node(
        package="ego_local_planner",
        executable="ego_local_planner_node",
        name="ego_local_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ego_config_file"),
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "global_path_topic": "/planned_path",
                "map_frame": "map",
            },
        ],
    )

    return LaunchDescription(
        [
            pct_config_arg,
            ego_config_arg,
            use_sim_time_arg,
            map_file_arg,
            tomogram_file_arg,
            tomogram_dir_arg,
            planner_lib_dir_arg,
            LogInfo(msg=["PCT Global Planner publishes nav_msgs/Path on /planned_path in frame map."]),
            LogInfo(msg=["EGO local planner subscribes global_path_topic=/planned_path and publishes /cmd_vel."]),
            LogInfo(msg=["FAST-LIO/ICP localization is not launched here; provide TF map->base_link externally."]),
            pct_node,
            ego_node,
        ]
    )
