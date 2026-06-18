from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_bin_path = LaunchConfiguration("input_bin_path")
    output_pcd_path = LaunchConfiguration("output_pcd_path")
    output_dir = LaunchConfiguration("output_dir")
    fields_per_point = LaunchConfiguration("fields_per_point")
    intensity_field_index = LaunchConfiguration("intensity_field_index")
    save_binary = LaunchConfiguration("save_binary")
    overwrite = LaunchConfiguration("overwrite")
    skip_invalid_points = LaunchConfiguration("skip_invalid_points")
    exit_after_save = LaunchConfiguration("exit_after_save")

    return LaunchDescription([
        DeclareLaunchArgument("input_bin_path", default_value=""),
        DeclareLaunchArgument("output_pcd_path", default_value=""),
        DeclareLaunchArgument("output_dir", default_value=""),
        DeclareLaunchArgument("fields_per_point", default_value="4"),
        DeclareLaunchArgument("intensity_field_index", default_value="3"),
        DeclareLaunchArgument("save_binary", default_value="true"),
        DeclareLaunchArgument("overwrite", default_value="true"),
        DeclareLaunchArgument("skip_invalid_points", default_value="true"),
        DeclareLaunchArgument("exit_after_save", default_value="true"),
        Node(
            package="map_mannger",
            executable="bin2pcd_node",
            name="bin2pcd_node",
            output="screen",
            parameters=[{
                "input_bin_path": input_bin_path,
                "output_pcd_path": output_pcd_path,
                "output_dir": output_dir,
                "fields_per_point": ParameterValue(fields_per_point, value_type=int),
                "intensity_field_index": ParameterValue(intensity_field_index, value_type=int),
                "save_binary": ParameterValue(save_binary, value_type=bool),
                "overwrite": ParameterValue(overwrite, value_type=bool),
                "skip_invalid_points": ParameterValue(skip_invalid_points, value_type=bool),
                "exit_after_save": ParameterValue(exit_after_save, value_type=bool),
            }],
        ),
    ])
