import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TOMOGRAPHY_OVERRIDE_ARGS = [
    'scene',
    'pcd_dir',
    'pcd_file',
    'tomogram_dir',
    'benchmark_repeats',
    'map_frame',
    'pointcloud_topic',
    'layer_g_topic_prefix',
    'layer_c_topic_prefix',
    'tomogram_topic',
    'resolution',
    'ground_h',
    'slice_dh',
    'kernel_size',
    'interval_min',
    'interval_free',
    'slope_max',
    'step_max',
    'standable_ratio',
    'cost_barrier',
    'safe_margin',
    'inflation',
    'auto_run',
    'enable_preprocess',
    'preprocess_input_pcd',
    'preprocess_output_pcd',
    'preprocess_overwrite',
    'enable_manual_transform',
    'roll',
    'pitch',
    'yaw',
    'tx',
    'ty',
    'tz',
    'enable_auto_level',
    'ground_percentile',
    'max_ground_points',
    'ransac_distance_threshold',
    'ransac_n',
    'ransac_num_iterations',
    'normal_target_axis',
    'enable_ground_z_shift',
    'target_ground_z',
    'ground_z_shift_mode',
    'ground_z_shift_percentile',
    'random_seed',
]

BOOL_ARGS = {
    'auto_run',
    'enable_preprocess',
    'preprocess_overwrite',
    'enable_manual_transform',
    'enable_auto_level',
    'enable_ground_z_shift',
}
INT_ARGS = {
    'benchmark_repeats',
    'kernel_size',
    'max_ground_points',
    'ransac_n',
    'ransac_num_iterations',
    'random_seed',
}
FLOAT_ARGS = {
    'resolution',
    'ground_h',
    'slice_dh',
    'interval_min',
    'interval_free',
    'slope_max',
    'step_max',
    'standable_ratio',
    'cost_barrier',
    'safe_margin',
    'inflation',
    'roll',
    'pitch',
    'yaw',
    'tx',
    'ty',
    'tz',
    'ground_percentile',
    'ransac_distance_threshold',
    'target_ground_z',
    'ground_z_shift_percentile',
}
LIST_ARGS = {'normal_target_axis'}


def convert_override_value(name, value):
    if name in BOOL_ARGS:
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    if name in INT_ARGS:
        return int(value)
    if name in FLOAT_ARGS:
        return float(value)
    if name in LIST_ARGS:
        parsed = yaml.safe_load(value)
        if not isinstance(parsed, list):
            raise ValueError('%s must be a YAML list, for example "[0.0, 0.0, 1.0]"' % name)
        return parsed
    return value


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('tomography_config').perform(context)
    overrides = {}
    for name in TOMOGRAPHY_OVERRIDE_ARGS:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = convert_override_value(name, value)

    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='pct_planner_ros2',
            executable='pct_tomography',
            name='pct_tomography',
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'rviz',
        'pct_ros2.rviz',
    ])
    tomography_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'config',
        'tomography.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('tomography_config', default_value=tomography_config),
        DeclareLaunchArgument('scene', default_value=''),
        DeclareLaunchArgument('pcd_dir', default_value=''),
        DeclareLaunchArgument('pcd_file', default_value=''),
        DeclareLaunchArgument('tomogram_dir', default_value=''),
        DeclareLaunchArgument('benchmark_repeats', default_value=''),
        DeclareLaunchArgument('map_frame', default_value=''),
        DeclareLaunchArgument('pointcloud_topic', default_value=''),
        DeclareLaunchArgument('layer_g_topic_prefix', default_value=''),
        DeclareLaunchArgument('layer_c_topic_prefix', default_value=''),
        DeclareLaunchArgument('tomogram_topic', default_value=''),
        DeclareLaunchArgument('resolution', default_value=''),
        DeclareLaunchArgument('ground_h', default_value=''),
        DeclareLaunchArgument('slice_dh', default_value=''),
        DeclareLaunchArgument('kernel_size', default_value=''),
        DeclareLaunchArgument('interval_min', default_value=''),
        DeclareLaunchArgument('interval_free', default_value=''),
        DeclareLaunchArgument('slope_max', default_value=''),
        DeclareLaunchArgument('step_max', default_value=''),
        DeclareLaunchArgument('standable_ratio', default_value=''),
        DeclareLaunchArgument('cost_barrier', default_value=''),
        DeclareLaunchArgument('safe_margin', default_value=''),
        DeclareLaunchArgument('inflation', default_value=''),
        DeclareLaunchArgument('auto_run', default_value=''),
        DeclareLaunchArgument('enable_preprocess', default_value=''),
        DeclareLaunchArgument('preprocess_input_pcd', default_value=''),
        DeclareLaunchArgument('preprocess_output_pcd', default_value=''),
        DeclareLaunchArgument('preprocess_overwrite', default_value=''),
        DeclareLaunchArgument('enable_manual_transform', default_value=''),
        DeclareLaunchArgument('roll', default_value=''),
        DeclareLaunchArgument('pitch', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('tx', default_value=''),
        DeclareLaunchArgument('ty', default_value=''),
        DeclareLaunchArgument('tz', default_value=''),
        DeclareLaunchArgument('enable_auto_level', default_value=''),
        DeclareLaunchArgument('ground_percentile', default_value=''),
        DeclareLaunchArgument('max_ground_points', default_value=''),
        DeclareLaunchArgument('ransac_distance_threshold', default_value=''),
        DeclareLaunchArgument('ransac_n', default_value=''),
        DeclareLaunchArgument('ransac_num_iterations', default_value=''),
        DeclareLaunchArgument('normal_target_axis', default_value=''),
        DeclareLaunchArgument('enable_ground_z_shift', default_value=''),
        DeclareLaunchArgument('target_ground_z', default_value=''),
        DeclareLaunchArgument('ground_z_shift_mode', default_value=''),
        DeclareLaunchArgument('ground_z_shift_percentile', default_value=''),
        DeclareLaunchArgument('random_seed', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),

        OpaqueFunction(function=launch_setup),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
