from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


OVERRIDE_ARGS = [
    'input_pcd',
    'output_pcd',
    'frame_id',
    'processed_points_topic',
    'publish_period_sec',
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
    'ground_z_shift_mode',
    'ground_z_shift_percentile',
    'target_ground_z',
    'random_seed',
]

BOOL_ARGS = {
    'enable_manual_transform',
    'enable_auto_level',
    'enable_ground_z_shift',
}

INT_ARGS = {
    'max_ground_points',
    'ransac_n',
    'ransac_num_iterations',
    'random_seed',
}

FLOAT_ARGS = {
    'publish_period_sec',
    'roll',
    'pitch',
    'yaw',
    'tx',
    'ty',
    'tz',
    'ground_percentile',
    'ransac_distance_threshold',
    'ground_z_shift_percentile',
    'target_ground_z',
}


def as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def parse_list(value):
    text = value.strip()
    if text.startswith('[') and text.endswith(']'):
        text = text[1:-1]
    return [float(item.strip()) for item in text.split(',') if item.strip()]


def convert_override_value(name, value):
    if name in BOOL_ARGS:
        return as_bool(value)
    if name in INT_ARGS:
        return int(value)
    if name in FLOAT_ARGS:
        return float(value)
    if name == 'normal_target_axis':
        return parse_list(value)
    return value


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    overrides = {}
    for name in OVERRIDE_ARGS:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = convert_override_value(name, value)

    parameters = [config_file]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='pcd_preprocessor_ros2',
            executable='pcd_preprocessor_node',
            name='pcd_preprocessor_node',
            output='screen',
            parameters=parameters,
        )
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('pcd_preprocessor_ros2'),
        'config',
        'pcd_preprocessor.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pcd_preprocessor_ros2'),
        'rviz',
        'pcd_preprocessor.rviz',
    ])

    declarations = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    declarations.extend(DeclareLaunchArgument(name, default_value='') for name in OVERRIDE_ARGS)

    return LaunchDescription([
        *declarations,
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
