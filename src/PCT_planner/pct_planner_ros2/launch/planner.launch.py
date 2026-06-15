from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BOOL_ARGS = {
    'auto_run',
    'plan_on_pose_update',
    'publish_map',
    'publish_marker_array',
    'publish_on_feedback',
    'publish_initial_poses',
    'publish_raw_cloud',
    'publish_tomogram_cloud',
    'rviz',
    'show_rviz',
    'use_interactive_start_goal',
    'use_quintic',
    'use_scene_defaults',
}

INT_ARGS = set()

FLOAT_ARGS = {
    'default_goal_pitch',
    'default_goal_roll',
    'default_goal_x',
    'default_goal_y',
    'default_goal_yaw',
    'default_goal_z',
    'default_start_pitch',
    'default_start_roll',
    'default_start_x',
    'default_start_y',
    'default_start_yaw',
    'default_start_z',
    'goal_x',
    'goal_y',
    'marker_scale',
    'max_heading_rate',
    'path_z_offset',
    'astar_cost_threshold',
    'astar_step_cost_weight',
    'optimizer_safe_cost_threshold',
    'sphere_radius',
    'start_x',
    'start_y',
}

LAUNCH_OVERRIDE_ARGS = [
    'publish_map',
    'use_interactive_start_goal',
    'show_rviz',
    'rviz',
    'rviz_config',
]

PLANNER_OVERRIDE_ARGS = [
    'scene',
    'tomogram_file',
    'tomogram_dir',
    'planner_lib_dir',
    'use_quintic',
    'max_heading_rate',
    'path_z_offset',
    'astar_cost_threshold',
    'astar_step_cost_weight',
    'optimizer_safe_cost_threshold',
    'map_frame',
    'path_topic',
    'start_x',
    'start_y',
    'goal_x',
    'goal_y',
    'start_pose_topic',
    'goal_pose_topic',
    'plan_on_pose_update',
    'auto_run',
]

MAP_OVERRIDE_ARGS = [
    'scene',
    'pcd_dir',
    'pcd_file',
    'tomogram_dir',
    'tomogram_file',
    'map_frame',
    'pointcloud_topic',
    'tomogram_topic',
    'publish_raw_cloud',
    'publish_tomogram_cloud',
]

MARKER_OVERRIDE_ARGS = [
    'scene',
    'frame_id',
    'use_scene_defaults',
    'start_pose_topic',
    'goal_pose_topic',
    'marker_array_topic',
    'server_namespace',
    'marker_scale',
    'sphere_radius',
    'publish_on_feedback',
    'publish_initial_poses',
    'publish_marker_array',
    'default_start_x',
    'default_start_y',
    'default_start_z',
    'default_start_roll',
    'default_start_pitch',
    'default_start_yaw',
    'default_goal_x',
    'default_goal_y',
    'default_goal_z',
    'default_goal_roll',
    'default_goal_pitch',
    'default_goal_yaw',
]

ALL_OVERRIDE_ARGS = sorted(
    set(
        LAUNCH_OVERRIDE_ARGS
        + PLANNER_OVERRIDE_ARGS
        + MAP_OVERRIDE_ARGS
        + MARKER_OVERRIDE_ARGS
    )
)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def convert_override_value(name, value):
    if name in BOOL_ARGS:
        return as_bool(value)
    if name in INT_ARGS:
        return int(value)
    if name in FLOAT_ARGS:
        return float(value)
    return value


def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def launch_params(config):
    return dict(config.get('planner_launch', {}).get('ros__parameters', {}) or {})


def command_line_overrides(context, names):
    overrides = {}
    for name in names:
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            overrides[name] = convert_override_value(name, value)
    return overrides


def filtered(overrides, names):
    return {name: overrides[name] for name in names if name in overrides}


def default_rviz_config():
    return str(Path(get_package_share_directory('pct_planner_ros2')) / 'rviz' / 'pct_ros2.rviz')


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('planner_config').perform(context)
    config = load_yaml(config_file)
    overrides = command_line_overrides(context, ALL_OVERRIDE_ARGS)

    launch_cfg = launch_params(config)
    launch_cfg.update(filtered(overrides, LAUNCH_OVERRIDE_ARGS))
    if 'rviz' in overrides and 'show_rviz' not in overrides:
        launch_cfg['show_rviz'] = overrides['rviz']

    planner_overrides = filtered(overrides, PLANNER_OVERRIDE_ARGS)
    map_overrides = filtered(overrides, MAP_OVERRIDE_ARGS)
    marker_overrides = filtered(overrides, MARKER_OVERRIDE_ARGS)

    if 'map_frame' in overrides and 'frame_id' not in marker_overrides:
        marker_overrides['frame_id'] = overrides['map_frame']

    actions = []

    if as_bool(launch_cfg.get('publish_map', True)):
        actions.append(
            Node(
                package='pct_planner_ros2',
                executable='pct_map_publisher',
                name='pct_map_publisher',
                output='screen',
                parameters=[config_file, map_overrides] if map_overrides else [config_file],
            )
        )

    if as_bool(launch_cfg.get('use_interactive_start_goal', True)):
        actions.append(
            Node(
                package='pct_planner_ros2',
                executable='pct_start_goal_marker',
                name='pct_start_goal_marker',
                output='screen',
                parameters=[config_file, marker_overrides] if marker_overrides else [config_file],
            )
        )

    actions.append(
        Node(
            package='pct_planner_ros2',
            executable='pct_plan',
            name='pct_planner',
            output='screen',
            parameters=[config_file, planner_overrides] if planner_overrides else [config_file],
        )
    )

    show_rviz = as_bool(launch_cfg.get('show_rviz', False)) or as_bool(launch_cfg.get('rviz', False))
    if show_rviz:
        rviz_config = launch_cfg.get('rviz_config') or default_rviz_config()
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        )

    return actions


def generate_launch_description():
    planner_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'config',
        'planner.yaml',
    ])

    declarations = [
        DeclareLaunchArgument('planner_config', default_value=planner_config),
    ]
    declarations.extend(
        DeclareLaunchArgument(name, default_value='')
        for name in ALL_OVERRIDE_ARGS
    )

    return LaunchDescription([
        *declarations,
        OpaqueFunction(function=launch_setup),
    ])
