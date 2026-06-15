from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_pcd_dir = str(Path.home() / '.ros' / 'pct_planner' / 'pcd')
    default_tomogram_dir = str(Path.home() / '.ros' / 'pct_planner' / 'tomogram')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'rviz',
        'pct_ros2.rviz',
    ])
    marker_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'config',
        'start_goal_marker.yaml',
    ])
    show_rviz_condition = IfCondition(PythonExpression([
        "'true' == '",
        LaunchConfiguration('rviz'),
        "' or 'true' == '",
        LaunchConfiguration('show_rviz'),
        "'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='Spiral'),
        DeclareLaunchArgument('pcd_dir', default_value=default_pcd_dir),
        DeclareLaunchArgument('pcd_file', default_value=''),
        DeclareLaunchArgument('tomogram_file', default_value=''),
        DeclareLaunchArgument('tomogram_dir', default_value=default_tomogram_dir),
        DeclareLaunchArgument('planner_lib_dir', default_value=''),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/global_points'),
        DeclareLaunchArgument('tomogram_topic', default_value='/tomogram'),
        DeclareLaunchArgument('path_topic', default_value='/pct_path'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('show_rviz', default_value='false'),
        DeclareLaunchArgument('publish_map', default_value='true'),
        DeclareLaunchArgument('publish_raw_cloud', default_value='true'),
        DeclareLaunchArgument('publish_tomogram_cloud', default_value='true'),
        DeclareLaunchArgument('use_interactive_start_goal', default_value='true'),
        DeclareLaunchArgument('marker_config', default_value=marker_config),
        DeclareLaunchArgument('start_pose_topic', default_value='/pct_planner/start_pose'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/pct_planner/goal_pose'),
        DeclareLaunchArgument('plan_on_pose_update', default_value='true'),
        DeclareLaunchArgument('auto_run', default_value='true'),
        DeclareLaunchArgument('start_x', default_value='nan'),
        DeclareLaunchArgument('start_y', default_value='nan'),
        DeclareLaunchArgument('goal_x', default_value='nan'),
        DeclareLaunchArgument('goal_y', default_value='nan'),

        Node(
            package='pct_planner_ros2',
            executable='pct_map_publisher',
            name='pct_map_publisher',
            output='screen',
            parameters=[{
                'scene': LaunchConfiguration('scene'),
                'pcd_dir': LaunchConfiguration('pcd_dir'),
                'pcd_file': LaunchConfiguration('pcd_file'),
                'tomogram_file': LaunchConfiguration('tomogram_file'),
                'tomogram_dir': LaunchConfiguration('tomogram_dir'),
                'map_frame': LaunchConfiguration('map_frame'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'tomogram_topic': LaunchConfiguration('tomogram_topic'),
                'publish_raw_cloud': LaunchConfiguration('publish_raw_cloud'),
                'publish_tomogram_cloud': LaunchConfiguration('publish_tomogram_cloud'),
            }],
            condition=IfCondition(LaunchConfiguration('publish_map')),
        ),
        Node(
            package='pct_planner_ros2',
            executable='pct_start_goal_marker',
            name='pct_start_goal_marker',
            output='screen',
            parameters=[
                LaunchConfiguration('marker_config'),
                {
                    'frame_id': LaunchConfiguration('map_frame'),
                    'scene': LaunchConfiguration('scene'),
                    'use_scene_defaults': True,
                    'start_pose_topic': LaunchConfiguration('start_pose_topic'),
                    'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('use_interactive_start_goal')),
        ),
        Node(
            package='pct_planner_ros2',
            executable='pct_plan',
            name='pct_planner',
            output='screen',
            parameters=[{
                'scene': LaunchConfiguration('scene'),
                'tomogram_file': LaunchConfiguration('tomogram_file'),
                'tomogram_dir': LaunchConfiguration('tomogram_dir'),
                'planner_lib_dir': LaunchConfiguration('planner_lib_dir'),
                'map_frame': LaunchConfiguration('map_frame'),
                'path_topic': LaunchConfiguration('path_topic'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'start_pose_topic': LaunchConfiguration('start_pose_topic'),
                'goal_pose_topic': LaunchConfiguration('goal_pose_topic'),
                'plan_on_pose_update': LaunchConfiguration('plan_on_pose_update'),
                'auto_run': LaunchConfiguration('auto_run'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=show_rviz_condition,
        ),
    ])
