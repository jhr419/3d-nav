from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    marker_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'config',
        'start_goal_marker.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'rviz',
        'pct_ros2.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_interactive_start_goal', default_value='true'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('marker_config', default_value=marker_config),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('scene', default_value=''),
        DeclareLaunchArgument('use_scene_defaults', default_value='false'),

        Node(
            package='pct_planner_ros2',
            executable='pct_start_goal_marker',
            name='pct_start_goal_marker',
            output='screen',
            parameters=[
                LaunchConfiguration('marker_config'),
                {
                    'frame_id': LaunchConfiguration('frame_id'),
                    'scene': LaunchConfiguration('scene'),
                    'use_scene_defaults': LaunchConfiguration('use_scene_defaults'),
                },
            ],
            condition=IfCondition(LaunchConfiguration('use_interactive_start_goal')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('show_rviz')),
        ),
    ])
