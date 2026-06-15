from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_pcd_dir = str(Path.home() / '.ros' / 'pct_planner' / 'pcd')
    default_tomogram_dir = str(Path.home() / '.ros' / 'pct_planner' / 'tomogram')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pct_planner_ros2'),
        'rviz',
        'pct_ros2.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='Spiral'),
        DeclareLaunchArgument('pcd_dir', default_value=default_pcd_dir),
        DeclareLaunchArgument('pcd_file', default_value=''),
        DeclareLaunchArgument('tomogram_dir', default_value=default_tomogram_dir),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('benchmark_repeats', default_value='10'),
        DeclareLaunchArgument('rviz', default_value='false'),

        Node(
            package='pct_planner_ros2',
            executable='pct_tomography',
            name='pct_tomography',
            output='screen',
            parameters=[{
                'scene': LaunchConfiguration('scene'),
                'pcd_dir': LaunchConfiguration('pcd_dir'),
                'pcd_file': LaunchConfiguration('pcd_file'),
                'tomogram_dir': LaunchConfiguration('tomogram_dir'),
                'map_frame': LaunchConfiguration('map_frame'),
                'benchmark_repeats': LaunchConfiguration('benchmark_repeats'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
