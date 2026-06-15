import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    backend_share = get_package_share_directory('sc_lio_sam_backend')
    backend_prefix = os.path.dirname(os.path.dirname(backend_share))

    fast_lio_launch = os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
    backend_config = os.path.join(backend_share, 'config', 'sc_lio_sam_backend.yaml')
    backend_rviz_config = os.path.join(backend_share, 'rviz', 'sc_lio_sam_backend.rviz')
    gtsam_abi_path = os.path.join(
        backend_prefix, 'lib', 'sc_lio_sam_backend', 'gtsam_abi')

    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    backend_cloud_topic = LaunchConfiguration('backend_cloud_topic')
    publish_backend_tf = LaunchConfiguration('publish_backend_tf')
    enable_loop_closure = LaunchConfiguration('enable_loop_closure')
    backend_rviz = LaunchConfiguration('backend_rviz')
    backend_rviz_cfg = LaunchConfiguration('backend_rviz_cfg')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('fast_lio_config_file', default_value='mid360.yaml'),
        DeclareLaunchArgument('backend_cloud_topic', default_value='/cloud_registered_body'),
        DeclareLaunchArgument('publish_backend_tf', default_value='true'),
        DeclareLaunchArgument('enable_loop_closure', default_value='false'),
        DeclareLaunchArgument('backend_rviz', default_value='true'),
        DeclareLaunchArgument('backend_rviz_cfg', default_value=backend_rviz_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config_file': fast_lio_config_file,
                'rviz': 'false',
            }.items(),
        ),
        Node(
            package='sc_lio_sam_backend',
            executable='sc_lio_sam_backend_node',
            name='sc_lio_sam_backend',
            output='screen',
            additional_env={
                'LD_LIBRARY_PATH': [
                    gtsam_abi_path,
                    ':',
                    EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
                ],
            },
            parameters=[
                backend_config,
                {
                    'use_sim_time': use_sim_time,
                    'cloud_topic': backend_cloud_topic,
                    'publish_tf': publish_backend_tf,
                    'enable_loop_closure': enable_loop_closure,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', backend_rviz_cfg],
            condition=IfCondition(backend_rviz),
            output='screen',
        ),
    ])
