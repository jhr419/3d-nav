import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('sc_lio_sam_backend')
    package_prefix = os.path.dirname(os.path.dirname(package_share))
    default_config = os.path.join(package_share, 'config', 'sc_lio_sam_backend.yaml')
    gtsam_abi_path = os.path.join(
        package_prefix, 'lib', 'sc_lio_sam_backend', 'gtsam_abi')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    publish_tf = LaunchConfiguration('publish_tf')
    enable_loop_closure = LaunchConfiguration('enable_loop_closure')
    save_pcd = LaunchConfiguration('save_pcd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to the SC-LIO-SAM backend YAML config.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated clock.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='Frontend odometry topic.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered_body',
            description='Frontend keyframe/current cloud topic.'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish backend TF correction.'),
        DeclareLaunchArgument(
            'enable_loop_closure',
            default_value='false',
            description='Enable Scan Context loop factors. Keep false for corridor debugging.'),
        DeclareLaunchArgument(
            'save_pcd',
            default_value='false',
            description='Save optimized PCD map on node shutdown.'),
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
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_topic': odom_topic,
                    'cloud_topic': cloud_topic,
                    'publish_tf': publish_tf,
                    'enable_loop_closure': enable_loop_closure,
                    'save_pcd': save_pcd,
                },
            ],
        ),
    ])
