import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'sc_pgo'
    package_share = get_package_share_directory(package_name)
    default_config_file = os.path.join(package_share, 'config', 'sc_pgo.yaml')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_topic = LaunchConfiguration('odom_topic')
    keyframe_cloud_topic = LaunchConfiguration('keyframe_cloud_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='SC-PGO YAML parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock when replaying bags with simulated time',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/Odometry',
            description='FAST-LIO odometry topic',
        ),
        DeclareLaunchArgument(
            'keyframe_cloud_topic',
            default_value='/cloud_registered',
            description='FAST-LIO registered cloud topic used for keyframes',
        ),
        LogInfo(msg=[
            'Starting SC-PGO only. Subscribing odom=', odom_topic,
            ', keyframe_cloud=', keyframe_cloud_topic,
            ', config=', config_file,
            '. Publishing /pgo/optimized_odometry, /pgo/optimized_path, /pgo/optimized_map, /pgo/loop_markers.',
        ]),
        Node(
            package=package_name,
            executable='sc_pgo_node',
            name='sc_pgo_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_topic': odom_topic,
                    'cloud_topic': keyframe_cloud_topic,
                    'keyframe_cloud_topic': keyframe_cloud_topic,
                },
            ],
        ),
    ])
