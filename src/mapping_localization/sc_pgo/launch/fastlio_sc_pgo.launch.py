import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    sc_pgo_share = get_package_share_directory('sc_pgo')

    fastlio_config_file = LaunchConfiguration('fastlio_config_file')
    sc_pgo_config_file = LaunchConfiguration('sc_pgo_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    fastlio_rviz = LaunchConfiguration('fastlio_rviz')

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': fastlio_config_file,
            'use_sim_time': use_sim_time,
            'rviz': fastlio_rviz,
        }.items(),
    )

    sc_pgo_node = Node(
        package='sc_pgo',
        executable='sc_pgo_node',
        name='sc_pgo_node',
        output='screen',
        parameters=[
            sc_pgo_config_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio_config_file',
            default_value='mid360.yaml',
            description='FAST-LIO config file name under fast_lio/config',
        ),
        DeclareLaunchArgument(
            'sc_pgo_config_file',
            default_value=os.path.join(sc_pgo_share, 'config', 'sc_pgo.yaml'),
            description='SC-PGO YAML parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock when replaying bags with simulated time',
        ),
        DeclareLaunchArgument(
            'fastlio_rviz',
            default_value='true',
            description='Whether to start FAST-LIO RViz from mapping.launch.py',
        ),
        LogInfo(msg=[
            'Starting FAST-LIO2 + SC-PGO. FAST-LIO config=', fastlio_config_file,
            ', SC-PGO config=', sc_pgo_config_file,
        ]),
        fast_lio_launch,
        sc_pgo_node,
    ])
