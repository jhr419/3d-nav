import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    rosbag_enable = LaunchConfiguration('rosbag_enable')
    rosbag_path = LaunchConfiguration('rosbag_path')
    rosbag_storage_id = LaunchConfiguration('rosbag_storage_id')
    rosbag_auto_shutdown = LaunchConfiguration('rosbag_auto_shutdown')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_rosbag_enable_cmd = DeclareLaunchArgument(
        'rosbag_enable', default_value='false',
        description='Read ROS 2 bag directly instead of subscribing to live topics'
    )
    declare_rosbag_path_cmd = DeclareLaunchArgument(
        'rosbag_path', default_value='',
        description='ROS 2 bag directory path used when rosbag_enable is true'
    )
    declare_rosbag_storage_id_cmd = DeclareLaunchArgument(
        'rosbag_storage_id', default_value='sqlite3',
        description='ROS 2 bag storage backend, for example sqlite3 or mcap'
    )
    declare_rosbag_auto_shutdown_cmd = DeclareLaunchArgument(
        'rosbag_auto_shutdown', default_value='true',
        description='Shutdown after direct rosbag mapping finishes'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time},
                    {'rosbag.enable': rosbag_enable},
                    {'rosbag.path': rosbag_path},
                    {'rosbag.storage_id': rosbag_storage_id},
                    {'rosbag.auto_shutdown': rosbag_auto_shutdown}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_rosbag_enable_cmd)
    ld.add_action(declare_rosbag_path_cmd)
    ld.add_action(declare_rosbag_storage_id_cmd)
    ld.add_action(declare_rosbag_auto_shutdown_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
