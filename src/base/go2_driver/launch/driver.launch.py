from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
def generate_launch_description():
    go2_desc_pkg = get_package_share_directory("go2_description")
    go2_driver_pkg = get_package_share_directory("go2_driver")
    rviz_path = os.path.join(go2_driver_pkg, "rviz")

    use_rviz = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true"
    )
    return LaunchDescription([
        use_rviz,
        #机器人可视化
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(go2_desc_pkg, "launch", "display.launch.py")
            ),
            launch_arguments=[("use_joint_state_publisher", "false")]#关节状态由driver发布,不再由display发布
        ),
        #rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(rviz_path, "display.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "radar", "--child-frame-id", "utlidar_lidar"]
        ),
        #里程计
        Node(
            package="go2_driver",
            executable="driver",
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base',
                'publish_tf': True
            }]
        )
    ])