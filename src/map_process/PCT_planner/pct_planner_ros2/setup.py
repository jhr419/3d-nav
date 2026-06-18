from glob import glob

from setuptools import find_packages, setup


package_name = 'pct_planner_ros2'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/pcd', glob('pcd/*')),
        ('share/' + package_name + '/tomogram', glob('tomogram/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PCT Planner Maintainers',
    maintainer_email='byangar@connect.ust.hk',
    description='ROS 2 wrapper nodes for PCT Planner tomography and trajectory planning.',
    license='GPL-2.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pct_map_publisher = pct_planner_ros2.map_publisher_node:main',
            'pct_tomography = pct_planner_ros2.tomography_node:main',
            'pct_plan = pct_planner_ros2.planner_node:main',
            'pct_start_goal_marker = pct_planner_ros2.start_goal_marker_node:main',
        ],
    },
)
