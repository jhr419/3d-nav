from glob import glob

from setuptools import find_packages, setup


package_name = 'pcd_preprocessor_ros2'


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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PCT Planner Maintainers',
    maintainer_email='jhr@example.com',
    description='ROS 2 PCD preprocessing node for PCT Planner tomography maps.',
    license='GPL-2.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_preprocessor_node = pcd_preprocessor_ros2.pcd_preprocessor_node:main',
        ],
    },
)
