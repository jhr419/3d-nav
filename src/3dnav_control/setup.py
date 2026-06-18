from glob import glob

from setuptools import find_packages, setup


package_name = "nav3d_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jhr",
    maintainer_email="jhr@example.com",
    description="Navigation execution controller and velocity gate for the 3D navigation stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nav_execution_controller_node = nav3d_control.nav_execution_controller_node:main",
            "cmd_vel_gate_node = nav3d_control.cmd_vel_gate_node:main",
        ],
    },
)
