from glob import glob

from setuptools import find_packages, setup


package_name = "pct_global_planner"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jhr",
    maintainer_email="jhr@example.com",
    description="Global planner adapter that publishes PCT Planner paths for EGO-Planner.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pct_global_planner_node = pct_global_planner.pct_global_planner_node:main",
            "pct_global_map_publisher_node = pct_global_planner.map_publisher_node:main",
            "pct_goal_marker_node = pct_global_planner.goal_marker_node:main",
        ],
    },
)
