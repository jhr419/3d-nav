from setuptools import find_packages, setup


package_name = "nav3d_diagnostics"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="jhr",
    maintainer_email="jhr@example.com",
    description="System self-check and runtime diagnostics for the 3D navigation stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nav3d_system_diagnostics_node = nav3d_diagnostics.nav3d_system_diagnostics_node:main",
        ],
    },
)
