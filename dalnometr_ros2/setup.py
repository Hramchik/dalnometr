from setuptools import find_packages, setup
import os
from glob import glob

package_name = "dalnometr_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "smbus2"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS2 Humble node for DYP-A22 ultrasonic sensors over I2C",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dyp_a22_node = dalnometr_ros2.dyp_a22_node:main",
        ],
    },
)
