"""
Launch file for the DYP-A22 ultrasonic sensor node.

Override parameters from the command line, e.g.:
  ros2 launch dalnometr_ros2 dyp_a22.launch.py i2c_bus:=1 addresses:=[0x57,0x58]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("addresses", default_value="[116]"),  # 0x74
            DeclareLaunchArgument("scan_all_7bit", default_value="false"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("frame_prefix", default_value="ultrasonic"),
            Node(
                package="dalnometr_ros2",
                executable="dyp_a22_node",
                name="dyp_a22",
                namespace="sensors",
                output="screen",
                parameters=[
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "addresses": LaunchConfiguration("addresses"),
                        "scan_all_7bit": LaunchConfiguration("scan_all_7bit"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "frame_prefix": LaunchConfiguration("frame_prefix"),
                    }
                ],
            ),
        ]
    )
