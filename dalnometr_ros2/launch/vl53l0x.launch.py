"""
Launch file for the VL53L0X ToF sensor node.

Override parameters from the command line, e.g.:
  ros2 launch dalnometr_ros2 vl53l0x.launch.py i2c_bus:=1 addresses:=[0x29,0x30]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("i2c_bus", default_value="1"),
            DeclareLaunchArgument("addresses", default_value="[41]"),  # 0x29
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("frame_prefix", default_value="tof"),
            Node(
                package="dalnometr_ros2",
                executable="vl53l0x_node",
                name="vl53l0x",
                namespace="sensors",
                output="screen",
                parameters=[
                    {
                        "i2c_bus": LaunchConfiguration("i2c_bus"),
                        "addresses": LaunchConfiguration("addresses"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "frame_prefix": LaunchConfiguration("frame_prefix"),
                    }
                ],
            ),
        ]
    )
