"""Compatibility launch name for existing hand-teleoperation commands."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hand_teleoperation",
                executable="filter_node",
                name="filter_node",
            )
        ]
    )
