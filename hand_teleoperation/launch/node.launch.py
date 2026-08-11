import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # camera driver -> publishes /camera/image_raw
        # Node(
        #     package='camera_ros',
        #     executable='camera_node',
        #     name='camera'
        # ),

        # Node(
        #     package='hand_teleoperation',
        #     executable='mediapipe_node',
        #     name='mediapipe_node'
        # ),

        # Node(
        #     package='hand_teleoperation',
        #     executable='filter',
        #     name='filter'
        # ),

        Node(
            package='hand_teleoperation',
            executable='filter_node',
            name='filter_node'
        )

        # maybe add a delay node that measures delay in mediapipe frames
    ])
