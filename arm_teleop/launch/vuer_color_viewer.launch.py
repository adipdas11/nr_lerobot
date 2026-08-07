"""Show one or two RealSense RGB streams in Vuer without robot or hand tracking."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("arm_teleop"))

    camera_serial = LaunchConfiguration("camera_serial")
    camera_topic = LaunchConfiguration("camera_topic")
    enable_camera2 = LaunchConfiguration("enable_camera2")
    camera2_serial = LaunchConfiguration("camera2_serial")
    camera2_topic = LaunchConfiguration("camera2_topic")
    vuer_config_file = LaunchConfiguration("vuer_config_file")
    vuer_host = LaunchConfiguration("vuer_host")
    vuer_port = LaunchConfiguration("vuer_port")
    vuer_cert_file = LaunchConfiguration("vuer_cert_file")
    vuer_key_file = LaunchConfiguration("vuer_key_file")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "realsense.launch.py")),
        launch_arguments={
            "enable_camera1": "true",
            "enable_camera2": enable_camera2,
            "camera1_serial": camera_serial,
            "camera2_serial": camera2_serial,
            "enable_depth": "false",
            "pointcloud_enable": "false",
            "align_depth": "false",
        }.items(),
    )

    bridge = Node(
        package="arm_teleop",
        executable="vuer_quest_bridge",
        name="vuer_quest_bridge",
        output="screen",
        parameters=[
            vuer_config_file,
            {
                "host": vuer_host,
                "port": ParameterValue(vuer_port, value_type=int),
                "cert_file": vuer_cert_file,
                "key_file": vuer_key_file,
                "enable_hand_tracking": False,
                "enable_camera_streams": True,
                "enable_camera1_stream": True,
                "enable_camera2_stream": ParameterValue(enable_camera2, value_type=bool),
                "camera1_topic": camera_topic,
                "camera2_topic": camera2_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_serial", default_value="_233722072289"),
            DeclareLaunchArgument("enable_camera2", default_value="false"),
            DeclareLaunchArgument("camera2_serial", default_value="_231522072957"),
            DeclareLaunchArgument(
                "camera_topic",
                default_value="/camera1/realsense_camera/color/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "camera2_topic",
                default_value="/camera2/realsense_camera/color/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "vuer_config_file",
                default_value=str(package_share / "config" / "vuer_quest_bridge.yaml"),
            ),
            DeclareLaunchArgument("vuer_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("vuer_port", default_value="8012"),
            DeclareLaunchArgument(
                "vuer_cert_file",
                default_value=EnvironmentVariable("VUER_CERT_FILE", default_value=""),
            ),
            DeclareLaunchArgument(
                "vuer_key_file",
                default_value=EnvironmentVariable("VUER_KEY_FILE", default_value=""),
            ),
            camera,
            bridge,
        ]
    )
