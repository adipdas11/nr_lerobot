"""Meta Quest WebXR teleoperation through Vuer, with optional RealSense video."""

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

    hardware_type = LaunchConfiguration("hardware_type")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    exotica_ready_timeout = LaunchConfiguration("exotica_ready_timeout")
    enable_uf850 = LaunchConfiguration("enable_uf850")
    enable_xarm5 = LaunchConfiguration("enable_xarm5")
    uf850_hand = LaunchConfiguration("uf850_hand")
    xarm5_hand = LaunchConfiguration("xarm5_hand")
    enable_cameras = LaunchConfiguration("enable_cameras")
    enable_camera1 = LaunchConfiguration("enable_camera1")
    enable_camera2 = LaunchConfiguration("enable_camera2")
    camera1_serial = LaunchConfiguration("camera1_serial")
    camera2_serial = LaunchConfiguration("camera2_serial")
    enable_camera_streams = LaunchConfiguration("enable_camera_streams")
    enable_camera1_stream = LaunchConfiguration("enable_camera1_stream")
    enable_camera2_stream = LaunchConfiguration("enable_camera2_stream")
    enable_hand_tracking = LaunchConfiguration("enable_hand_tracking")
    teleop_config_file = LaunchConfiguration("teleop_config_file")
    vuer_config_file = LaunchConfiguration("vuer_config_file")
    vuer_host = LaunchConfiguration("vuer_host")
    vuer_port = LaunchConfiguration("vuer_port")
    vuer_cert_file = LaunchConfiguration("vuer_cert_file")
    vuer_key_file = LaunchConfiguration("vuer_key_file")
    hide_hand_meshes = LaunchConfiguration("hide_hand_meshes")

    quest_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "quest_exotica_teleop.launch.py")),
        launch_arguments={
            "hardware_type": hardware_type,
            "use_rviz": use_rviz,
            "use_sim_time": use_sim_time,
            "exotica_ready_timeout": exotica_ready_timeout,
            "enable_uf850": enable_uf850,
            "enable_xarm5": enable_xarm5,
            "uf850_hand": uf850_hand,
            "xarm5_hand": xarm5_hand,
            "enable_cameras": enable_cameras,
            "enable_camera1": enable_camera1,
            "enable_camera2": enable_camera2,
            "camera1_serial": camera1_serial,
            "camera2_serial": camera2_serial,
            "config_file": teleop_config_file,
        }.items(),
    )

    vuer_bridge = Node(
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
                "enable_camera_streams": ParameterValue(enable_camera_streams, value_type=bool),
                "enable_camera1_stream": ParameterValue(enable_camera1_stream, value_type=bool),
                "enable_camera2_stream": ParameterValue(enable_camera2_stream, value_type=bool),
                "enable_hand_tracking": ParameterValue(enable_hand_tracking, value_type=bool),
                "hide_hand_meshes": ParameterValue(hide_hand_meshes, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("exotica_ready_timeout", default_value="240.0"),
            DeclareLaunchArgument("enable_uf850", default_value="true"),
            DeclareLaunchArgument("enable_xarm5", default_value="true"),
            DeclareLaunchArgument("uf850_hand", default_value="right"),
            DeclareLaunchArgument("xarm5_hand", default_value="left"),
            DeclareLaunchArgument(
                "enable_cameras",
                default_value="true",
                description="Launch the two RealSense ROS nodes.",
            ),
            DeclareLaunchArgument("enable_camera1", default_value="true"),
            DeclareLaunchArgument("enable_camera2", default_value="true"),
            DeclareLaunchArgument("camera1_serial", default_value="_233722072289"),
            DeclareLaunchArgument("camera2_serial", default_value="_231522072957"),
            DeclareLaunchArgument(
                "enable_camera_streams",
                default_value="true",
                description="Show the two configured compressed-image topics in Quest.",
            ),
            DeclareLaunchArgument("enable_camera1_stream", default_value="true"),
            DeclareLaunchArgument("enable_camera2_stream", default_value="true"),
            DeclareLaunchArgument("enable_hand_tracking", default_value="true"),
            DeclareLaunchArgument(
                "teleop_config_file",
                default_value=str(package_share / "config" / "quest_exotica_teleop.yaml"),
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
            DeclareLaunchArgument(
                "hide_hand_meshes",
                default_value="false",
                description="Hide Vuer's built-in tracked hand meshes while retaining tracking.",
            ),
            quest_teleop,
            vuer_bridge,
        ]
    )
