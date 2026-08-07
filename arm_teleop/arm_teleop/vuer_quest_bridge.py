"""Vuer WebXR hand-tracking and RealSense video bridge for Meta Quest."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

from arm_teleop.hand_math import matrix_to_quat
from arm_teleop.vuer_hand_tracking import (
    INDEX_TIP,
    PINKY_TIP,
    THUMB_TIP,
    WRIST,
    gripper_aperture,
    is_vuer_hand_absent,
    joint_distance,
    parse_vuer_hand,
    update_hysteresis,
    webxr_to_ros_matrices,
)


class VuerQuestBridge(Node):
    def __init__(self):
        super().__init__("vuer_quest_bridge")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8012)
        self.declare_parameter("cert_file", "")
        self.declare_parameter("key_file", "")
        self.declare_parameter("frame_id", "vuer_world")
        self.declare_parameter("hand_fps", 30)
        self.declare_parameter("hide_hand_meshes", False)
        self.declare_parameter("pinky_pinch_on_threshold_m", 0.025)
        self.declare_parameter("pinky_pinch_off_threshold_m", 0.035)
        self.declare_parameter("gripper_aperture_min_ratio", 0.15)
        self.declare_parameter("gripper_aperture_max_ratio", 1.80)
        self.declare_parameter("enable_camera_streams", True)
        self.declare_parameter("camera1_topic", "/camera1/realsense_camera/color/image_raw/compressed")
        self.declare_parameter("camera2_topic", "/camera2/realsense_camera/color/image_raw/compressed")
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_codec", "H264")
        self.declare_parameter("camera_bitrate_bps", 4_000_000)
        self.declare_parameter("panel_distance_m", 2.2)
        self.declare_parameter("panel_height_m", 0.72)
        self.declare_parameter("panel_horizontal_offset_m", 0.55)
        self.declare_parameter("panel_vertical_offset_m", 0.05)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._hand_fps = max(1, int(self.get_parameter("hand_fps").value))
        self._hide_hands = bool(self.get_parameter("hide_hand_meshes").value)
        self._pinky_on = float(self.get_parameter("pinky_pinch_on_threshold_m").value)
        self._pinky_off = float(self.get_parameter("pinky_pinch_off_threshold_m").value)
        self._aperture_min = float(self.get_parameter("gripper_aperture_min_ratio").value)
        self._aperture_max = float(self.get_parameter("gripper_aperture_max_ratio").value)
        self._pinky_state = {"left": False, "right": False}
        self._last_event_error_log = 0.0

        self._hand_publishers = {}
        for hand in ("left", "right"):
            prefix = f"/teleop_hand_tracking/{hand}"
            self._hand_publishers[hand] = {
                "wrist": self.create_publisher(PoseStamped, f"{prefix}/wrist", 10),
                "landmarks": self.create_publisher(PoseArray, f"{prefix}/landmarks", 10),
                "pinch": self.create_publisher(Bool, f"{prefix}/pinch", 10),
                "pinky_pinch": self.create_publisher(Bool, f"{prefix}/pinky_pinch", 10),
                "fist": self.create_publisher(Bool, f"{prefix}/fist", 10),
                "gripper_aperture": self.create_publisher(Float32, f"{prefix}/gripper_aperture", 10),
            }

        try:
            from vuer import Vuer
            from vuer.schemas import DefaultScene, Hands, WebRTCVideoPlane
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Vuer with WebRTC support is required. Install it with: pip install 'vuer[webrtc]==0.1.6'"
            ) from exc

        self._DefaultScene = DefaultScene
        self._Hands = Hands
        self._WebRTCVideoPlane = WebRTCVideoPlane
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        cert_file, key_file = self._tls_paths()
        self._app = Vuer(
            host=host,
            port=port,
            cert=cert_file,
            key=key_file,
            free_port=False,
            queries={"grid": False, "collapseMenu": True},
        )
        self._app.add_handler("HAND_MOVE")(self._on_hand_move)

        self._camera_streams = {}
        self._camera_last_push = {"camera1": 0.0, "camera2": 0.0}
        self._camera_period = 1.0 / max(float(self.get_parameter("camera_fps").value), 1.0)
        self._camera_size = (
            max(1, int(self.get_parameter("camera_width").value)),
            max(1, int(self.get_parameter("camera_height").value)),
        )
        if bool(self.get_parameter("enable_camera_streams").value):
            self._configure_camera_streams()

        self._app.spawn(self._serve_session)
        self._server_thread = threading.Thread(target=self._run_vuer, name="vuer-server", daemon=True)
        self._server_thread.start()

        protocol = "https" if cert_file else "http"
        socket_protocol = "wss" if cert_file else "ws"
        self.get_logger().info(
            f"Vuer Quest bridge listening on {host}:{port}. Open "
            f"{protocol}://<ubuntu-ip>:{port}/?ws={socket_protocol}://<ubuntu-ip>:{port} in Quest Browser."
        )
        if not cert_file:
            self.get_logger().warning(
                "Vuer is running without TLS. WebXR hand tracking requires HTTPS on a remote headset."
            )

    def _tls_paths(self) -> tuple[str | None, str | None]:
        cert = str(self.get_parameter("cert_file").value).strip()
        key = str(self.get_parameter("key_file").value).strip()
        if not cert and not key:
            return None, None
        if not cert or not key:
            raise RuntimeError("Both cert_file and key_file must be set when TLS is enabled.")
        missing = [path for path in (cert, key) if not Path(path).is_file()]
        if missing:
            raise RuntimeError(f"Vuer TLS file(s) not found: {', '.join(missing)}")
        return cert, key

    def _configure_camera_streams(self):
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("OpenCV is required to stream ROS camera images to Vuer.") from exc
        self._cv2 = cv2

        codec = str(self.get_parameter("camera_codec").value)
        bitrate = max(100_000, int(self.get_parameter("camera_bitrate_bps").value))
        camera_fps = max(1, int(round(float(self.get_parameter("camera_fps").value))))
        for camera in ("camera1", "camera2"):
            self._camera_streams[camera] = self._app.create_webrtc_stream(
                camera,
                codec=codec,
                max_bitrate=bitrate,
                max_framerate=camera_fps,
                resolution=self._camera_size,
            )
            topic = str(self.get_parameter(f"{camera}_topic").value)
            self.create_subscription(
                CompressedImage,
                topic,
                lambda message, name=camera: self._camera_callback(name, message),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Vuer {camera} stream subscribed to {topic}")

    def _run_vuer(self):
        try:
            self._app.start()
        except Exception as exc:  # The server runs outside the ROS executor thread.
            self.get_logger().error(f"Vuer server stopped: {exc}")

    async def _serve_session(self, session):
        session.set @ self._DefaultScene(frameloop="always")
        session.upsert @ self._Hands(
            fps=self._hand_fps,
            stream=True,
            key="quest-hands",
            hideLeft=self._hide_hands,
            hideRight=self._hide_hands,
        )

        if self._camera_streams:
            distance = float(self.get_parameter("panel_distance_m").value)
            height = float(self.get_parameter("panel_height_m").value)
            horizontal = float(self.get_parameter("panel_horizontal_offset_m").value)
            vertical = float(self.get_parameter("panel_vertical_offset_m").value)
            aspect = self._camera_size[0] / self._camera_size[1]
            session.upsert(
                [
                    self._WebRTCVideoPlane(
                        src=self._camera_streams["camera1"].url,
                        key="camera1-panel",
                        distanceToCamera=distance,
                        height=height,
                        aspect=aspect,
                        position=[-horizontal, vertical, 0.0],
                    ),
                    self._WebRTCVideoPlane(
                        src=self._camera_streams["camera2"].url,
                        key="camera2-panel",
                        distanceToCamera=distance,
                        height=height,
                        aspect=aspect,
                        position=[horizontal, vertical, 0.0],
                    ),
                ]
            )
        await session.forever()

    async def _on_hand_move(self, event, _session):
        value = event.value if isinstance(event.value, dict) else {}
        for hand in ("left", "right"):
            raw_matrices = value.get(hand)
            if raw_matrices is None or is_vuer_hand_absent(raw_matrices):
                continue
            try:
                webxr_matrices = parse_vuer_hand(raw_matrices)
                matrices = webxr_to_ros_matrices(webxr_matrices)
                state = value.get(f"{hand}State") or {}
                if not isinstance(state, dict):
                    state = {}
                self._publish_hand(hand, matrices, state)
            except (TypeError, ValueError) as exc:
                now = time.monotonic()
                if now - self._last_event_error_log > 2.0:
                    self.get_logger().warning(f"Ignored malformed Vuer {hand} hand event: {exc}")
                    self._last_event_error_log = now

    def _publish_hand(self, hand: str, matrices: np.ndarray, state: dict):
        stamp = self.get_clock().now().to_msg()
        wrist_pose = self._pose_from_matrix(matrices[WRIST])

        wrist = PoseStamped()
        wrist.header.stamp = stamp
        wrist.header.frame_id = self._frame_id
        wrist.pose = wrist_pose
        self._hand_publishers[hand]["wrist"].publish(wrist)

        landmarks = PoseArray()
        landmarks.header.stamp = stamp
        landmarks.header.frame_id = self._frame_id
        landmarks.poses = [self._pose_from_matrix(matrix) for matrix in matrices]
        self._hand_publishers[hand]["landmarks"].publish(landmarks)

        pinch_distance = joint_distance(matrices, THUMB_TIP, INDEX_TIP)
        pinch = bool(state.get("pinch", pinch_distance < 0.02))
        pinch_message = Bool()
        pinch_message.data = pinch
        self._hand_publishers[hand]["pinch"].publish(pinch_message)

        fist_message = Bool()
        fist_message.data = bool(state.get("squeeze", False))
        self._hand_publishers[hand]["fist"].publish(fist_message)

        pinky_distance = joint_distance(matrices, THUMB_TIP, PINKY_TIP)
        self._pinky_state[hand] = update_hysteresis(
            self._pinky_state[hand], pinky_distance, self._pinky_on, self._pinky_off
        )
        pinky_message = Bool()
        pinky_message.data = self._pinky_state[hand]
        self._hand_publishers[hand]["pinky_pinch"].publish(pinky_message)

        aperture, _ = gripper_aperture(matrices, self._aperture_min, self._aperture_max)
        aperture_message = Float32()
        aperture_message.data = aperture
        self._hand_publishers[hand]["gripper_aperture"].publish(aperture_message)

    @staticmethod
    def _pose_from_matrix(matrix: np.ndarray) -> Pose:
        quaternion = matrix_to_quat(matrix[:3, :3].tolist())
        pose = Pose()
        pose.position.x = float(matrix[0, 3])
        pose.position.y = float(matrix[1, 3])
        pose.position.z = float(matrix[2, 3])
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def _camera_callback(self, camera: str, message: CompressedImage):
        stream = self._camera_streams[camera]
        if not stream.ready:
            return
        now = time.monotonic()
        if now - self._camera_last_push[camera] < self._camera_period:
            return

        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = self._cv2.imdecode(encoded, self._cv2.IMREAD_COLOR)
        if frame is None:
            return
        if (frame.shape[1], frame.shape[0]) != self._camera_size:
            frame = self._cv2.resize(frame, self._camera_size, interpolation=self._cv2.INTER_AREA)
        stream.push_frame(frame)
        self._camera_last_push[camera] = now


def main(args=None):
    rclpy.init(args=args)
    node = VuerQuestBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
