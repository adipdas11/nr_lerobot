import os
import cv2
import time
import rclpy
import numpy as np
import mediapipe as mp
from rclpy.qos import *
from rclpy.node import Node
from cv_bridge import CvBridge
from mediapipe import solutions
from sensor_msgs.msg import Image
from mediapipe.framework.formats import landmark_pb2
from hand_teleoperation_interfaces.msg import HandAngles
from ament_index_python.packages import get_package_share_directory
'''
The "proper" long-term option is a custom message
(e.g. a HandAngles.msg with named fields like thumb, index, ...),
which makes the subscriber code far more readable than indexing into
data[3]. But Float64MultiArray is the fast way to get moving, and you can upgrade later.
'''

# TODO: Look at this chat and follow the debugging processes: https://claude.ai/chat/7268c26f-f078-432f-9d5a-69b04086c3c6


FINGER_JOINTS = {
    # index finger
    "index_mcp": (0, 5, 6),
    # "index_pip": (5, 6, 7),
    # "index_dip": (6, 7, 8),

    # middle finger
    "middle_mcp": (0, 9, 10),
    # "middle_pip": (9, 10, 11),
    # "middle_dip": (10, 11, 12),

    # ring finger
    "ring_mcp": (0, 13, 14),
    # "ring_pip": (13, 14, 15),
    # "ring_dip": (14, 15, 16),

    # thumb
    "thumb_mcp": (1, 2, 3),
    # "thumb_ip": (2, 3, 4),
}

ANGLE_OFFSETS = {
    "index_mcp": -10
}

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

"""
THIS IS THE CURRENT QoS PROFILE OF THE CAMERA TOPIC:

QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
"""

class LandmarkDetector(Node):
    def __init__(self):
        super().__init__('landmark_angles')

        ### SUBS AND PUBS ###
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.img_callback,
            qos_profile)

        self.publisher = self.create_publisher(
            HandAngles,
            'landmark_angles',
            qos_profile)

        self.create_timer(1/30.0, self.publish_latest)
        self.get_logger().info("[INIT] Initialising mediapipe node")
        self.bridge = CvBridge()


        ### MEDIAPIPE OPTIONS + CONFIGURATIONS ###
        self.BaseOptions = mp.tasks.BaseOptions
        self.HandLandmarker = mp.tasks.vision.HandLandmarker
        self.HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        self.VisionRunningMode = mp.tasks.vision.RunningMode
        self.HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult

        self.model_path = os.path.join(
            get_package_share_directory('hand_teleoperation'), 'data', 'hand_landmarker.task')

        self.hand = []
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_solutions_hands = mp.solutions.hands
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands

        self.options = self.HandLandmarkerOptions(
            num_hands = 1,
            min_hand_detection_confidence=0.5, # The lower this number, the slower mediapipe will be.
            min_tracking_confidence=0.5, # The lower this number, the slower mediapipe will be.
            base_options=self.BaseOptions(model_asset_path=self.model_path), # Selecting the model
            running_mode=self.VisionRunningMode.LIVE_STREAM, # Configure the gesture recognizer for live stream mode.
            result_callback=self.landmark_result) # When a result is ready, MediaPipe will automatically call print_result.

        self.landmarker = self.HandLandmarker.create_from_options(self.options)

        self.frame_count = 0
        self.hwl = None   # latest HandLandmarkerResult; set by landmark_result()
        self.latest_msg = None
        self.threshold = 2
        self.prev_angles = {}

    def img_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8') # converting the image from a ROS2 format to an array format that cv2 can process it
            self.mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=self.current_frame) # change the image into the correct format for MediaPipe to read it

            self.frame_count += 1
            self.get_logger().info(f"Frame count: {self.frame_count}", throttle_duration_sec = 1.0)

            # TODO: Maybe processing the frames first and then displaying the old ones or vice versa
            self.landmarker.detect_async(self.mp_image, self.frame_count)

            cv2.imshow("landmarks", cv2.cvtColor(self.draw_landmarks(), cv2.COLOR_RGB2BGR)) # TODO: Maybe the fact that you're showing it adds a lot of delay.
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"{e}")

    # Create a gesture recognizer instance with the live stream mode
    def landmark_result(self, result, mp_image, timestamp_ms):
        try:
            # angles
            # self.get_logger().info("is this working", throttle_duration_sec = 1.0)
            hwl = result.hand_world_landmarks
            if not hwl:
                return

            # self.get_logger().info(f"Checking for data: {len(hwl)}", throttle_duration_sec = 1.0) # making sure hwl is outputting data

            self.hwl = result   # draw_landmarks() reads self.hwl.hand_landmarks / .handedness

            for i in range(len(hwl)): # one per hand
                landmarks = hwl[i] # the 21 world landmarks for this hand

                angles = {}
                for name, (a_idx, b_idx, c_idx) in FINGER_JOINTS.items():   # one per joint
                    a = np.array([landmarks[a_idx].x, landmarks[a_idx].y, landmarks[a_idx].z])
                    b = np.array([landmarks[b_idx].x, landmarks[b_idx].y, landmarks[b_idx].z])
                    c = np.array([landmarks[c_idx].x, landmarks[c_idx].y, landmarks[c_idx].z])
                    angles[name] = self.get_joint_angles(a, b, c)

                msg = HandAngles()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.handedness = result.handedness[i][0].category_name

                # checking if the value is within a certain threshold to reduce jitter
                for name, value in angles.items():
                    setattr(msg, name, value + ANGLE_OFFSETS.get(name, 0)) # setattr: allows you to set the value of an attribute of an object dynamically at runtime using the attribute's name as a string
                self.latest_msg = msg # store instead of publish

        except Exception as e:
            self.get_logger().error(f"{e}")

    def publish_latest(self):

        if self.latest_msg is None:    # nothing detected yet
            return
        self.latest_msg.header.stamp = self.get_clock().now().to_msg()  # optional: fresh timestamp
        self.publisher.publish(self.latest_msg)

    def get_joint_angles(self, a, b, c):
        # get the vectors between the points
        v1 = b-a
        v2 = c-b

        # calculate the dot product
        dot_product = np.dot(v1, v2) # v1.v2 = (v1[0] x v2[0]) + (v1[1] x v2[1]) + (v1[2] x v2[2])

        #calcualte magnitude
        magnitude_v1 = np.linalg.norm(v1)
        magnitude_v2 = np.linalg.norm(v2)

        #calculate the angle of the vectors
        cos_angle = np.clip(dot_product/(magnitude_v1*magnitude_v2), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)

        #convert to degrees
        angle_deg = np.degrees(angle_rad)

        return round(angle_deg, 0) # Add or take away degrees from here when calibrating

    def draw_landmarks(self):
        MARGIN = 10
        FONT_SIZE = 1
        FONT_THICKNESS = 1
        HANDEDNESS_TEXT_COLOUR = (88, 205, 54)

        annotated_img = np.copy(self.current_frame)

        # The detection callback runs asynchronously, so self.hwl may not be
        # set yet (e.g. first frames) or may hold no hands. Return the raw frame.
        if self.hwl is None:
            return annotated_img

        hand_landmarks_list = self.hwl.hand_landmarks
        handedness_list = self.hwl.handedness


        for i in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[i]
            handedness = handedness_list[i]

            # convert raw Tasks-API landmarks -> NormalizedLandmarkList proto
            proto = landmark_pb2.NormalizedLandmarkList()
            proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
                for lm in hand_landmarks
            ])

            # indented into the loop, pass proto, connections (plural)
            self.mp_drawing.draw_landmarks(
                annotated_img,
                proto,
                self.mp_solutions_hands.HAND_CONNECTIONS,
                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                self.mp_drawing_styles.get_default_hand_connections_style())

            height, width, _ = annotated_img.shape

            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]

            text_x = max(0, int(min(x_coordinates)*width) - MARGIN)
            text_y = max(FONT_SIZE*20 + MARGIN, int(min(y_coordinates)*height) - MARGIN)

            cv2.putText(annotated_img, f"{handedness[0].category_name}",
                        (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                        FONT_SIZE, HANDEDNESS_TEXT_COLOUR, FONT_THICKNESS, cv2.LINE_AA)

        return annotated_img

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(f"{e}")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
