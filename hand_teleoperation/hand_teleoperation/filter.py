import os
import rclpy
import numpy as np
from rclpy.qos import *
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
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
    "index_pip": (5, 6, 7),
    "index_dip": (6, 7, 8),

    # middle finger
    "middle_mcp": (0, 9, 10),
    "middle_pip": (9, 10, 11),
    "middle_dip": (10, 11, 12),

    # ring finger
    "ring_mcp": (0, 13, 14),
    "ring_pip": (13, 14, 15),
    "ring_dip": (14, 15, 16),

    #pinky
    "pinky_mcp": (0, 17, 18),
    "pinky_pip": (17, 18, 19),
    "pinky_dip": (18, 19, 20),

    # thumb
    "thumb_mcp": (1, 2, 3),
    "thumb_ip": (2, 3, 4),
}

ANGLE_OFFSETS = {
    "index_mcp": -10,
    "index_pip": +10
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

class Filter(Node):
    def __init__(self):
        super().__init__('filtered_angles')

        self.subscription_mp = self.create_subscription(
            HandAngles,
            '/landmark_angles',
            self.angle_filter_mp,
            qos_profile)

        self.subscription_oc_right = self.create_subscription(
            PoseArray,
            '/teleop_hand_tracking/right/landmarks',
            self.angle_filter_oc,
            qos_profile)

        # TODO: The publisher could be in form of a PoseArray to make it easier to relate the landmark to the angle.
        # ^^ So the information is linked to eachother and not create another package.
        self.publisher_oc = self.create_publisher(
            PoseArray,
            '/filtered_angles_quest',
            qos_profile)

        self.publisher = self.create_publisher(
            HandAngles,
            '/filtered_angles_quest_mp',
            qos_profile)

        self.create_timer(1/30.0, self.publish_latest)
        self.get_logger().info("[INIT] Initialising filter node")

        self.latest_msg = None
        self.threshold = 2
        self.prev_angles = {}

    def angle_filter_mp(self, msg):
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

        for name, value in angles.items():
            prev = self.prev_angles.get(name)
            if prev is not None and abs(value-prev) < self.threshold:
                value = prev
            self.prev_angles[name] = value
            value += ANGLE_OFFSETS.get(name, 0)     # .get(name, 0) → 0 if joint isn't in the table
            setattr(msg, name, value) # setattr: allows you to set the value of an attribute of an object dynamically at runtime using the attribute's name as a string

    def angle_filter_oc(self, msg):

        '''
        | Index | Joint |
        | ---: | --- |
        | 0 | Wrist |
        | 1 | Thumb metacarpal |
        | 2 | Thumb proximal |
        | 3 | Thumb distal |
        | 4 | Thumb tip |
        | 5 | Index metacarpal |
        | 6 | Index proximal |
        | 7 | Index intermediate |
        | 8 | Index distal |
        | 9 | Index tip |
        | 10 | Middle metacarpal |
        | 11 | Middle proximal |
        | 12 | Middle intermediate |
        | 13 | Middle distal |
        | 14 | Middle tip |
        | 15 | Ring metacarpal |
        | 16 | Ring proximal |
        | 17 | Ring intermediate |
        | 18 | Ring distal |
        | 19 | Ring tip |
        | 20 | Pinky metacarpal |
        | 21 | Pinky proximal |
        | 22 | Pinky intermediate |
        | 23 | Pinky distal |
        | 24 | Pinky tip |
        '''

        angles_array = msg.poses

        # self.publisher_oc.publish(angles_array[])


    def publish_latest(self):

        if self.latest_msg is None:    # nothing detected yet
            return
        self.latest_msg.header.stamp = self.get_clock().now().to_msg()  # optional: fresh timestamp
        self.publisher.publish(self.latest_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Filter()

    try:
        rclpy.spin(node)

    except Exception as e:
        node.get_logger().error(f"{e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# =============================================================================
# NEW VERSION - replaces everything above once you're happy with it.
#
# Filter does one job: smooth HandAngles. Angle extraction stays in the source
# nodes (mediapipe_node for the 21-landmark convention, quest_node for the 25).
# One instance per source, topics set by remapping in the launch file.
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from hand_teleoperation_interfaces.msg import HandAngles

# The 14 float64 joint fields, read off the message itself so this node needs no
# edit when HandAngles.msg gains or loses a joint.
JOINT_NAMES = tuple(
    name for name, ftype in HandAngles.get_fields_and_field_types().items()
    if ftype == 'double'
)

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Filter(Node):
    def __init__(self):
        super().__init__('filter')

        # tunable per instance from the launch file - MediaPipe and the Quest
        # will not be equally noisy
        self.declare_parameter('threshold', 2.0)
        self.threshold = self.get_parameter('threshold').value

        self.subscription = self.create_subscription(
            HandAngles, 'hand_angles', self.on_angles, qos_profile)

        self.publisher = self.create_publisher(
            HandAngles, 'filtered_angles', qos_profile)

        self.prev = {}   # handedness -> {joint_name: last published angle}
        self.get_logger().info(f"[INIT] filter up, threshold={self.threshold} deg")

    def on_angles(self, msg):
        prev = self.prev.setdefault(msg.handedness, {})

        out = HandAngles()
        out.header = msg.header          # keep the SOURCE stamp, do not restamp
        out.handedness = msg.handedness

        for name in JOINT_NAMES:
            value = getattr(msg, name)
            last = prev.get(name)

            # deadband: below the threshold, hold the last value so the servos
            # don't twitch. Swap for an EMA when the 2 deg stepping shows up:
            #     value = self.alpha * value + (1.0 - self.alpha) * last
            if last is not None and abs(value - last) < self.threshold:
                value = last

            prev[name] = value
            setattr(out, name, value)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Filter()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"{e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
