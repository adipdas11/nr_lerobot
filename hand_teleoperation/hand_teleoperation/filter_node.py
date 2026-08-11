import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray
from hand_teleoperation_interfaces.msg import HandAngles

'''
One node, two pipelines, one output type.

  mediapipe_node  --HandAngles-->  on_mediapipe  -\
                                                   >-- smooth() --> HandAngles
  Quest (vuer)    --PoseArray--->  on_quest    ---/

MediaPipe sends angles it computed itself, since it owns the 21-landmark
convention. The Quest sends raw landmarks, so the 25-joint convention lives
here. The two callbacks differ only in how they get from a message to a dict of
14 angles - deadband, message building and publishing are shared.
'''

# The 14 float64 joint fields, read off the message itself so this node needs no
# edit when HandAngles.msg gains or loses a joint.
JOINT_NAMES = tuple(
    name for name, ftype in HandAngles.get_fields_and_field_types().items()
    if ftype == 'double'
)

'''
Quest / vuer pose index meaning - 25 joints, metacarpal-based:

| Index | Joint |             | Index | Joint |
| ---: | --- |                | ---: | --- |
| 0 | Wrist |                 | 13 | Middle distal |
| 1 | Thumb metacarpal |      | 14 | Middle tip |
| 2 | Thumb proximal |        | 15 | Ring metacarpal |
| 3 | Thumb distal |          | 16 | Ring proximal |
| 4 | Thumb tip |             | 17 | Ring intermediate |
| 5 | Index metacarpal |      | 18 | Ring distal |
| 6 | Index proximal |        | 19 | Ring tip |
| 7 | Index intermediate |    | 20 | Pinky metacarpal |
| 8 | Index distal |          | 21 | Pinky proximal |
| 9 | Index tip |             | 22 | Pinky intermediate |
| 10 | Middle metacarpal |    | 23 | Pinky distal |
| 11 | Middle proximal |      | 24 | Pinky tip |
| 12 | Middle intermediate |  |    |    |

This is NOT MediaPipe's numbering - the Quest has an extra metacarpal joint per
finger that MediaPipe doesn't measure, which is why mediapipe_node keeps its own
table instead of sharing this one.

Only the MCP joints are listed, because that's all HandAngles.msg carries - one
angle per finger, no pinky. The triple is (metacarpal, proximal, intermediate),
so the angle sits at the knuckle, the same joint mediapipe_node measures with
its wrist-based (0, mcp, pip) triple.
'''
QUEST_JOINTS = {
    "index_mcp": (5, 6, 7),
    "middle_mcp": (10, 11, 12),
    "ring_mcp": (15, 16, 17),
    "thumb_mcp": (1, 2, 3),
}

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
class Filter(Node):
    def __init__(self):
        super().__init__('filter')

        # noise level differs per source - monocular MediaPipe is jumpier than
        # headset tracking - so both are tunable from the launch file
        self.declare_parameter('threshold_mediapipe', 2.0)
        self.declare_parameter('threshold_quest', 1.0)
        self.threshold = {
            'mediapipe': self.get_parameter('threshold_mediapipe').value,
            'quest': self.get_parameter('threshold_quest').value,
        }

        ### MEDIAPIPE PIPELINE - angles already computed upstream ###
        self.sub_mp = self.create_subscription(
            HandAngles,
            '/landmark_angles',
            self.on_mediapipe,
            qos_profile)

        ### QUEST PIPELINE - raw landmarks, angles computed below ###
        # handedness comes from which topic it arrived on, since PoseArray has
        # no field for it
        self.sub_quest_left = self.create_subscription(
            PoseArray,
            '/teleop_hand_tracking/left/landmarks',
            self.on_quest_left,
            qos_profile)

        self.sub_quest_right = self.create_subscription(
            PoseArray,
            '/teleop_hand_tracking/right/landmarks',
            self.on_quest_right,
            qos_profile)

        self.publisher_all = self.create_publisher(
            HandAngles,
            '/hand_angles',
            qos_profile)

        # (source, handedness) -> {joint_name: last published angle}. Keyed this
        # way so the two sources - and the two hands - never get compared
        # against each other's history.
        self.prev = {}

        # a PoseArray shorter than this can't fill every joint triple
        self.quest_min_poses = max(i for tri in QUEST_JOINTS.values() for i in tri) + 1

        # catches a QUEST_JOINTS key that doesn't exist in HandAngles.msg.
        # Fatal, not a warning: otherwise the node starts fine and only dies on
        # the first Quest frame, when publish() setattr's the missing field.
        mismatch = set(QUEST_JOINTS) ^ set(JOINT_NAMES)
        if mismatch:
            raise RuntimeError(
                f"[INIT] joint names don't match HandAngles.msg: {sorted(mismatch)}")

        self.get_logger().info(
            f"[INIT] Initialising filter node - thresholds {self.threshold} deg")

    def on_mediapipe(self, msg):
        # mediapipe_node already did the geometry, so just pull the fields out
        angles = {name: getattr(msg, name) for name in JOINT_NAMES}

        smoothed = self.smooth(angles, ('mediapipe', msg.handedness))
        self.publish(smoothed, msg.header, msg.handedness)

    def on_quest_left(self, msg):
        self.on_quest(msg, 'Left')

    def on_quest_right(self, msg):
            self.on_quest(msg, 'Right')

    def on_quest(self, msg, handedness):
        if len(msg.poses) < self.quest_min_poses:
            self.get_logger().warn(
                f"[{handedness}] got {len(msg.poses)} poses, need {self.quest_min_poses}",
                throttle_duration_sec=2.0)
            return

        # positions only. The Quest's per-joint quaternions are absolute
        # orientations in vuer_world, not bend angles, so using them would give
        # a different definition of "angle" than the MediaPipe side.
        points = np.array([[p.position.x, p.position.y, p.position.z]
                           for p in msg.poses])

        angles = {}
        for name, (a_idx, b_idx, c_idx) in QUEST_JOINTS.items():
            angles[name] = self.get_joint_angles(
                points[a_idx], points[b_idx], points[c_idx])

        smoothed = self.smooth(angles, ('quest', handedness))
        self.publish(smoothed, msg.header, handedness)

    def smooth(self, angles, key):
        '''Deadband: a change smaller than the threshold is held at the previous
        value, so the servos don't twitch on tracking noise. Swap for an EMA
        when the stepping becomes visible:

            value = alpha * value + (1.0 - alpha) * last

        alpha would need tuning per source though - it's rate-dependent, and the
        two pipelines don't publish at the same rate.'''
        prev = self.prev.setdefault(key, {})
        threshold = self.threshold[key[0]]

        smoothed = {}
        for name, value in angles.items():
            last = prev.get(name)
            if last is not None and abs(value - last) < threshold:
                value = last
            prev[name] = value
            smoothed[name] = value

        return smoothed

    def publish(self, angles, header, handedness):
        msg = HandAngles()
        msg.header = header   # the SOURCE stamp - restamping here would hide the
                              # real latency from the delay measurements
        msg.handedness = handedness

        for name, value in angles.items():
            setattr(msg, name, float(value))

        self.publisher_all.publish(msg)

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

        return round(angle_deg, 0)


def main(args=None):
    rclpy.init(args=args)
    node = Filter()

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
