# Porting the Meta Quest + Vuer teleoperation setup to ABB YuMi

This guide explains how the working Meta Quest integration in this repository
was built and how to reproduce it in a ROS 2 workspace for an ABB YuMi. The
Quest/Vuer bridge is robot-independent. The part that must be replaced for
YuMi is the node that converts hand motion into robot targets and sends them
through the YuMi driver/controller.

Do not copy this repository's entire `arm_teleop` package into the YuMi
workspace. That package also depends on the UF850/xArm descriptions, EXOTica,
and this workcell's controller configuration. Create a small standalone Vuer
bridge package, then connect its stable ROS interface to the existing YuMi
stack.

## 1. Resulting architecture

```text
                         HAND DATA

Meta Quest Browser
  Vuer WebXR Hands
       |
       | HTTPS page + WSS HAND_MOVE events
       | 25 transforms for each tracked hand
       v
vuer_quest_bridge (Ubuntu / ROS 2)
       |
       +--> /teleop_hand_tracking/left/wrist
       +--> /teleop_hand_tracking/right/wrist
       +--> /teleop_hand_tracking/{left,right}/landmarks
       +--> pinch, fist, pinky_pinch and gripper_aperture
       |
       v
yumi_vuer_teleop adapter
       |
       +--> relative calibration
       +--> workspace and velocity limits
       +--> filtering and tracking watchdog
       +--> IK / MoveIt / YuMi controller commands
       v
ABB YuMi driver and controllers


                         CAMERA DATA

ROS camera drivers
       |
       | sensor_msgs/msg/CompressedImage
       v
vuer_quest_bridge
       |
       | JPEG decode -> WebRTC H.264
       v
Two Vuer video panels in the Quest view
```

The two paths are deliberately independent. Hand tracking can be tested with
the robot and cameras disabled. Camera streaming can also be tested without
allowing any robot command.

## 2. What was implemented in this repository

The implementation was built in layers:

1. Vuer serves a secure browser page directly from Ubuntu on port `8012`.
2. Vuer's `Hands` scene component requests Quest WebXR hand tracking and emits
   `HAND_MOVE` events.
3. The bridge decodes 25 WebXR joint matrices for the left and right hands.
4. WebXR coordinates are converted to a ROS-friendly coordinate convention.
5. The bridge publishes a robot-independent ROS topic contract.
6. The existing teleoperation node consumes wrist motion and gesture topics.
7. Two ROS compressed-image topics are decoded and sent back to the Quest as
   independent WebRTC/H.264 panels.
8. TLS, Docker host networking, camera device access, launch composition, and
   tests make the setup reproducible.

The important design choice was to keep the Vuer bridge unaware of robot
joint names, kinematics, MoveIt groups, or controller topics. That is what
makes the same bridge usable with YuMi.

## 3. Reusable files

Use these files as the reference implementation:

| Current file | Purpose in a YuMi workspace |
| --- | --- |
| `arm_teleop/arm_teleop/vuer_quest_bridge.py` | Vuer server, ROS publishers, gestures, and WebRTC camera streams |
| `arm_teleop/arm_teleop/vuer_hand_tracking.py` | Packed payload parsing, WebXR-to-ROS conversion, and gesture geometry |
| `arm_teleop/arm_teleop/hand_math.py` | The bridge currently imports `matrix_to_quat`; copy that helper or provide an equivalent |
| `arm_teleop/config/vuer_quest_bridge.yaml` | Hand, camera, bitrate, and panel parameters |
| `arm_teleop/test/test_vuer_hand_tracking.py` | Pure unit tests for payload and coordinate conversion |
| `setup_vuer_tls.sh` | Creates a local certificate containing the Ubuntu Wi-Fi IP |
| `arm_teleop/launch/vuer_quest_teleop.launch.py` | Example of composing robot bring-up, cameras, and the Vuer node |
| `arm_teleop/launch/realsense.launch.py` | Two RealSense nodes and compressed-image republishers |
| `Dockerfile` and `docker-compose.yml` | Reproducible Python dependencies, host networking, TLS paths, and `/dev` access |

The full landmark index order and message example are in
`VUER_HAND_TOPICS.md`.

## 4. Create a standalone ROS 2 package in the YuMi workspace

From the YuMi workspace source directory:

```bash
cd <yumi-workspace>/src
ros2 pkg create vuer_quest_bridge \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs
```

Create this package layout:

```text
vuer_quest_bridge/
├── config/
│   └── vuer_quest_bridge.yaml
├── launch/
│   ├── vuer_bridge.launch.py
│   └── yumi_vuer_teleop.launch.py
├── resource/
│   └── vuer_quest_bridge
├── test/
│   └── test_vuer_hand_tracking.py
├── vuer_quest_bridge/
│   ├── __init__.py
│   ├── hand_math.py
│   ├── vuer_hand_tracking.py
│   └── vuer_quest_bridge.py
├── package.xml
├── setup.cfg
└── setup.py
```

Copy the reusable Python, YAML, and test files listed above. Change Python
imports from `arm_teleop...` to `vuer_quest_bridge...`.

The package needs these ROS dependencies:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
```

If the package also launches camera nodes, add the camera package and
`image_transport` as execution dependencies. Do not add Vuer as a rosdep
dependency; install its Python package in the container or ROS Python
environment.

Add the bridge executable in `setup.py`:

```python
entry_points={
    "console_scripts": [
        "vuer_quest_bridge = vuer_quest_bridge.vuer_quest_bridge:main",
    ],
},
```

Install launch and config files through `data_files`, following the existing
`arm_teleop/setup.py` example.

## 5. Install the Vuer runtime reproducibly

The tested ROS Humble image in this repository installs:

```dockerfile
RUN uv pip install --system --no-cache \
    "numpy<2" \
    opencv-python \
    "vuer[webrtc]==0.1.6"
```

Use the ROS distribution and Python version required by the YuMi workspace.
For ROS Humble, the bridge must run with the ROS Python 3.10 environment; a
separate LeRobot Python 3.12 environment cannot import Humble's binary
`rclpy`.

Keep the working Vuer version pinned while porting. Upgrade only after the
hand payload tests and Quest camera tests pass again.

For Docker Compose, use host networking:

```yaml
services:
  yumi_teleop:
    network_mode: host
    privileged: true
    environment:
      VUER_CERT_FILE: /ws/.vuer/tls/cert.pem
      VUER_KEY_FILE: /ws/.vuer/tls/key.pem
    volumes:
      - /dev:/dev
      - <yumi-workspace>:/ws:rw
```

Host networking lets the Quest reach the Vuer HTTPS/WSS endpoint, permits
WebRTC to advertise usable LAN candidates, and preserves normal ROS access to
the robot controller. The `/dev` mount is needed only when cameras are opened
inside the container. A more restrictive device mapping can replace it after
the required USB/video devices are known.

## 6. Configure HTTPS for Quest WebXR

Quest and Ubuntu must be connected to the same Wi-Fi network and must be able
to communicate directly. Some guest Wi-Fi networks enable client isolation;
being connected to the same access point is not sufficient if the clients
cannot ping or open each other's ports.

Give Ubuntu a stable Wi-Fi address or DHCP reservation. Copy
`setup_vuer_tls.sh` to the YuMi repository and adjust `WORKSPACE_ROOT` if its
directory depth differs. Then run:

```bash
./setup_vuer_tls.sh <ubuntu-wifi-ip>
```

The certificate's Subject Alternative Name must contain the exact IP used by
the Quest. Keep the generated private key outside Git.

Run the bridge and open this exact form of URL in Quest Browser:

```text
https://<ubuntu-wifi-ip>:8012/?ws=wss://<ubuntu-wifi-ip>:8012
```

Accept the local certificate warning, reload the full URL, enter immersive
VR, and grant hand-tracking permission. Select **Passthrough** in Vuer when the
real workcell should remain visible. Opening the page without entering VR does
not produce Quest `HAND_MOVE` events.

## 7. How the hand payload is converted

Vuer sends 25 transforms per tracked hand. A packed frame contains
`25 * 16 = 400` little-endian `float32` values, or 1600 bytes. WebGL matrices
are serialized column-major, so each 4-by-4 block is transposed after it is
reshaped.

The bridge also ignores Vuer's small binary sentinel when a hand is not
currently tracked. A missing hand must result in no new pose messages; it must
not reuse the last pose indefinitely.

The coordinate conventions are:

```text
WebXR / Three.js: +X right, +Y up, -Z forward
Bridge ROS frame: +X forward, +Y left, +Z up
```

The basis-change matrix is:

```text
R = [ 0  0 -1 ]
    [-1  0  0 ]
    [ 0  1  0 ]
```

For each joint transform, the bridge applies:

```text
rotation_ros    = R * rotation_webxr * transpose(R)
translation_ros = R * translation_webxr
```

The output frame is named `vuer_world`. It is not automatically registered in
the YuMi TF tree. The current robot adapter uses relative motion calibration,
which avoids treating the Quest's session origin as a surveyed workcell frame.

## 8. ROS interface that should remain unchanged

For each hand, the bridge publishes:

| Topic | Type | Use |
| --- | --- | --- |
| `/teleop_hand_tracking/<hand>/wrist` | `geometry_msgs/msg/PoseStamped` | End-effector motion input |
| `/teleop_hand_tracking/<hand>/landmarks` | `geometry_msgs/msg/PoseArray` | All 25 WebXR joints |
| `/teleop_hand_tracking/<hand>/pinch` | `std_msgs/msg/Bool` | Thumb-index gesture |
| `/teleop_hand_tracking/<hand>/pinky_pinch` | `std_msgs/msg/Bool` | Thumb-pinky gesture with hysteresis |
| `/teleop_hand_tracking/<hand>/fist` | `std_msgs/msg/Bool` | Quest/Vuer squeeze state |
| `/teleop_hand_tracking/<hand>/gripper_aperture` | `std_msgs/msg/Float32` | Normalized hand opening from 0 to 1 |

Replace `<hand>` with `left` or `right`.

The normalized gripper aperture uses thumb-to-index distance divided by palm
width, so it is less sensitive to operator hand size. For a YuMi gripper,
apply filtering, hysteresis, and a bounded mapping from `0..1` to the driver's
allowed gripper width or joint range.

If finger joint angles are required, calculate them from three landmark
positions. For an angle at point `B` using points `A-B-C`:

```text
u = A - B
v = C - B
angle = acos(clamp(dot(u, v) / (norm(u) * norm(v)), -1, 1))
```

Reject the calculation when either vector is nearly zero, then filter the
angle before sending it to any actuator.

## 9. Replace the robot-specific adapter for YuMi

The current `exotica_arm_teleop.py` is not portable because it contains the
UF850/xArm joint names, MoveIt groups, home poses, controller backends, and
workspace limits. For YuMi, create `yumi_vuer_teleop.py` or adapt the YuMi
teleoperation node already present in that workspace.

The adapter needs four capabilities from the YuMi stack:

1. Read current joint states and the current TCP pose for each arm.
2. Solve a bounded TCP target through the configured YuMi kinematics/planning
   interface.
3. Send joint or trajectory targets through the YuMi controller interface.
4. Command the left and right grippers within their documented limits.

The exact actions/topics depend on the YuMi driver and controller
configuration already used in that workspace. Keep those details out of the
Vuer bridge.

Map hands explicitly:

```yaml
yumi_vuer_teleop:
  ros__parameters:
    right_arm.hand: right
    left_arm.hand: left
```

Do not assume the mapping from the topic name alone; make it a launch/config
parameter so it can be swapped without changing code.

### Relative calibration

When an arm is explicitly enabled and tracking has been stable, record:

```text
hand_origin = current Quest wrist pose
tcp_origin  = current YuMi TCP pose
```

For every later frame:

```text
hand_delta = hand_position_now - hand_origin
target_position = tcp_origin + R_vuer_to_yumi * scale * hand_delta
```

Use a calibrated 3-by-3 `R_vuer_to_yumi` and separate X/Y/Z scales. Then clamp
the target to a conservative YuMi workspace before IK.

For orientation:

```text
hand_delta_q = hand_q_now * inverse(hand_q_origin)
target_q = hand_delta_q * tcp_origin_q
```

YuMi has seven joints per arm, but full wrist orientation should still be
enabled only after its IK, redundancy resolution, collision checking, and
joint-limit behavior are validated in fake hardware.

### Required watchdog behavior

The adapter must stop generating new robot targets when:

- the wrist topic is stale;
- the hand disappears;
- IK fails or returns a joint-limit violation;
- the Quest or Vuer websocket disconnects;
- the explicit enable/deadman state is false;
- the robot driver or controller reports a fault.

On loss of tracking, hold or stop through the supported YuMi controller API.
Do not continue integrating the last hand velocity.

## 10. Compose a YuMi launch file

Create `yumi_vuer_teleop.launch.py` with four independently switchable parts:

```text
1. YuMi bring-up: fake or real
2. yumi_vuer_teleop adapter
3. camera drivers / compressed-image publishers
4. vuer_quest_bridge
```

The structure should follow the current
`arm_teleop/launch/vuer_quest_teleop.launch.py`, but replace its include of
`quest_exotica_teleop.launch.py` with the YuMi bring-up and adapter.

Expose at least these launch arguments:

```text
hardware_type:=fake|real
use_rviz:=true|false
enable_robot_commands:=false|true
enable_cameras:=false|true
enable_camera_streams:=false|true
vuer_host:=0.0.0.0
vuer_port:=8012
vuer_cert_file:=...
vuer_key_file:=...
hide_hand_meshes:=false|true
```

The default must be fake hardware with robot commands disabled.

## 11. Connect cameras to the Quest view

The bridge is not tied to RealSense. It only requires two
`sensor_msgs/msg/CompressedImage` topics. If the YuMi workspace uses different
cameras, publish or republish their color streams as compressed images:

```bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera_left/color/image_raw \
  -r out/compressed:=/camera_left/color/image_raw/compressed
```

Set the two topics in `vuer_quest_bridge.yaml`:

```yaml
camera1_topic: /camera_left/color/image_raw/compressed
camera2_topic: /camera_right/color/image_raw/compressed
```

The bridge decodes each JPEG to an OpenCV BGR frame, resizes it, and pushes it
into a Vuer WebRTC stream. Vuer encodes H.264 and displays the two streams as
separate head-locked panels. Do not treat two independent workcell cameras as
a stereo pair unless they are synchronized, calibrated, rectified, and the
display path is designed for stereo.

Start with:

```yaml
camera_fps: 20.0
camera_width: 640
camera_height: 480
camera_codec: H264
camera_bitrate_bps: 2000000
```

Increase frame rate or bitrate only after hand tracking and robot command
latency remain stable on the workcell Wi-Fi.

## 12. Safe bring-up sequence

### Stage A: Vuer page only

Run the bridge with camera streams disabled and no YuMi adapter:

```bash
ros2 launch vuer_quest_bridge vuer_bridge.launch.py \
  enable_camera_streams:=false
```

Enter VR and confirm both hands are visible through Vuer's native hand meshes.
The custom landmark overlay used during early testing was removed because it
added an unnecessary Quest-to-Ubuntu-to-Quest round trip and visibly lagged
behind the native mesh.

### Stage B: Validate ROS tracking

```bash
ros2 topic hz /teleop_hand_tracking/right/wrist
ros2 topic hz /teleop_hand_tracking/left/wrist
ros2 topic echo /teleop_hand_tracking/right/landmarks --once
ros2 topic echo /teleop_hand_tracking/left/landmarks --once
```

Confirm that messages stop when the corresponding hand is no longer tracked.

### Stage C: Fake YuMi

Launch the YuMi model, planning stack, and controllers in fake/mock mode. Keep
the teleop adapter's output disabled. Visualize the calculated target pose
first, then enable fake commands at low speed.

Verify:

- left/right hand-to-arm mapping;
- all translation directions;
- translation scale;
- orientation direction;
- joint and Cartesian limits;
- singularity and IK-failure behavior;
- stale-tracking stop behavior;
- gripper direction and range.

### Stage D: Camera panels

```bash
ros2 topic hz /camera_left/color/image_raw/compressed
ros2 topic hz /camera_right/color/image_raw/compressed
```

Enable `enable_camera_streams` and verify both panels before combining camera
encoding with fake robot motion.

### Stage E: Real YuMi

Only after fake-hardware validation:

1. Clear the real workcell and keep the physical emergency stop accessible.
2. Use the lowest supported speed/acceleration limits.
3. Start with one arm and orientation tracking disabled.
4. Require an explicit enable/deadman action after tracking is stable.
5. Test small translations near a safe home pose.
6. Test loss of hand tracking and Quest disconnection.
7. Add the second arm, grippers, and orientation one feature at a time.

The software enable service is not a replacement for ABB safety systems,
protective stops, workspace supervision, or the physical emergency stop.

## 13. Validation commands

Build the standalone package:

```bash
cd <yumi-workspace>
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install --packages-select vuer_quest_bridge
source install/setup.bash
```

Run the pure parser tests:

```bash
pytest -q src/vuer_quest_bridge/test/test_vuer_hand_tracking.py
```

Check interfaces:

```bash
ros2 node info /vuer_quest_bridge
ros2 topic list | grep teleop_hand_tracking
ros2 topic type /teleop_hand_tracking/right/landmarks
ros2 topic hz /teleop_hand_tracking/right/landmarks
```

Expected landmark type:

```text
geometry_msgs/msg/PoseArray
```

Check the server from another device on the same Wi-Fi:

```bash
curl -kI https://<ubuntu-wifi-ip>:8012/
```

## 14. Common porting failures

- **Quest page opens but there is no hand data:** enter immersive VR, grant
  hand permission, and confirm the page uses HTTPS plus a WSS query URL.
- **Quest says the server sent no data:** confirm port `8012`, Ubuntu's Wi-Fi
  IP, the running bridge process, and the firewall.
- **HTTPS works but WebRTC cameras are black:** verify compressed ROS topics,
  image encoding, host networking, and WebRTC/UDP firewall behavior.
- **Only one hand publishes:** check whether the other hand is visible to the
  Quest cameras; an absent-hand binary sentinel is normal.
- **Robot moves in the wrong direction:** correct `R_vuer_to_yumi`; do not
  modify the low-level WebXR decoding to compensate for workcell mounting.
- **Robot jumps when enabling:** use relative calibration, wait for stable
  tracking, record the current TCP origin, and cap the first target step.
- **Motion continues after tracking loss:** fix the adapter watchdog before
  any further real-hardware test.
- **Camera video makes tracking lag:** reduce camera FPS, resolution, and
  bitrate, or use a dedicated 5/6 GHz access point.
- **Solid VR instead of the real room:** select Vuer's Passthrough mode and
  allow the browser/headset permission; this is separate from ROS tracking.

## 15. Porting checklist

- [ ] Standalone Vuer bridge package builds in the YuMi ROS environment.
- [ ] Vuer, OpenCV, and compatible NumPy versions are pinned in Docker.
- [ ] TLS certificate contains the stable Ubuntu Wi-Fi IP.
- [ ] Quest can load the HTTPS/WSS URL on port `8012`.
- [ ] Both wrist topics publish and stop when tracking is lost.
- [ ] Both 25-pose landmark topics have the documented order.
- [ ] YuMi hand-to-arm mapping is configurable.
- [ ] Relative hand/TCP calibration is implemented independently per arm.
- [ ] Workspace, velocity, acceleration, joint, and target-step limits exist.
- [ ] A stale-tracking watchdog stops new commands.
- [ ] Fake YuMi motion is validated before real hardware.
- [ ] Camera topics exist and both Quest panels are visible.
- [ ] Explicit enable/deadman and physical emergency-stop procedures are used.

## References

- `VUER_QUEST_TELEOP.md` — operation and troubleshooting in this workspace
- `VUER_QUEST_COMMANDS.md` — terminal-by-terminal commands
- `VUER_HAND_TOPICS.md` — complete landmark message and index order
- [Vuer hand tracking example](https://docs.vuer.ai/en/latest/examples/vr_xr/hand_tracking.html)
- [Vuer TLS and WebXR setup](https://docs.vuer.ai/en/latest/tutorials/basics/ssl_proxy_webxr.html)
- [Vuer source repository](https://github.com/vuer-ai/vuer)

The only missing YuMi-specific information is the exact driver/controller
interface already used by that workspace. Once its ROS distribution, driver,
MoveIt groups, TCP frames, controller actions/topics, gripper interface, and
camera topic names are known, the adapter and combined launch file can be
implemented without changing the Quest/Vuer bridge.
