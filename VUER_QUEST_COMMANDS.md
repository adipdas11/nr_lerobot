# Vuer Quest Teleoperation Command Runbook

This is the short operational runbook for running Meta Quest teleoperation
directly with the Ubuntu computer over the same Wi-Fi network.

Use the sections in order. Test with fake hardware before starting the real
robots. Do not run the fake and real launch commands simultaneously.

## Terminal layout

| Terminal | Where it runs | Purpose |
| --- | --- | --- |
| Terminal 1 | Ubuntu host, then Docker | Main teleoperation launch |
| Terminal 2 | Ubuntu host, then Docker | Topic checks and enable/disable services |
| Quest | Quest Browser | WebXR hand tracking and camera display |

## 1. One-time setup on Ubuntu

Open a terminal on Ubuntu and enter the repository:

```bash
cd /home/adip/workspace/lerobot_ws/src/nr_lerobot
```

Find the Ubuntu Wi-Fi IPv4 address:

```bash
hostname -I
```

Use the address belonging to the same Wi-Fi network as the Quest. In the
examples below, the Ubuntu Wi-Fi address is `192.168.1.100`. Replace it with the
actual address.

Create the HTTPS/WSS certificate:

```bash
./setup_vuer_tls.sh 192.168.1.100
```

Build the Docker image:

```bash
./setup_teleop_docker.sh
```

The certificate only needs to be recreated if the Ubuntu Wi-Fi address changes.
The Docker image only needs to be rebuilt after Dockerfile or dependency
changes.

## 2. Optional RealSense check

Connect both RealSense cameras to USB 3 ports. Start the Docker shell:

```bash
cd /home/adip/workspace/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker, list the cameras:

```bash
rs-enumerate-devices | grep -E 'Name|Serial Number'
```

The configured defaults are:

```text
camera1: 233722072289
camera2: 231522072957
```

If the physical serials differ, pass them to the launch command with a leading
underscore, for example:

```text
camera1_serial:=_123456789 camera2_serial:=_987654321
```

Leave Docker when finished with this optional check:

```bash
exit
```

## 3. Fake-hardware teleoperation

### Terminal 1 — start Docker

On the Ubuntu host:

```bash
cd /home/adip/workspace/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

### Terminal 1 — launch fake hardware

Inside Docker:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  hide_hand_meshes:=false \
  enable_cameras:=true \
  enable_camera_streams:=true
```

If the RealSense cameras are not connected during the fake test, use:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  hide_hand_meshes:=false \
  enable_cameras:=false \
  enable_camera_streams:=false
```

Wait until the Vuer URL, RViz, EXOTica readiness message, and teleoperation
control panel appear. Both arms start disabled.

## 4. Quest Browser steps

No Quest terminal or Unity application is required.

1. Connect Quest to the same Wi-Fi network as Ubuntu.
2. Enable hand tracking in the Quest system settings.
3. Open Quest Browser.
4. Open the Vuer URL using the Ubuntu Wi-Fi address:

```text
https://192.168.1.100:8012/?ws=wss://192.168.1.100:8012
```

5. Accept the local certificate warning. If necessary, first open:

```text
https://192.168.1.100:8012
```

6. After accepting the certificate, reload the full URL containing `?ws=`.
7. Select **Enter VR** in Vuer.
8. Grant the browser permission to use hand tracking.
9. Hold both hands visible and stationary while checking the initial poses.

With the cameras enabled, two independent RealSense views should appear as
head-locked panels. They are separate workcell views, not left-eye/right-eye
stereo images.

## 5. Verify tracking in Terminal 2

Open a second Ubuntu terminal while Terminal 1 remains running:

```bash
cd /home/adip/workspace/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside the second Docker shell, check the Vuer hand topics:

```bash
ros2 topic hz /teleop_hand_tracking/right/wrist
```

In separate command runs, check the left hand and landmark array:

```bash
ros2 topic hz /teleop_hand_tracking/left/wrist
ros2 topic echo /teleop_hand_tracking/right/landmarks --once
ros2 topic echo /teleop_hand_tracking/left/landmarks --once
```

Check the gesture and gripper values:

```bash
ros2 topic echo /teleop_hand_tracking/right/pinch
ros2 topic echo /teleop_hand_tracking/right/pinky_pinch
ros2 topic echo /teleop_hand_tracking/right/gripper_aperture
```

Check both camera streams:

```bash
ros2 topic hz /camera1/realsense_camera/color/image_raw/compressed
ros2 topic hz /camera2/realsense_camera/color/image_raw/compressed
```

Use `Ctrl+C` to stop each topic command.

## 6. Test motion with fake hardware

Confirm all of the following before enabling:

- Both wrists update continuously.
- Left and right hands control the intended arms.
- Small hand motion produces motion in the expected RViz direction.
- The robot targets do not jump when tracking begins.
- Both camera panels update in Quest.

Enable the arms using the Tk teleoperation control panel. Alternatively, from
Terminal 2:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

Disable them again before ending the fake test:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

## 7. Stop fake hardware before using real hardware

In Terminal 1, disable both arms, then stop the launch:

```text
Ctrl+C
```

Wait for all fake controllers, EXOTica nodes, camera nodes, and the Vuer bridge
to stop. Do not start real mode while the fake launch is still running.

## 8. Real-hardware safety preparation

Before the real launch:

- Make sure the robot workspaces are clear.
- Confirm both robot controllers are reachable from Ubuntu.
- Keep the physical emergency stop accessible.
- Begin with both arms disabled.
- Keep both hands visible and stationary during initial calibration.
- Do not enable an arm until its Quest hand pose and RViz target are stable.

## 9. Real-hardware teleoperation

### Terminal 1 — Docker shell

If the previous Docker shell was closed, run:

```bash
cd /home/adip/workspace/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

### Terminal 1 — launch real hardware

Inside Docker:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  enable_cameras:=true \
  enable_camera_streams:=true
```

If different RealSense serials are installed:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  enable_cameras:=true \
  enable_camera_streams:=true \
  camera1_serial:=_SERIAL_ONE \
  camera2_serial:=_SERIAL_TWO
```

Repeat the Quest Browser steps if the Vuer page disconnected.

## 10. Enable real arms individually

Using the control panel is recommended. To enable only the UF850/right-hand
arm from Terminal 2:

```bash
ros2 service call /exotica_arm_teleop/set_right_arm_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

To enable only the xArm5/left-hand arm:

```bash
ros2 service call /exotica_arm_teleop/set_left_arm_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

Enable both only after testing each arm separately:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

Immediately disable both arms if tracking, Wi-Fi, video, or robot motion becomes
unstable:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

The physical emergency stop remains the primary response to unsafe motion.

## 11. Recalibration and home commands

Recalibrate the hand-to-robot relationship while both hands are stationary:

```bash
ros2 service call /exotica_arm_teleop/recalibrate std_srvs/srv/Trigger "{}"
```

Request the configured home motion only when the real workspace is clear:

```bash
ros2 service call /exotica_arm_teleop/go_home std_srvs/srv/Trigger "{}"
```

## 12. Shutdown procedure

1. Disable both arms from Terminal 2:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

2. Stop the main launch in Terminal 1 with `Ctrl+C`.
3. Wait for the robot, camera, EXOTica, and Vuer nodes to stop.
4. Close the Vuer tab in Quest Browser.
5. Exit both Docker shells:

```bash
exit
```

## 13. Common command variations

Run only the xArm5 with the right hand:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=real \
  enable_uf850:=false \
  enable_xarm5:=true \
  xarm5_hand:=right \
  enable_cameras:=true
```

Run only the UF850 with the right hand:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=real \
  enable_uf850:=true \
  enable_xarm5:=false \
  uf850_hand:=right \
  enable_cameras:=true
```

Run hand tracking without cameras:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=fake \
  enable_cameras:=false \
  enable_camera_streams:=false
```

## 14. Wi-Fi and browser troubleshooting commands

Show the Ubuntu Wi-Fi address:

```bash
ip -4 address
```

Confirm that Vuer is listening on port 8012:

```bash
ss -ltn | grep 8012
```

Test the Vuer HTTPS page from Ubuntu:

```bash
curl -k -I https://127.0.0.1:8012
```

If Quest cannot connect even though Ubuntu can open the page, verify that both
devices use the same Wi-Fi network and that the router is not using guest-mode,
client isolation, or AP isolation.

For design details and configuration explanations, see
[`VUER_QUEST_TELEOP.md`](VUER_QUEST_TELEOP.md).
