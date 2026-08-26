# NR Dual-Arm ROS 2 and LeRobot ACT

Expand a section below when you are ready to run that part of the workflow.
The Overview stays visible; all other sections are collapsed by default.

## Overview

This guide covers the complete workflow on a new Ubuntu system:

1. Clone the workspace in the directory layout expected by Docker.
2. Build and enter the ROS 2 Humble teleoperation container.
3. Start the dual-arm system in fake, real, or Isaac mode.
4. Start either RealSense camera or both cameras.
5. Teleoperate with Meta Quest/Vuer, MediaPipe, or a joystick.
6. Record and inspect ROS bags.
7. Convert the bags to a LeRobot v3 dataset.
8. Train an ACT policy.
9. Run inference from a saved checkpoint.

> **Safety:** Always test motion in `fake` mode first. Before using `real` mode,
> clear the workspace, keep the emergency stop accessible, and start with robot
> control disabled. Do not run two robot bring-up or teleoperation launches at
> the same time.

<details>
<summary><strong>Current ACT scope</strong></summary>

The ROS and teleoperation stack brings up both the UF850 and xArm5. The current
ACT data and inference code is narrower: it learns the five xArm5 joints plus
`xarm_gripper_right_drive_joint` from two RGB cameras. It does not yet train a
policy for both arms.

The ACT recording and conversion pipeline expects these inputs:

- `/joint_states`
- `/camera1/realsense_camera/color/image_raw/compressed`
- `/camera2/realsense_camera/color/image_raw/compressed`
- `xarm5_joint1` through `xarm5_joint5`
- `xarm_gripper_right_drive_joint`

</details>

<details>
<summary><strong>Where commands run</strong></summary>

| Location | Purpose |
| --- | --- |
| Ubuntu host | Clone, Docker build, Vuer TLS setup, dataset conversion, training, and ACT model server |
| Main Docker shell | Robot bring-up or one teleoperation launch |
| Attached Docker shells | Cameras, topic checks, bag recording, and the ACT ROS client |
| Quest Browser | Vuer WebXR hand tracking and camera panels |

Paths in host commands use `~/lerobot_ws`. Inside Docker, that workspace is
mounted at `/ws`, so this repository is `/ws/src/nr_lerobot`.

</details>

<details>
<summary><strong>1. New-system setup</strong></summary>

### 1.1 Install prerequisites

Use Ubuntu 22.04 for ROS 2 Humble compatibility. Install Docker Engine with
either Docker Compose v2 (`docker compose`) or legacy Compose
(`docker-compose`). Make sure Docker works without `sudo` for the current user.

Install the remaining host tools:

```bash
sudo apt update
sudo apt install -y curl ffmpeg git openssl x11-xserver-utils
```

Verify Docker and Compose:

```bash
docker --version
docker compose version || docker-compose --version
docker run --rm hello-world
```

An NVIDIA GPU and working NVIDIA driver are strongly recommended for ACT
training. Isaac Sim is a separate installation and is required only for Isaac
mode.

### 1.2 Create the workspace and clone the repository

The Compose file mounts two directories above the repository as `/ws`.
Therefore, first create the workspace and its `src` directory:

```bash
mkdir -p ~/lerobot_ws/src
cd ~/lerobot_ws/src
```

The repository ID is `adipdas11/nr_lerobot`. Choose exactly one clone method.

SSH:

```bash
git clone --branch lerobot_v1 --recurse-submodules \
  git@github.com:adipdas11/nr_lerobot.git
```

HTTPS:

```bash
git clone --branch lerobot_v1 --recurse-submodules \
  https://github.com/adipdas11/nr_lerobot.git
```

Enter the cloned repository and initialize its submodules:

```bash
cd ~/lerobot_ws/src/nr_lerobot
git submodule update --init --recursive
```

The resulting workspace layout must be:

```text
~/lerobot_ws/
└── src/
    └── nr_lerobot/
```

Confirm that the vendored LeRobot submodule is present:

```bash
git submodule status
test -f lerobot/src/lerobot/scripts/lerobot_train.py
```

### 1.3 Build the ROS/teleoperation Docker image

From the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./setup_teleop_docker.sh
```

Run this again after changing the Dockerfile or container dependencies. Source
changes normally do not require an image rebuild because the workspace is
mounted into the container.

### 1.4 Create the Python 3.12 LeRobot environment

ROS 2 Humble uses Python 3.10, while this LeRobot checkout requires Python
3.12. The local `.venv/` directory is intentionally ignored by Git and is not
pushed to the remote repository. A fresh clone recreates it from the tracked
`pyproject.toml`, `uv.lock`, and `.python-version` files.

The root dependency project contains only the host-side ACT, dataset,
conversion, training, and inference dependencies. ROS, MediaPipe, Vuer, and
hardware-driver dependencies remain in the Docker image because ROS 2 Humble
uses Python 3.10.

Install `uv`, enter the repository, and synchronize the locked environment:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env

cd ~/lerobot_ws/src/nr_lerobot
uv python install
uv sync --frozen
source .venv/bin/activate
```

Verify the environment and CUDA visibility:

```bash
cd ~/lerobot_ws/src/nr_lerobot
uv run --frozen python -c "import lerobot, pandas, pyarrow, rosbags, torch; print(torch.cuda.is_available())"
```

`True` means the environment can use the NVIDIA GPU. CPU operation is possible
but ACT training will be much slower.

When project dependencies change, update and commit the environment metadata:

```bash
cd ~/lerobot_ws/src/nr_lerobot
uv lock
git add pyproject.toml uv.lock .python-version
```

### 1.5 Start Docker and open extra terminals

For the first ROS terminal, run this on the host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

The script builds the required ROS packages when necessary, sources ROS, and
opens a shell at `/ws`.

While that container remains running, open every additional Docker terminal
from a new host terminal with:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

Use `attach_teleop_docker.sh` for additional terminals so all processes share
the same running container.

</details>

<details>
<summary><strong>2. Start the dual-arm system</strong></summary>

Choose exactly one mode. Each command below starts `ros2_control`, the
controllers, MoveIt, robot state publishing, and RViz.

### 2.1 Fake mode

On the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch nr_dual_arm_moveit_config demo.launch.py \
  hardware_type:=fake \
  use_rviz:=true
```

### 2.2 Real mode

Confirm that both robot controllers are reachable and the emergency stop is
accessible. On the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch nr_dual_arm_moveit_config demo.launch.py \
  hardware_type:=real \
  use_rviz:=true
```

### 2.3 Isaac mode

Start Isaac Sim on the Ubuntu host and open one of the repository stages. For
an Isaac Sim installation at `~/isaacsim`:

```bash
cd ~/isaacsim
./isaac-sim.sh \
  ~/lerobot_ws/src/nr_lerobot/nr_dual_arm_description/usd/nr_dual_arm_scene/nr_dual_arm_cube_pick.usd
```

Press **Play** in Isaac Sim. Its ROS graph must publish `/clock` and
`/isaac_joint_states` and subscribe to `/isaac_joint_commands`.

In a second host terminal:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch nr_dual_arm_moveit_config demo.launch.py \
  hardware_type:=isaac \
  use_rviz:=true \
  use_sim_time:=auto
```

From an attached Docker terminal, verify the simulator bridge one command at a
time:

```bash
ros2 topic echo /clock --once
ros2 topic hz /isaac_joint_states
```

Use `Ctrl+C` to stop `ros2 topic hz`.

</details>

<details>
<summary><strong>3. Start the RealSense cameras</strong></summary>

If no Docker container is running, start one:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

If the robot is already running in the main Docker shell, open a new host
terminal and attach instead:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

Inside Docker, inspect the connected camera serial numbers:

```bash
rs-enumerate-devices | grep -E "Name|Serial Number"
```

Pick one of the following launches. Depth is disabled here because ACT uses only
compressed RGB images.

### 3.1 Camera 1 only

```bash
ros2 launch arm_teleop realsense.launch.py \
  enable_camera1:=true \
  enable_camera2:=false \
  enable_depth:=false \
  align_depth:=false
```

### 3.2 Camera 2 only

```bash
ros2 launch arm_teleop realsense.launch.py \
  enable_camera1:=false \
  enable_camera2:=true \
  enable_depth:=false \
  align_depth:=false
```

### 3.3 Both cameras

```bash
ros2 launch arm_teleop realsense.launch.py \
  enable_camera1:=true \
  enable_camera2:=true \
  enable_depth:=false \
  align_depth:=false
```

The configured serials are `233722072289` for camera 1 and `231522072957` for
camera 2. Override them when the installed cameras differ; the leading
underscore is required by the RealSense ROS driver:

```bash
ros2 launch arm_teleop realsense.launch.py \
  camera1_serial:=_SERIAL_ONE \
  camera2_serial:=_SERIAL_TWO \
  enable_depth:=false \
  align_depth:=false
```

Verify the streams from another attached Docker terminal:

```bash
ros2 topic hz /camera1/realsense_camera/color/image_raw/compressed
ros2 topic hz /camera2/realsense_camera/color/image_raw/compressed
```

The current ACT runtime requires both cameras even though either camera can be
launched independently for inspection.

</details>

<details>
<summary><strong>4. Start teleoperation</strong></summary>

Each teleoperation launch below includes the dual-arm EXOTica/MoveIt stack. Do
not start `demo.launch.py` separately. Test in fake mode, stop it with `Ctrl+C`,
and only then start real mode. To teleoperate in Isaac instead, start Isaac Sim
as shown in Section 2.3, do not launch `demo.launch.py`, and change the selected
teleoperation command to `hardware_type:=isaac`.

### 4.1 Meta Quest with Vuer

#### 4.1.1 One-time Vuer TLS setup

Quest and Ubuntu must be on the same LAN. A stable Ubuntu LAN address is
recommended because the certificate contains that address.

On the Ubuntu host, find the LAN IPv4 address:

```bash
ip -4 route get 1.1.1.1
```

Create the certificate, replacing the example address with the `src` address
shown by the previous command:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./setup_vuer_tls.sh 192.168.1.100
```

The script creates:

```text
~/lerobot_ws/.vuer/tls/cert.pem
~/lerobot_ws/.vuer/tls/key.pem
```

These files are mounted into Docker automatically. The key stays on Ubuntu;
there is no certificate file to copy to Quest. Installation on Quest consists
of accepting the local certificate warning in Quest Browser.

If the Ubuntu LAN address changes, preserve the old certificate and generate a
new one:

```bash
mv ~/lerobot_ws/.vuer/tls ~/lerobot_ws/.vuer/tls.previous
cd ~/lerobot_ws/src/nr_lerobot
./setup_vuer_tls.sh 192.168.1.101
```

#### 4.1.2 Test Vuer with fake hardware

On the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch arm_teleop vuer_quest_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  enable_cameras:=true \
  enable_camera_streams:=true
```

If the RealSense cameras are not connected during the first test, use
`enable_cameras:=false enable_camera_streams:=false`.

In Quest Browser:

1. Connect Quest to the same LAN as Ubuntu and enable hand tracking.
2. Open `https://192.168.1.100:8012` and accept the certificate warning.
3. Open `https://192.168.1.100:8012/?ws=wss://192.168.1.100:8012`.
4. Select **Enter VR** and allow hand-tracking access.
5. Hold both hands visible and stationary while the initial poses settle.

Replace `192.168.1.100` with the address used in the certificate.

From an attached Docker terminal, verify tracking and video:

```bash
ros2 topic hz /teleop_hand_tracking/right/wrist
ros2 topic hz /teleop_hand_tracking/left/wrist
ros2 topic hz /camera1/realsense_camera/color/image_raw/compressed
ros2 topic hz /camera2/realsense_camera/color/image_raw/compressed
```

Both arms start disabled. Enable them only after tracking is stable, using the
Tk control panel. To disable both immediately from an attached terminal:

```bash
ros2 service call /exotica_arm_teleop/set_all_arms_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

#### 4.1.3 Start Vuer with real hardware

Stop the fake launch first. Then start a new Docker shell and run:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

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

For the full Vuer operational checklist and troubleshooting, see
[VUER_QUEST_COMMANDS.md](VUER_QUEST_COMMANDS.md) and
[VUER_QUEST_TELEOP.md](VUER_QUEST_TELEOP.md).

### 4.2 MediaPipe webcam teleoperation

Connect the webcam. On the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker, list the video devices:

```bash
v4l2-ctl --list-devices
```

Test with fake hardware and automatic webcam selection:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  camera_index:=-1
```

If auto-detection chooses the wrong device, use the number from `/dev/videoN`,
for example `camera_index:=12`.

After stopping the fake launch, the corresponding real-hardware command is:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=true \
  enable_xarm5:=true \
  uf850_hand:=right \
  xarm5_hand:=left \
  camera_index:=-1
```

Both arms start disabled and are enabled from the Tk control panel. This webcam
is used for MediaPipe tracking; start the RealSense launch separately when
recording ACT data.

### 4.3 Joystick teleoperation

Connect an Xbox-style gamepad. On the Ubuntu host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker, identify the joystick index:

```bash
ls /dev/input/js*
```

For `/dev/input/js0`, test with fake hardware:

```bash
ros2 launch arm_teleop joy_exotica_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  joy_dev:=0
```

Both arms start disabled. Wait for the control panel to show the gamepad as
connected and EXOTica as ready before enabling an arm.

| Control | Action |
| --- | --- |
| Left stick horizontal/vertical | Move the selected arm in Y/X |
| Right stick vertical | Move in Z |
| Right stick horizontal | Rotate yaw |
| LB | Switch between xArm5 and UF850 |
| X | Enable or disable the selected arm |
| B | Enable or disable both arms |
| RB | Open or close the selected gripper |

After stopping fake mode, start real joystick teleoperation with:

```bash
ros2 launch arm_teleop joy_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  joy_dev:=0
```

Use `joy_dev:=1` for `/dev/input/js1`, and so on.

</details>

<details>
<summary><strong>5. Record ROS bags for ACT</strong></summary>

Record one demonstration per rosbag directory. Use zero-padded episode names so
their sorted order is unambiguous. The example below uses joystick teleoperation
on real hardware; Vuer or MediaPipe can be used instead.

### 5.1 Terminal 1: start robot teleoperation

On the host:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch arm_teleop joy_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  joy_dev:=0
```

For an initial dry workflow, replace `real` with `fake`.

### 5.2 Terminal 2: start both recording cameras

Open another host terminal:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

Inside Docker:

```bash
ros2 launch arm_teleop realsense.launch.py \
  enable_camera1:=true \
  enable_camera2:=true \
  enable_depth:=false \
  align_depth:=false
```

Skip this terminal when the Vuer launch already started both cameras.

### 5.3 Terminal 3: check topics and record one episode

Open another host terminal:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

Inside Docker, confirm the required topics and joint names before recording:

```bash
ros2 topic list -t | grep -E \
  "/joint_states|/camera[12]/realsense_camera/color/image_raw/compressed"
ros2 topic echo /joint_states --once
```

The joint-state message must include `xarm5_joint1` through `xarm5_joint5` and
`xarm_gripper_right_drive_joint`.

Create the bag directory and record episode 1:

```bash
mkdir -p /ws/src/nr_lerobot/lerobot_pick_place/act/rosbags
ros2 bag record \
  -o /ws/src/nr_lerobot/lerobot_pick_place/act/rosbags/episode_0001 \
  /joint_states \
  /camera1/realsense_camera/color/image_raw/compressed \
  /camera2/realsense_camera/color/image_raw/compressed
```

Perform one complete demonstration, then press `Ctrl+C` and wait for rosbag to
finish writing `metadata.yaml`. Repeat with `episode_0002`, `episode_0003`, and
so on. Never reuse an existing episode output path.

### 5.4 Check that the ROS bags are usable

Run the dedicated verifier on the Ubuntu host. It uses the Python 3.12 project
environment and does not require another Docker terminal.

Verify the most recently recorded episode:

```bash
cd ~/lerobot_ws/src/nr_lerobot
uv run --frozen python lerobot_pick_place/act/verify_latest_rosbag.py
```

Verify one explicit episode:

```bash
cd ~/lerobot_ws/src/nr_lerobot
uv run --frozen python lerobot_pick_place/act/verify_latest_rosbag.py \
  --bag lerobot_pick_place/act/rosbags/episode_0001
```

Verify every recorded episode:

```bash
cd ~/lerobot_ws/src/nr_lerobot
uv run --frozen python lerobot_pick_place/act/verify_latest_rosbag.py --all
```

The verifier returns exit code `0` and prints `Overall result: PASS` only when
every selected bag contains readable joint and camera messages, the complete
xArm5 and gripper joint set, nonempty compressed images, and sufficient motion
for every required joint. Any failed check returns a nonzero exit code.

</details>

<details>
<summary><strong>6. Convert ROS bags to a LeRobot dataset</strong></summary>

Run conversion on the Ubuntu host, not inside the ROS container:

```bash
cd ~/lerobot_ws/src/nr_lerobot
source .venv/bin/activate

python lerobot_pick_place/act/rosbag_to_lerobot_act.py \
  --dir lerobot_pick_place/act/rosbags \
  --output lerobot_pick_place/act/lerobot_converted_dataset/pick_place_act \
  --fps 30 \
  --task "pick up the cube" \
  --validate
```

Use a new or empty output directory. A successful run prints the number of
episodes, frames, cameras, and `[Validate] Dataset structure OK`.

The generated dataset contains:

```text
lerobot_pick_place/act/lerobot_converted_dataset/pick_place_act/
├── data/       # Parquet state and action data
├── videos/     # One MP4 per camera and episode
└── meta/       # Dataset metadata, statistics, tasks, and episode index
```

The task string is saved as metadata, but ACT itself is not
language-conditioned.

</details>

<details>
<summary><strong>7. Train the LeRobot ACT policy</strong></summary>

Run training on the Ubuntu host. The output directory must not already exist
unless resuming a run.

```bash
cd ~/lerobot_ws/src/nr_lerobot
source .venv/bin/activate

python -c "import torch; print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CUDA unavailable')"

lerobot-train \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --dataset.repo_id=pick_place_act \
  --dataset.root=lerobot_pick_place/act/lerobot_converted_dataset/pick_place_act \
  --dataset.use_imagenet_stats=true \
  --output_dir=lerobot_pick_place/act/training_output \
  --job_name=pick_place_act \
  --wandb.enable=false \
  --batch_size=8 \
  --num_workers=4 \
  --steps=100000 \
  --save_freq=5000
```

Reduce `batch_size` if GPU memory is exhausted. Set `--policy.device=cpu` only
when CUDA is unavailable.

Checkpoints are written to:

```text
lerobot_pick_place/act/training_output/checkpoints/005000/pretrained_model/
lerobot_pick_place/act/training_output/checkpoints/010000/pretrained_model/
...
lerobot_pick_place/act/training_output/checkpoints/100000/pretrained_model/
```

</details>

<details>
<summary><strong>8. Run ACT inference from a checkpoint</strong></summary>

Stop Vuer, MediaPipe, and joystick teleoperation before live policy inference;
otherwise two nodes may send competing robot commands. Start only the dual-arm
system from Section 2 and both cameras from Section 3, then wait for controllers
and camera topics to become ready.

The Docker-first setup uses two inference processes because LeRobot requires
Python 3.12 on the host while ROS 2 Humble uses Python 3.10 in Docker. A Unix
socket under the mounted workspace connects them.

### 8.1 Terminal 3 on the host: start the ACT model server

This command automatically selects the highest-numbered checkpoint:

```bash
cd ~/lerobot_ws/src/nr_lerobot
source .venv/bin/activate

python lerobot_pick_place/act/act_policy_server.py \
  --checkpoint-root lerobot_pick_place/act/training_output \
  --socket-path lerobot_pick_place/act/act_policy.sock \
  --device auto
```

`auto` selects CUDA when available and otherwise uses the CPU.

To select one checkpoint explicitly, replace `--checkpoint-root` with:

```bash
--model-dir lerobot_pick_place/act/training_output/checkpoints/100000/pretrained_model
```

Leave the model server running.

### 8.2 Terminal 4 in Docker: perform a dry run

Open a new host terminal:

```bash
cd ~/lerobot_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

Inside Docker, request one action chunk without commanding the robot:

```bash
python3.10 /ws/src/nr_lerobot/lerobot_pick_place/act/act_ros_client.py \
  --hardware-type real \
  --socket-path /ws/src/nr_lerobot/lerobot_pick_place/act/act_policy.sock \
  --dry-run \
  --max-chunks 1
```

Confirm that the client receives joint states, both camera images, and an action
chunk. Stop it with `Ctrl+C` after the dry-run result.

### 8.3 Terminal 4 in Docker: start live inference

Only after the dry run is correct and the physical workspace is safe, remove
the dry-run options:

```bash
python3.10 /ws/src/nr_lerobot/lerobot_pick_place/act/act_ros_client.py \
  --hardware-type real \
  --socket-path /ws/src/nr_lerobot/lerobot_pick_place/act/act_policy.sock
```

For Isaac or fake mode, use the matching argument:

```bash
--hardware-type isaac
```

```bash
--hardware-type fake
```

Live inference begins sending action chunks as soon as the client has joint
states and both images. Use `Ctrl+C` to stop the client, then stop the model
server.

### 8.4 Optional host-ROS shortcut

If ROS 2 Humble is installed on the Ubuntu host and `~/lerobot_ws` has also been
built with colcon on the host, `run_act_policy.sh` starts both inference
processes:

```bash
cd ~/lerobot_ws/src/nr_lerobot/lerobot_pick_place/act
./run_act_policy.sh --real --checkpoint 100000 --dry-run --max-chunks 1
```

After the dry run, start live real inference with:

```bash
cd ~/lerobot_ws/src/nr_lerobot/lerobot_pick_place/act
./run_act_policy.sh --real --checkpoint 100000
```

Use `--isaac` instead of `--real` for Isaac mode. Omit `--checkpoint` to load
the highest-numbered checkpoint automatically.

</details>

<details>
<summary><strong>9. Shutdown order</strong></summary>

1. Disable robot control or stop the ACT ROS client.
2. Stop rosbag recording and wait for `metadata.yaml` to be written.
3. Stop camera nodes.
4. Stop the robot or teleoperation launch.
5. Stop the ACT model server, if running.
6. Exit attached Docker shells, then exit the main Docker shell.
7. Stop Isaac Sim last when using Isaac mode.

Generated rosbags, converted datasets, and training outputs are ignored by Git.
Do not commit robot IP addresses, model weights, or recorded data.

</details>
