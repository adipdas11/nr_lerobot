# SmolVLA Runtime Guide

This is the current source of truth for SmolVLA dataset conversion, training, and inference under `lerobot_pick_place/`.

## Contents

- [Overview](#overview)
- [Files](#files)
- [Runtime Architecture](#runtime-architecture)
- [New Machine Docker Setup](#new-machine-docker-setup)
- [Dataset Conversion](#dataset-conversion)
- [Training](#training)
- [Inference In Isaac](#inference-in-isaac)
- [Inference On Real Hardware](#inference-on-real-hardware)
- [Validation](#validation)
- [Known Constraints](#known-constraints)
- [Troubleshooting](#troubleshooting)

## Overview

The current workflow is:

1. Record rosbags.
2. Convert rosbags to a LeRobot dataset.
3. Train a new SmolVLA checkpoint from that dataset.
4. Run the split inference runtime against Isaac Sim or real hardware.

Important correction:

- The dataset converter now extracts xarm joints by name from `JointState`.
- The gripper value is now read from the recorded joint state instead of being hardcoded.

Use a checkpoint trained from the regenerated dataset only. Older checkpoints trained from the bad converted dataset should not be used.

## Files

Main files used in the current flow:

- `smolvla_policy_server.py`
  Python 3.12 inference server. Loads the trained SmolVLA checkpoint and runs policy inference.

- `smolvla_ros_client.py`
  Python 3.10 ROS 2 Humble client. Subscribes to ROS topics, sends observations to the model server, and streams commands through the motion backend.

- `smolvla_ipc.py`
  Unix-socket IPC helper used between the ROS client and policy server.

- `run_smolvla_policy.sh`
  Wrapper that starts both processes in the correct order.

- `rosbags_to_leorobot_data/rosbag_to_lerobot.py`
  Rosbag-to-LeRobot conversion script.

- `lerobot_converted_dataset/`
  Converted training dataset.

- `smolvla_training_output/`
  Training outputs and checkpoints.

Legacy file:

- `smolvla_inference_node.py`
  Earlier single-process attempt. Not the recommended runtime path.

## Runtime Architecture

The runtime is split because:

- local LeRobot code runs in Python 3.12
- ROS 2 Humble `rclpy` in this workspace runs in Python 3.10

The split path is:

1. `smolvla_policy_server.py` runs in Python 3.12 and owns the model.
2. `smolvla_ros_client.py` runs in Python 3.10 and owns ROS I/O.
3. The two processes communicate over `/tmp/smolvla_policy.sock`.

### Observation Mapping

The ROS client builds the 6-D state in this order:

1. `xarm5_joint1`
2. `xarm5_joint2`
3. `xarm5_joint3`
4. `xarm5_joint4`
5. `xarm5_joint5`
6. `xarm_gripper_right_drive_joint`

The policy server receives:

- the 6-D joint state
- compressed camera 1 image bytes
- compressed camera 2 image bytes
- task string

The model returns a `50 x 6` action chunk in the same order.

## New Machine Docker Setup

This section assumes:

- you will do all work inside the workspace Docker container
- the workspace is mounted inside the container
- the Python 3.12 model environment will live in the repo-local `.venv`
- ROS 2 Humble is already present in the Docker image

Recommended in-container repo path:

- `/workspace/disassembly_ws/src/agentic_disassembly`

Recommended in-container workspace root:

- `/workspace/disassembly_ws`

### 1. Start The Workspace Container

Use your normal workspace Docker or Compose entrypoint, but make sure the container has:

- GPU access
- host ROS networking or equivalent ROS connectivity
- the workspace mounted at `/workspace/disassembly_ws`
- a persistent Hugging Face cache mounted if you want offline model loading

Example shape only:

```bash
docker run --gpus all --network host -it \
  -v /path/on/host/disassembly_ws:/workspace/disassembly_ws \
  -v /path/on/host/hf_cache:/root/.cache/huggingface \
  <your-workspace-image>
```

### 2. Build The ROS Workspace

Run this inside the container:

```bash
cd /workspace/disassembly_ws
source /opt/ros/humble/setup.bash
colcon build
source /workspace/disassembly_ws/install/setup.bash
```

### 3. Create The Repo-Local Python 3.12 Virtual Environment

Run this inside the container:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
```

### 4. Install LeRobot And Model-Side Python Dependencies

Run this inside the container:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
cd lerobot
pip install -e .
```

If your image does not already contain the required model-side packages, install the missing ones into `.venv`:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
pip install torch transformers safetensors opencv-python numpy pandas pyarrow rosbags
```

### 5. Verify The Two Python Environments

ROS side:

```bash
source /opt/ros/humble/setup.bash
source /workspace/disassembly_ws/install/setup.bash
python3.10 -c "import rclpy; print('ROS Python OK')"
```

Model side:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
python -c "import torch, transformers, cv2, numpy; print('model Python OK', torch.cuda.is_available())"
```

### 6. Put The Runtime Assets In Place

You need these available inside the container:

- the rosbag directory if you want dataset conversion
- the converted dataset if you want training
- the trained checkpoint if you want inference
- the Hugging Face SmolVLM cache if you want offline runtime startup

Relevant paths:

- rosbags:
  `/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/rosbags_to_leorobot_data/pick_place_rosbags`
- converted dataset:
  `/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset/pick_place_rosbags_lerobot`
- training output:
  `/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output`

### 7. Convert The Dataset

If you are starting from rosbags on the new machine, run:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
python lerobot_pick_place/rosbags_to_leorobot_data/rosbag_to_lerobot.py \
  --dir /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/rosbags_to_leorobot_data/pick_place_rosbags \
  --output-root /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset \
  --task "pick up the cube" \
  --arm-joint-names xarm5_joint1 xarm5_joint2 xarm5_joint3 xarm5_joint4 xarm5_joint5 \
  --gripper-joint-name xarm_gripper_right_drive_joint
```

### 8. Start Training

Run this inside the container:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
cd lerobot
python -m lerobot.scripts.lerobot_train \
  --policy.type smolvla \
  --dataset.repo_id my_dataset \
  --dataset.root /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset/pick_place_rosbags_lerobot \
  --dataset.use_imagenet_stats false \
  --wandb.enable false \
  --output_dir /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output \
  --policy.push_to_hub false \
  --tolerance_s 0.05 \
  --batch_size 64 \
  --num_workers 16 \
  --policy.use_amp true
```

### 9. Find The New Checkpoint

After training:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
find lerobot_pick_place/smolvla_training_output -maxdepth 2 -type d -name pretrained_model | sort
```

### 10. Run The Newly Trained Checkpoint

For Isaac:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type isaac \
  --model-dir /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed \
  --dry-run \
  --max-chunks 1
```

For real hardware:

```bash
cd /workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type real \
  --model-dir /workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed \
  --dry-run \
  --max-chunks 1
```

Only move to live execution after the dry-run succeeds.

## Dataset Conversion

Dataset root used by training:

- `/home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset/pick_place_rosbags_lerobot`

Rosbag source directory:

- `/home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/rosbags_to_leorobot_data/pick_place_rosbags`

<details>
<summary>Convert rosbags to LeRobot dataset</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
python lerobot_pick_place/rosbags_to_leorobot_data/rosbag_to_lerobot.py \
  --dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/rosbags_to_leorobot_data/pick_place_rosbags \
  --output-root /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset \
  --task "pick up the cube" \
  --arm-joint-names xarm5_joint1 xarm5_joint2 xarm5_joint3 xarm5_joint4 xarm5_joint5 \
  --gripper-joint-name xarm_gripper_right_drive_joint
```

</details>

Notes:

- The converter skips bag folders with no `/joint_states`.
- Optional LeRobot verification may fail inside this sandbox because Hugging Face dataset locking writes under `~/.cache`. That does not invalidate the conversion output itself.

## Training

Train against the regenerated dataset only.

Training output root:

- `/home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output`

<details>
<summary>Run SmolVLA training</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
source .venv/bin/activate
cd lerobot
python -m lerobot.scripts.lerobot_train \
  --policy.type smolvla \
  --dataset.repo_id my_dataset \
  --dataset.root /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/lerobot_converted_dataset/pick_place_rosbags_lerobot \
  --dataset.use_imagenet_stats false \
  --wandb.enable false \
  --output_dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output \
  --policy.push_to_hub false \
  --tolerance_s 0.05 \
  --batch_size 64 \
  --num_workers 16 \
  --policy.use_amp true
```

</details>

After training, use the checkpoint directory under:

- `lerobot_pick_place/smolvla_training_output/<STEP>/pretrained_model`

Examples:

- `060000`
- `065000`
- `070000`

## Inference In Isaac

### Required Runtime Conditions

Before policy inference, these must already be running:

- Isaac Sim scene publishing `/isaac_joint_states`
- MoveIt and EXOTica stack in Isaac mode
- controller stack publishing `/filtered_joint_states`
- both camera topics available as `sensor_msgs/msg/CompressedImage`

### Launch Order

<details>
<summary>1. Start MoveIt + EXOTica in Isaac mode</summary>

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch nr_dual_arm_moveit_config exotica.launch.py hardware_type:=isaac
```

</details>

<details>
<summary>2. Republish raw camera topics to compressed, if needed</summary>

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera1/realsense_camera/color/image_raw \
  -r out/compressed:=/camera1/realsense_camera/color/image_raw/compressed
```

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera2/realsense_camera/color/image_raw \
  -r out/compressed:=/camera2/realsense_camera/color/image_raw/compressed
```

</details>

<details>
<summary>3. Dry-run the newly trained checkpoint</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type isaac \
  --model-dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed \
  --dry-run \
  --max-chunks 1
```

</details>

<details>
<summary>4. Run live inference</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type isaac \
  --model-dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed
```

</details>

If you omit `--model-dir`, the wrapper selects the latest numeric checkpoint under `smolvla_training_output/`.

## Inference On Real Hardware

### Required Runtime Conditions

Before running policy inference on the real robot, these must already be true:

- the real hardware bridge is up
- MoveIt and EXOTica are running in real mode
- the xarm joint state topic is available on `/robot_joint_states`
- both camera topics are available as `sensor_msgs/msg/CompressedImage`
- the robot is in a safe known pose and the scene is clear

The ROS client will use `/robot_joint_states` automatically when `--hardware-type real` is set.

### Recommended Safety Sequence

Use this order:

1. Power and enable the real robot.
2. Confirm cameras and joint state topics are live.
3. Start MoveIt and EXOTica in real mode.
4. Run one dry-run chunk first.
5. Only then run live inference.

### Launch Order

<details>
<summary>1. Start MoveIt + EXOTica in real mode</summary>

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 launch nr_dual_arm_moveit_config exotica.launch.py hardware_type:=real
```

</details>

<details>
<summary>2. Republish raw camera topics to compressed, if needed</summary>

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera1/realsense_camera/color/image_raw \
  -r out/compressed:=/camera1/realsense_camera/color/image_raw/compressed
```

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/camera2/realsense_camera/color/image_raw \
  -r out/compressed:=/camera2/realsense_camera/color/image_raw/compressed
```

</details>

<details>
<summary>3. Dry-run the newly trained checkpoint on real hardware</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type real \
  --model-dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed \
  --dry-run \
  --max-chunks 1
```

</details>

<details>
<summary>4. Run live inference on real hardware</summary>

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./lerobot_pick_place/run_smolvla_policy.sh \
  --hardware-type real \
  --model-dir /home/adip/workspace/disassembly_ws/src/agentic_disassembly/lerobot_pick_place/smolvla_training_output/<NEW_CHECKPOINT>/pretrained_model \
  --camera1-topic /camera1/realsense_camera/color/image_raw/compressed \
  --camera2-topic /camera2/realsense_camera/color/image_raw/compressed
```

</details>

If you omit `--model-dir`, the wrapper selects the latest numeric checkpoint under `smolvla_training_output/`.

### Real-Hardware Checks Before Live Run

Run these before the live command:

```bash
ros2 topic echo --once /robot_joint_states
ros2 topic echo --once /camera1/realsense_camera/color/image_raw/compressed
ros2 topic echo --once /camera2/realsense_camera/color/image_raw/compressed
ros2 action list | grep follow_joint_trajectory
```

During the live run, watch:

```bash
ros2 topic echo /xarm5_controller/joint_trajectory
ros2 topic echo /xarm_gripper_controller/joint_trajectory
```

If the checkpoint is new and unvalidated, prefer starting with:

- `--dry-run --max-chunks 1`
- then a short supervised live test

Do not go straight to unattended execution on the real arm.

## Validation

### Confirm The Policy Is Producing Actions

In dry-run mode, the ROS client should log a chunk-level range summary.

### Confirm Commands Are Being Sent

```bash
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /xarm5_controller/joint_trajectory
ros2 topic echo /xarm_gripper_controller/joint_trajectory
```

### Confirm Isaac Is Receiving Commands

```bash
ros2 topic echo /isaac_joint_commands
ros2 topic echo /filtered_joint_states
```

## Known Constraints

- The LeRobot metadata still uses generic feature names `joint1..joint5, gripper`.
  The runtime maps those six dimensions to xarm joints explicitly.
- The wrapper forces offline Hugging Face mode because the SmolVLM base assets are cached locally.
- Empty bag folders are skipped during conversion.

## Troubleshooting

### No compressed camera topics

Republish raw image topics with `image_transport republish`.

### No motion in Isaac

Check:

```bash
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /xarm5_controller/joint_trajectory
ros2 topic echo /isaac_joint_commands
```

If chunk prediction logs appear but there is no command traffic, the issue is on the ROS controller side.

If controller traffic exists but Isaac does not move, the issue is on the Isaac side or the trained model output.

### Socket server does not start

Check:

- checkpoint path exists
- `.venv` exists
- `/tmp/smolvla_policy.sock` is not blocked by a stale process

### ROS client cannot connect to server

```bash
ls -l /tmp/smolvla_policy.sock
```
