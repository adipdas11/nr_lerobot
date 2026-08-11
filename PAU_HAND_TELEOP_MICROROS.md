# Hand teleoperation and micro-ROS in Docker

This setup is available on the `pau` branch. The hand packages came from
`paum007/NR-Internship` commit
`bfc6af5b9ff5db1cdb88e767fd194714e0b4af26`.

## Check the new system prerequisites

The Ubuntu system must have Git, Git LFS, Docker Engine, Docker Compose, and
X11 support installed. Confirm that the commands are available:

```bash
git --version
git lfs version
docker --version
docker compose version || docker-compose --version
xhost -help >/dev/null
```

Add the current user to the Docker group if Docker requires `sudo`:

```bash
sudo usermod -aG docker "$USER"
newgrp docker
docker ps
```

## Clone the `pau` branch into a ROS workspace

The repository must be inside the workspace `src` directory because the
Compose configuration mounts the workspace into Docker as `/ws`:

```bash
mkdir -p ~/teleop_ws/src
cd ~/teleop_ws/src
git lfs install
git clone --branch pau --recurse-submodules https://github.com/adipdas11/nr_lerobot.git
cd nr_lerobot
git lfs pull
```

The clone command becomes available on another system after the local `pau`
branch has been committed and pushed to `origin`.

## Build the Docker image once on the new system

Run the provided setup script from the repository directory:

```bash
cd ~/teleop_ws/src/nr_lerobot
./setup_teleop_docker.sh
```

This image contains ROS 2 Humble and the pinned Humble-compatible micro-ROS
Agent under `/opt/micro_ros_humble`.

## Find the Ubuntu Wi-Fi IP address

```bash
hostname -I
```

Use the address belonging to the Wi-Fi adapter as `<UBUNTU_WIFI_IP>` in the
following commands and in the NodeMCU firmware.

## Create the Vuer HTTPS certificate once

```bash
cd ~/teleop_ws/src/nr_lerobot
./setup_vuer_tls.sh <UBUNTU_WIFI_IP>
```

Regenerate the certificate if the Ubuntu Wi-Fi IP address changes.

Remove the old certificate before regenerating it for a changed address:

```bash
rm ~/teleop_ws/.vuer/tls/cert.pem ~/teleop_ws/.vuer/tls/key.pem
cd ~/teleop_ws/src/nr_lerobot
./setup_vuer_tls.sh <NEW_UBUNTU_WIFI_IP>
```

## Start and enter Docker in Terminal 1

The first normal run builds the mounted ROS workspace and opens a shell inside
the running container:

```bash
cd ~/teleop_ws/src/nr_lerobot
./run_teleop_docker.sh
```

Use a clean rebuild after changing the Dockerfile, changing dependencies, or
recovering from an incomplete build:

```bash
cd ~/teleop_ws/src/nr_lerobot
./run_teleop_docker.sh --clean
```

Keep Terminal 1 and its Docker shell open. Closing it stops and removes this
temporary Compose container.

## Attach another shell in Terminal 2

Run the provided attach script from a host terminal:

```bash
cd ~/teleop_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

The script locates the running teleoperation container and opens another shell
with the workspace sourced.

## Start the Vuer hand-tracking bridge in Terminal 2

Inside the attached Docker shell:

```bash
source /opt/ros/humble/setup.bash
source /opt/micro_ros_humble/setup.bash
source /ws/install/setup.bash

ros2 run arm_teleop vuer_quest_bridge --ros-args \
  --params-file /ws/install/arm_teleop/share/arm_teleop/config/vuer_quest_bridge.yaml \
  -p cert_file:="$VUER_CERT_FILE" \
  -p key_file:="$VUER_KEY_FILE" \
  -p enable_camera_streams:=false \
  -p enable_hand_tracking:=true \
  -p hide_hand_meshes:=false
```

Open this address in the Meta Quest Browser and enter VR:

```text
https://vuer.ai?ws=wss://<UBUNTU_WIFI_IP>:8012
```

## Attach another shell in Terminal 3

From another host terminal:

```bash
cd ~/teleop_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

## Start the hand-angle node in Terminal 3

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /opt/micro_ros_humble/setup.bash
source /ws/install/setup.bash
ros2 launch hand_teleoperation node_launch.py
```

The original upstream launch filename is also available:

```bash
ros2 launch hand_teleoperation node.launch.py
```

The node subscribes to:

```text
/teleop_hand_tracking/left/landmarks
/teleop_hand_tracking/right/landmarks
```

It publishes the calculated finger angles on:

```text
/hand_angles
```

## Attach another shell in Terminal 4

From another host terminal:

```bash
cd ~/teleop_ws/src/nr_lerobot
./attach_teleop_docker.sh
```

## Start the Wi-Fi micro-ROS Agent in Terminal 4

Inside Docker:

```bash
source /opt/ros/humble/setup.bash
source /opt/micro_ros_humble/setup.bash
source /ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port "$MICRO_ROS_AGENT_PORT" -v6
```

The Agent should report that `UDPv4AgentLinux` is running on port `8888`.

## Configure the NodeMCU Wi-Fi transport

Configure the micro-ROS firmware with these values:

```text
Wi-Fi network: same network as the Ubuntu system
Agent address: <UBUNTU_WIFI_IP>
Agent transport: UDP over IPv4
Agent port: 8888
ROS domain ID: 0
```

The NodeMCU connects directly to the Agent address and port. The Ubuntu access
point or router must allow devices on the Wi-Fi network to communicate with
each other; client/AP isolation must be disabled.

## Check the Docker ROS network settings

Inside Docker:

```bash
env | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|MICRO_ROS_AGENT_PORT)='
```

Expected values:

```text
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
MICRO_ROS_AGENT_PORT=8888
```

## Check that the micro-ROS Agent is listening

Inside Docker or on the Ubuntu host:

```bash
ss -lunp | grep ':8888'
```

## Check the Vuer landmark topics

Inside an attached Docker shell:

```bash
ros2 topic echo /teleop_hand_tracking/left/landmarks
```

```bash
ros2 topic echo /teleop_hand_tracking/right/landmarks
```

## Check the calculated hand-angle output

Inside an attached Docker shell:

```bash
ros2 topic echo /hand_angles
```

## Check the NodeMCU topics and nodes

After the NodeMCU connects to the Agent:

```bash
ros2 node list
ros2 topic list
ros2 topic info /hand_angles --verbose
```

## Verify all installed Docker packages

Inside Docker:

```bash
ros2 pkg prefix hand_teleoperation_interfaces
ros2 pkg prefix hand_teleoperation
ros2 pkg prefix micro_ros_msgs
ros2 pkg prefix micro_ros_agent
ros2 pkg executables hand_teleoperation
ros2 pkg executables micro_ros_agent
ros2 interface show hand_teleoperation_interfaces/msg/HandAngles
```

The hand packages should resolve below `/ws/install`. The micro-ROS packages
should resolve below `/opt/micro_ros_humble`.

## Stop the complete setup

Press `Ctrl+C` once in each terminal running a ROS node. Then type `exit` in
the attached shells. Finally, type `exit` in Terminal 1 to stop and remove the
temporary Docker container.
