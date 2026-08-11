FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    libmsgpack-dev \
    libompl-dev \
    v4l-utils \
    usbutils \
    iputils-ping \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgtk-3-0 \
    curl \
    ca-certificates \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-camera-ros \
    ros-humble-cv-bridge \
    ros-humble-geometric-shapes \
    ros-humble-joint-state-publisher-gui \
    ros-humble-image-transport \
    ros-humble-joy \
    ros-humble-moveit \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-msgs \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-servo \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realsense2-camera \
    ros-humble-tf2-ros \
    ros-humble-topic-based-ros2-control \
    ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

# Install uv for fast python package management
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# Install Python dependencies for the ROS side (Python 3.10)
# Note: lerobot itself runs on the host in a 3.12 venv.
RUN uv pip install --system --no-cache \
    "numpy<2" \
    "mediapipe==0.10.8" \
    opencv-python \
    pyassimp \
    pymodbus \
    PyYAML \
    scipy \
    transforms3d \
    xarm-python-sdk \
    "vuer[webrtc]==0.1.6"

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
 && rosdep update

# Build a Humble-compatible micro-ROS Agent in an image-owned prefix. Keeping
# it outside /ws prevents a workspace --clean from removing it and avoids
# accidentally compiling the Jazzy/Rolling Agent against Humble's Fast-CDR 1.x.
ARG MICRO_ROS_AGENT_COMMIT=4f363a79ae96aed39dcc5269f53f73f6a70d3a2c
ARG MICRO_ROS_MSGS_COMMIT=100bf269e78da3fe0a58f6531ebe47d6991bd9ab

RUN mkdir -p /opt/micro_ros_ws/src/micro-ROS-Agent /opt/micro_ros_ws/src/micro_ros_msgs \
 && git -C /opt/micro_ros_ws/src/micro-ROS-Agent init \
 && git -C /opt/micro_ros_ws/src/micro-ROS-Agent remote add origin https://github.com/micro-ROS/micro-ROS-Agent.git \
 && git -C /opt/micro_ros_ws/src/micro-ROS-Agent fetch --depth 1 origin "${MICRO_ROS_AGENT_COMMIT}" \
 && git -C /opt/micro_ros_ws/src/micro-ROS-Agent checkout --detach FETCH_HEAD \
 && git -C /opt/micro_ros_ws/src/micro_ros_msgs init \
 && git -C /opt/micro_ros_ws/src/micro_ros_msgs remote add origin https://github.com/micro-ROS/micro_ros_msgs.git \
 && git -C /opt/micro_ros_ws/src/micro_ros_msgs fetch --depth 1 origin "${MICRO_ROS_MSGS_COMMIT}" \
 && git -C /opt/micro_ros_ws/src/micro_ros_msgs checkout --detach FETCH_HEAD

WORKDIR /opt/micro_ros_ws
RUN source /opt/ros/humble/setup.bash \
 && colcon build \
      --merge-install \
      --install-base /opt/micro_ros_humble \
      --executor sequential \
      --event-handlers console_direct+ \
      --packages-up-to micro_ros_agent \
 && source /opt/micro_ros_humble/setup.bash \
 && ros2 pkg prefix micro_ros_msgs \
 && ros2 pkg prefix micro_ros_agent \
 && ros2 pkg executables micro_ros_agent | grep -Fq "micro_ros_agent micro_ros_agent" \
 && rm -rf /opt/micro_ros_ws/build /opt/micro_ros_ws/log /opt/micro_ros_ws/src

WORKDIR /ws/src
COPY . /ws/src/agentic_disassembly

WORKDIR /ws
RUN source /opt/ros/humble/setup.bash \
 && package_paths="$(colcon list --base-paths src --packages-up-to arm_teleop nr_dual_arm_moveit_config nr_dual_arm_description ros_tcp_endpoint exotica_ik_solver exotica_collision_scene_fcl_latest exotica_core_task_maps --paths-only)" \
 && rosdep install --from-paths ${package_paths} --ignore-src -r -y --rosdistro humble --skip-keys "opencv-python python3-pyassimp ros-humble-ompl ompl pinocchio" \
 && colcon build --packages-up-to arm_teleop nr_dual_arm_moveit_config nr_dual_arm_description ros_tcp_endpoint exotica_ik_solver exotica_collision_scene_fcl_latest exotica_core_task_maps

COPY docker/entrypoint.sh /ros_entrypoint_local.sh
RUN chmod +x /ros_entrypoint_local.sh

ENTRYPOINT ["/ros_entrypoint_local.sh"]
CMD ["bash"]
