# ============================================================================
# 3D Frontier Explorer — Dedicated ROS 2 Humble + Gazebo Classic environment
# ============================================================================
# Base: osrf/ros:humble-desktop (ROS 2 Humble + RViz2 + core tools)
# Adds: Gazebo Classic, sjtu_drone (UAV sim), OctoMap stack, PCL
# Pre-builds: sjtu_drone + octomap_mapping into /opt/deps_ws so user workspace
#             /root/ros2_ws stays clean for the user's frontier_explorer_3d code.
# ============================================================================

FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---------- 1. System utilities ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl wget git nano vim htop screen \
        net-tools iputils-ping \
        build-essential cmake \
        python3-pip \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ---------- 2. Gazebo Classic + ROS 2 bridge ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs \
        ros-${ROS_DISTRO}-gazebo-ros2-control \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------- 3. OctoMap + PCL stack ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-octomap \
        ros-${ROS_DISTRO}-octomap-msgs \
        ros-${ROS_DISTRO}-octomap-ros \
        ros-${ROS_DISTRO}-octomap-rviz-plugins \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-perception-pcl \
    && rm -rf /var/lib/apt/lists/*

# ---------- 4. Pre-build dependency workspace (/opt/deps_ws) ----------
#  - sjtu_drone (NovoG93/ros2): kinematic UAV simulator
#  - octomap_mapping (ros2 branch): octomap_server for 3D occupancy mapping
RUN mkdir -p /opt/deps_ws/src
WORKDIR /opt/deps_ws/src

RUN git clone --depth 1 -b ros2 https://github.com/NovoG93/sjtu_drone.git && \
    git clone --depth 1 -b ros2 https://github.com/OctoMap/octomap_mapping.git

WORKDIR /opt/deps_ws

# rosdep + colcon build (parallel-workers 2 for memory safety on Mac/Win Docker)
RUN apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "tf" && \
    colcon build --symlink-install --parallel-workers 2 && \
    rm -rf /var/lib/apt/lists/*

# ---------- 5. User workspace (mounted at runtime) ----------
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# ---------- 6. Auto-source ROS + deps + user workspace ----------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/deps_ws/install/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "defshell -bash" >> /root/.screenrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/deps_ws/install/sjtu_drone_description/share/sjtu_drone_description/models" >> /root/.bashrc

WORKDIR /root/ros2_ws
