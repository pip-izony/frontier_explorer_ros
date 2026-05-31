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

# ---------- 4. rosdep reset ----------
RUN rosdep update --rosdistro=${ROS_DISTRO} || true

# ---------- 5. User workspace ----------
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# ---------- 6. Auto-source ----------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc && \
    echo "defshell -bash" >> /root/.screenrc

WORKDIR /root/ros2_ws
