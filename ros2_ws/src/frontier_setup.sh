#!/bin/bash

apt update

cd /root/ros2_ws/src

[ -d sjtu_drone ] || git clone -b ros2 https://github.com/NovoG93/sjtu_drone.git
[ -d octomap_mapping ] || git clone -b ros2 https://github.com/OctoMap/octomap_mapping.git

PATCH_FILE=/root/ros2_ws/src/velodyne.patch
cd sjtu_drone
patch -p2 < "$PATCH_FILE"
cd ..
mv playground.world /root/ros2_ws/src/sjtu_drone/sjtu_drone_description/worlds/playground.world

cd /root/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash

rosdep update --rosdistro=${ROS_DISTRO}
rosdep install --from-paths src --ignore-src -r -y --skip-keys "tf"
colcon build --symlink-install --packages-select sjtu_drone_description sjtu_drone_bringup sjtu_drone_control octomap_server
