# 3D Frontier Explorer — Development Environment

ROS 2 Humble + Gazebo Classic + sjtu_drone + OctoMap, packaged as a single Docker environment for the **3D Frontier Explorer** project (disaster-response UAV exploration in GPS-denied 3D interiors).

---

## Quick start

### 1. First-time setup (one-time, ~15–25 min)

```bash
docker compose build
```

### 2. Start the environment

```bash
docker compose up
```

Leave this terminal running. Open a browser to:

> **http://localhost:8080/vnc.html**

You will see a Linux desktop. Gazebo and RViz2 will appear here.

---

### 3. Apply Velodyne patch and build (first time only, ~5 min)

Open a container shell:

```bash
docker compose exec ros bash
```

Inside the container:
```bash
cd /root/ros2_ws/src
./frontier_setup.sh
```

This clones dependencies, patches the sjtu_drone URDF to add a VLP-16 LiDAR, and builds all packages. Safe to re-run if the container is recreated.

---

### 4. Launch the simulator

In a container shell (`docker compose exec ros bash`):

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

Gazebo and RViz2 should appear in the browser tab.

---

### 5. Takeoff + static TF

In a new container shell:

```bash
source /root/ros2_ws/install/setup.bash

# Takeoff (You can directly enter the command to takeoff)
# ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty "{}" --once

# Publish the velodyne_link → base_footprint static transform
ros2 run tf2_ros static_transform_publisher \
  0 0 0.10 0 0 0 simple_drone/base_footprint velodyne_link
```

---
### 6. Run octomap_server

In a new container shell:

```bash
source /root/ros2_ws/install/setup.bash
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/simple_drone/velodyne_points \
  -p frame_id:=simple_drone/odom \
  -p resolution:=0.1 \
  -p sensor_model.max_range:=25.0 \
  -p use_sim_time:=true \
  -p publish_free_space:=true
```

> `publish_free_space:=true` is required for the frontier extractor.

---

### 7. Add collision avoidance (frontier_safety)

In a new container shell:

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch frontier_safety collision_avoidance.launch.py
```

If `frontier_safety` was not built by `frontier_setup.sh`, build it first:

```bash
cd /root/ros2_ws
colcon build --packages-select frontier_safety --symlink-install
source install/setup.bash
```

---

### 8. Run frontier extractor

Build once (or after any source change):

```bash
cd /root/ros2_ws
colcon build --packages-select frontier_explorer_py --symlink-install
source install/setup.bash
```

Launch:

```bash
ros2 launch frontier_explorer_py frontier_extractor.launch.py
```

---

### 9. Visualize in RViz2

In RViz2 (visible in the VNC browser tab):

1. Set **Global Options → Fixed Frame** to `simple_drone/odom`
2. Add displays:

| Display type | Topic | Notes |
|---|---|---|
| OctoMap | `/octomap_full` | requires `octomap_rviz_plugins`; shows 3D occupancy |
| PointCloud2 | `/frontier_extractor/frontier_cloud` | colour by `cluster_id` field |
| MarkerArray | `/frontier_extractor/cluster_markers` | coloured spheres at cluster centroids |
| PoseArray | `/frontier_extractor/cluster_centroids` | viewpoint candidates for NBV |

Monitor frontier counts:

```bash
ros2 topic echo /frontier_extractor/status
# {"num_frontiers": N, "num_clusters": K}
```

---

### 10. Run frontier_explorer_3d

```bash
cd /root/ros2_ws/src
./build_explorer.sh
```

Launch:

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch frontier_explorer_py frontier_extractor.launch.py
```

## Next steps (project roadmap)

1. ✅ Environment up
2. ✅ Velodyne VLP-16 LiDAR added to sjtu_drone URDF
3. ✅ LiDAR → `octomap_server` → 3D occupancy in RViz2
4. ✅ `frontier_explorer_py`: incremental frontier extraction + clustering
5. ✅ `frontier_nbv`: information-gain next-best-view selector
6. ✅ `frontier_rrt`: RRT* collision-free path planner
7. ⬜ Collapsed-building Gazebo world for evaluation
