# 3D Frontier Explorer — Development Environment

ROS 2 Humble + Gazebo Classic + sjtu_drone + OctoMap, packaged as a single Docker environment for the **3D Frontier Explorer** project (disaster-response UAV exploration in GPS-denied 3D interiors).

---

## Quick start

### 1. First-time setup (one-time, ~15–25 min)

```bash
cd frontier_explorer_env
docker compose build
```

### 2. Start the environment

```bash
docker compose up
```

Leave this terminal running. Open a browser to:

> **http://localhost:8080/vnc.html** → click **Connect**

You'll see a Linux desktop. This is where Gazebo and RViz will appear.

### 3. Run sjtu_drone

Open a **new terminal** on the host:

```bash
cd frontier_explorer_env
docker compose exec ros bash
```

Inside the container:

```bash
# Apply patch for the sjtu
./frontier_setup.sh
# Launch the simulator
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

You should see Gazebo with a quadrotor appear in your browser tab.

In a **third terminal** (also `docker compose exec ros bash`):

```bash
# Takeoff
ros2 topic pub /drone/takeoff std_msgs/msg/Empty "{}" --once

# List topics to confirm sensors are publishing
ros2 topic list

ros2 run tf2_ros static_transform_publisher \
  0 0 0.10 0 0 0 simple_drone/base_footprint velodyne_link
```
---

### 4. Add collision avoidance (frontier_safety)

In a new terminal (`docker compose exec ros bash`):
```bash
# Generate + build the frontier_safety package
./collision_setup.sh
```
This writes and builds the `frontier_safety` package (LiDAR-based collision
avoidance with a gaussian-smoothed obstacle histogram). Safe to re-run.

Launch the node (with the simulator already running):
```bash
source /root/ros2_ws/install/setup.bash
ros2 launch frontier_safety collision_avoidance.launch.py
```



## Next steps (project roadmap)

1. ✅ Environment up (this README)
2. ⬜ Add a Velodyne VLP-16-style 3D LiDAR sensor to the sjtu_drone URDF
3. ⬜ Connect the LiDAR PointCloud2 topic to `octomap_server` → visualize 3D occupancy in RViz
4. ⬜ Create `frontier_explorer_3d` package with:
   - Incremental frontier extraction node (subscribes to `/octomap_full`)
   - Information-gain NBV selector (raycast through octree)
   - 3D RRT* planner
5. ⬜ Design a "collapsed building" Gazebo world for evaluation
