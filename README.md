# 3D Frontier Explorer — Development Environment

ROS 2 Humble + Gazebo Classic + sjtu_drone + OctoMap, packaged as a single Docker environment for the **3D Frontier Explorer** project (disaster-response UAV exploration in GPS-denied 3D interiors).

---

## What's inside

| Component | Version | Where |
|---|---|---|
| ROS 2 | Humble | `/opt/ros/humble` |
| Gazebo Classic | 11 | system |
| sjtu_drone (kinematic UAV) | `NovoG93/ros2` branch | `/opt/deps_ws` (pre-built) |
| OctoMap server | `ros2` branch | `/opt/deps_ws` (pre-built) |
| PCL ROS | apt | system |
| Your code | — | `./ros2_ws/src/` (mounted) |
| GUI access | noVNC via browser | http://localhost:8080/vnc.html |

The dependency workspace `/opt/deps_ws` is **pre-built into the Docker image**, so you never have to rebuild sjtu_drone or OctoMap. Your own code lives in `./ros2_ws/src/` on the host and is mounted into the container.

---

## Prerequisites

- **Docker Desktop** with at least:
  - **8 GB RAM** allocated (Settings → Resources → Memory)
  - **4 CPU cores**
  - **20 GB disk space** (the image is ~5 GB)
- A browser for the noVNC GUI

---

## Quick start

### 1. First-time setup (one-time, ~15–25 min)

```bash
cd frontier_explorer_env
docker compose build         # Builds the image: installs apt deps + compiles sjtu_drone + octomap
```

### 2. Start the environment

```bash
docker compose up            # Starts ros + novnc containers
```

Leave this terminal running. Open a browser to:

> **http://localhost:8080/vnc.html** → click **Connect**

You'll see a Linux desktop. This is where Gazebo and RViz will appear.

### 3. Run sjtu_drone (smoke test)

Open a **new terminal** on the host:

```bash
cd frontier_explorer_env
docker compose exec ros bash
```

Inside the container:

```bash
# Environment is already auto-sourced via .bashrc, but you can verify:
ros2 pkg list | grep sjtu          # should show 3 sjtu_drone_* packages
ros2 pkg list | grep octomap       # should show octomap_server, octomap_msgs, ...

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
```

### 4. Shutdown

```bash
# Ctrl+C in any docker compose exec terminal to stop the running ROS node
# Then exit the container shells

# Finally, in the docker compose up terminal:
# Ctrl+C, then:
docker compose down
```

---

## Your workspace layout

```
frontier_explorer_env/
├── Dockerfile               # Image recipe (don't modify unless adding global deps)
├── docker-compose.yml       # Services: ros + novnc
├── ros.env                  # ROS_DOMAIN_ID, RMW, etc.
├── novnc.env                # Display size
├── README.md                # This file
└── ros2_ws/
    └── src/                 # ← Your packages go here
        └── (frontier_explorer_3d/ to be created)
```

When you create your own ROS 2 package inside `ros2_ws/src/`, build it from inside the container:

```bash
docker compose exec ros bash
cd /root/ros2_ws
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

---

## Adding new system-level dependencies

If you need to apt-install something globally (e.g. a new RViz plugin), add it to the relevant `apt-get install` block in the `Dockerfile`, then:

```bash
docker compose build         # Rebuild image
docker compose up            # Restart
```

For just adding new ROS source packages, prefer putting them in `ros2_ws/src/` rather than rebuilding the image.

---

## Troubleshooting

**"Cannot connect to display"**
- Make sure both containers are up: `docker compose ps` (you should see `ros` and `novnc` both running).
- Check the browser connected: http://localhost:8080/vnc.html → Connect.

**Gazebo is black / very slow**
- Confirm Docker Desktop has ≥8 GB RAM and ≥4 cores.
- `LIBGL_ALWAYS_SOFTWARE=1` is already set — Gazebo runs on software rendering. This is unavoidable on Mac/Win Docker but expect 20–30 FPS, not 60.

**Apple Silicon (M1/M2/M3) Mac**
- The image is built for `linux/amd64` via Docker Desktop's Rosetta emulation. Performance is reduced (~50–70%) but works. Make sure "Use Rosetta for x86/amd64 emulation" is enabled in Docker Desktop → Settings → General.

**Build hangs at colcon step**
- Out of memory. Increase Docker Desktop memory and re-run `docker compose build`.

---

## Next steps (project roadmap)

1. ✅ Environment up (this README)
2. ⬜ Add a Velodyne VLP-16-style 3D LiDAR sensor to the sjtu_drone URDF
3. ⬜ Connect the LiDAR PointCloud2 topic to `octomap_server` → visualize 3D occupancy in RViz
4. ⬜ Create `frontier_explorer_3d` package with:
   - Incremental frontier extraction node (subscribes to `/octomap_full`)
   - Information-gain NBV selector (raycast through octree)
   - 3D RRT* planner
5. ⬜ Design a "collapsed building" Gazebo world for evaluation
