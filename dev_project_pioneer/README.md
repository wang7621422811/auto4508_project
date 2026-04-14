# Pioneer 3-AT Autonomous Navigation — ROS 2 / Gazebo

A ROS 2 (Jazzy) + Gazebo Harmonic simulation of a **Pioneer 3-AT** robot that autonomously navigates a scaled James Oval environment, avoids static obstacles, builds a map with SLAM, and records the driven path.

---

## Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic (`ros_gz_sim`) |
| Nav2 + SLAM Toolbox | via `apt` (see below) |

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

---

## Quick Start

```bash
cd ~/auto4508/dev_project_pioneer
source /opt/ros/jazzy/setup.bash
colcon build --packages-select p3at
source install/setup.bash

# Full autonomous navigation stack
ros2 launch p3at navigation.launch.py
```

For a minimal simulation with teleop only:

```bash
ros2 launch p3at sdf.launch.py rviz:=false
# In a new terminal (after sourcing):
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

Keyboard controls: `i` forward, `,` reverse, `j`/`l` turn, `k` stop, `q`/`z` adjust linear speed, `w`/`x` adjust angular speed.

---

## Project Structure

```
src/p3at/
├── robots/
│   └── pioneer.urdf              # Robot description (chassis, wheels, sensors)
├── worlds/
│   └── basic_urdf.sdf            # James Oval simulation world (1/10 scale)
├── config/
│   ├── nav2_params.yaml          # Nav2 stack parameters
│   └── slam_params.yaml          # slam_toolbox online async parameters
├── launch/
│   ├── sdf.launch.py             # Gazebo + bridge + optional RViz/rqt
│   └── navigation.launch.py     # Full stack: Gazebo + SLAM + Nav2 + custom controller
└── p3at/
    ├── waypoint_controller.py    # Custom 3-phase proportional controller
    └── path_recorder.py          # TF-based path recording node
```

---

## Simulation World

The world models **James Oval** at **1/10 scale** (~15 m × 12.5 m). Obstacles (traffic cones and buckets) use **real-world geometry** to ensure realistic lidar interactions:

| Model prefix | Description |
|---|---|
| `ground_plane` | Green grass plane |
| `oval_fence_*` | Segmented elliptical boundary walls |
| `cone_*` | Orange traffic cones (r=0.18 m, h=0.32 m) |
| `bucket_*` | Cylindrical buckets (r=0.22 m, h=0.38 m) |
| `pioneer` | Robot spawn point (west side of oval) |

---

## Sensors

All sensors are configured with realistic (non-idealised) parameters:

| Sensor | ROS 2 Topic | Key Limitations |
|---|---|---|
| 2D LiDAR (`gpu_lidar`) | `/scan` | 240° FOV, 720 rays, 12 m max range, 10 Hz, Gaussian noise (σ≈0.015 m) |
| IMU | `/imu` | 100 Hz, Gaussian noise on angular velocity and linear acceleration |
| RGB Camera | `/camera` + `/camera_info` | 640×480, 20 Hz, ~62° horizontal FOV, image noise |
| Wheel Odometry | `/odom` | Slip and integration drift from Gazebo differential drive model |

---

## Autonomous Navigation Stack

```
Gazebo Sim
    │  /scan, /odom, /imu, /tf  (ros_gz_bridge)
    ▼
slam_toolbox (online async) ──► /map, map→odom TF
    │
    ▼
Nav2 Stack
  bt_navigator → planner_server (NavFn)
               → controller_server (DWB)
               → behavior_server
  costmap_2d (global + local, from /scan)
  velocity_smoother → /cmd_vel
```

Key Nav2 parameters:

| Parameter | Value | Rationale |
|---|---|---|
| `max_vel_x` | 0.4 m/s | Safety margin below Pioneer's 0.7 m/s max |
| `max_vel_theta` | 1.5 rad/s | Reasonable differential-drive turn rate |
| `robot_radius` | 0.25 m | ~Half of Pioneer chassis width |
| `inflation_radius` | 0.55 m | 0.3 m clearance beyond robot radius |
| `slam resolution` | 0.05 m | 5 cm grid — sufficient to resolve cones (∅≈0.36 m) |

---

## Custom Waypoint Controller

A self-contained **rotate–drive–rotate** proportional controller (`waypoint_controller.py`) that does **not** rely on Nav2's DWB plugin.

### Control phases

```
Phase 1 — Rotate-to-Goal     Spin in place until heading error < heading_tol
Phase 2 — Drive              Drive forward with proportional heading correction
                             (falls back to Phase 1 if heading error > 0.8 rad)
Phase 3 — Rotate-to-Heading  Spin in place to match target orientation
```

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `kp_linear` | 0.6 | Linear speed proportional gain |
| `kp_angular` | 2.0 | Angular speed proportional gain |
| `max_linear` | 0.35 m/s | Linear velocity cap |
| `max_angular` | 1.2 rad/s | Angular velocity cap |
| `position_tol` | 0.15 m | Position arrival threshold |
| `heading_tol` | 0.10 rad | Heading alignment threshold |

### Interfaces

| Topic / Service | Type | Direction | Description |
|---|---|---|---|
| `/goal_pose` | `PoseStamped` | Subscribe | Appends one waypoint (RViz 2D Goal Pose) |
| `/waypoints` | `nav_msgs/Path` | Subscribe | Replaces entire waypoint queue at once |
| `/clear_waypoints` | `std_srvs/Empty` | Service | Clears queue and stops robot immediately |
| `/waypoint_path` | `nav_msgs/Path` | Publish | Remaining waypoints (visualise in RViz) |
| `/cmd_vel` | `Twist` | Publish | Velocity commands |

### Sending waypoints

**Single goal (RViz):** Use the **2D Goal Pose** tool — each click appends to the queue.

**Full path (CLI):**

```bash
ros2 topic pub --once /waypoints nav_msgs/msg/Path "{
  header: {frame_id: 'map'},
  poses: [
    {pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}},
    {pose: {position: {x: 2.0, y: 2.0}, orientation: {z: 0.707, w: 0.707}}},
    {pose: {position: {x: 0.0, y: 2.0}, orientation: {z: 1.0, w: 0.0}}},
    {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}
  ]
}"
```

**Emergency stop:**

```bash
ros2 service call /clear_waypoints std_srvs/srv/Empty
```

---

## Obstacle Avoidance

The controller implements **reactive obstacle avoidance** using `/scan` during the DRIVE phase. The front ±40° cone is split into left/right halves:

| Condition | Behaviour |
|---|---|
| `min_front ≥ 0.7 m` | Normal proportional drive |
| `0.35 m < min_front < 0.7 m` | Reduce speed linearly + steer away from closer side |
| `min_front ≤ 0.35 m` | Full stop + max angular rate away from obstacle |

This handles **previously unseen** static obstacles that are not yet in the costmap.

---

## Map Building & Path Recording

Both features run automatically within `navigation.launch.py`.

### Save map and path

```bash
ros2 service call /path_recorder/save std_srvs/srv/Empty
```

Output files are written to `~/map_data/`:

```
~/map_data/
├── map_<timestamp>.pgm       # Occupancy grid image (white=free, black=obstacle, grey=unknown)
├── map_<timestamp>.yaml      # Map metadata (resolution, origin)
└── path_<timestamp>.csv      # Driven path in map frame
```

### CSV format

```csv
stamp_sec,stamp_nanosec,x,y,yaw_rad
125,400000000,-4.9800,0.0100,-0.0032
```

### Plot path in Python

```python
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv('~/map_data/path_<timestamp>.csv')
plt.plot(df['x'], df['y'], 'r-', linewidth=1)
plt.axis('equal'); plt.xlabel('x (m)'); plt.ylabel('y (m)')
plt.title('Driven Path'); plt.savefig('driven_path.png')
```

**RViz:** Add a **Path** display on `/driven_path` to see the trajectory live.

---

## Useful Diagnostic Commands

```bash
# Simulation clock
ros2 topic hz /clock

# TF chain
ros2 run tf2_ros tf2_echo map base_link

# SLAM lifecycle state (should be active)
ros2 lifecycle get /slam_toolbox

# Nav2 lifecycle states
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor

# Velocity pipeline
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel

# Sensor topics
ros2 topic list | grep -E 'scan|imu|camera|odom'
ros2 topic echo /imu --once
```

---

## Known Issues & Fixes

| # | Issue | Root Cause | Fix |
|---|---|---|---|
| 1 | No sensor data | Gazebo not auto-playing | Add `-r` to `gz_args` in launch |
| 2 | TF tree broken | DiffDrive adds model name prefix to frame IDs | Set `<child_frame_id>base_link</child_frame_id>` and `<tf_topic>/tf</tf_topic>` in URDF |
| 3 | SLAM not publishing `/map` | Lifecycle node never activated | Use `slam_toolbox`'s official `online_async_launch.py` |
| 4 | ROS Time stuck at 0 | `/clock` not bridged to ROS 2 | Add `'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'` to bridge |
| 5 | Map only shows black/grey, no free space | Lidar range < field half-width; `min_pass_through` too strict | Increase lidar `max` to 12 m; set `min_pass_through: 1` |
| 6 | Nav2 ignores 2D Goal Pose | Missing config for Jazzy-new nodes (`collision_monitor` etc.) | Add full config for `collision_monitor`, `docking_server`, `route_server` in `nav2_params.yaml` |
