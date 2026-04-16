# Pioneer P3-AT Part 2 — Autonomous Mission

Extends the Part 1 simulation with a full autonomous mission:
GPS waypoint navigation, cone photography, colored object detection,
slalom weaving, gamepad control, and journey summary.

---

## Prerequisites

Same as Part 1 (ROS 2 Jazzy, Gazebo Harmonic, Nav2, SLAM Toolbox) **plus**:

```bash
sudo apt install ros-jazzy-joy ros-jazzy-cv-bridge python3-opencv
```

Part 1 must be built first (the `p3at` package provides the robot URDF, world
fence segments, and Nav2/SLAM configs):

```bash
cd ~/auto4508/dev_project_pioneer
source /opt/ros/jazzy/setup.bash
colcon build --packages-select p3at
source install/setup.bash
```

---

## Build

```bash
cd ~/auto4508/dev_pioneer_part_2
source /opt/ros/jazzy/setup.bash
# Part 1 must be on the path so pioneer_part2 can find p3at at launch time
source ~/auto4508/dev_project_pioneer/install/setup.bash
colcon build --packages-select pioneer_part2
source install/setup.bash
```

---

## Quick Start

```bash
# Source both workspaces
source /opt/ros/jazzy/setup.bash
source ~/auto4508/dev_project_pioneer/install/setup.bash
source ~/auto4508/dev_pioneer_part_2/install/setup.bash

# Launch full mission stack
ros2 launch pioneer_part2 mission.launch.py
```

After launch (~8 s boot):
1. Press **X** on the gamepad to enable automated mode.
2. Hold **L2 + R2** (dead-man switch) — robot starts navigating.
3. Robot visits WP1 → weaves through cones → WP2 → WP3 → WP4 → returns to start.
4. At each waypoint: photo saved + coloured object detected.
5. Journey summary printed to terminal on completion.

Press **O** at any time to switch to manual mode and drive with the left stick.

---

## Manual testing without a gamepad

```bash
# Publish "auto" mode to skip the gamepad requirement:
ros2 topic pub --once /gamepad_mode std_msgs/msg/String '{data: "auto"}'

# Manually trigger photo + detection at any time:
ros2 service call /capture_photo std_srvs/srv/Empty
ros2 service call /detect_object std_srvs/srv/Empty

# Get journey summary:
ros2 service call /path_recorder/journey_summary std_srvs/srv/Empty

# Save map + CSV path:
ros2 service call /path_recorder/save std_srvs/srv/Empty

# Emergency stop:
ros2 service call /clear_waypoints std_srvs/srv/Empty
```

---

## Package Structure

```
dev_pioneer_part_2/src/pioneer_part2/
├── pioneer_part2/
│   ├── mission_controller.py   Top-level state machine (IDLE→NAVIGATE→WEAVE→AT_WP→RETURN→COMPLETE)
│   ├── waypoint_controller.py  Enhanced P-controller (1.5 m tol, pass-on-right, /waypoint_reached)
│   ├── cone_weaver.py          LiDAR-based slalom between WP1 and WP2
│   ├── vision_detector.py      Camera: photo capture, HSV colour + shape detection, distance
│   ├── gamepad_controller.py   Joy: X=auto, O=manual, L2+R2 dead-man, left-stick drive
│   └── path_recorder.py        TF path recording + journey summary service
├── config/
│   ├── waypoints.yaml          4 GPS waypoints + weave cone prior positions
│   └── mission_params.yaml     Per-node ROS parameters
├── launch/
│   └── mission.launch.py       Full stack launch
└── worlds/
    └── mission_world.sdf       Gazebo world: oval fence + 4 waypoint cones +
                                5 weaving cones + 4 coloured shape objects
```

---

## Task Coverage

| Task | Implementation |
|---|---|
| 1. Drive GPS waypoints, return to start | `mission_controller.py` + `waypoint_controller.py` |
| 2. Photo at each waypoint, cone on right | `vision_detector.py` + pass-on-right bias in `waypoint_controller.py` |
| 3. Weave through cones WP1→WP2 | `cone_weaver.py` (LiDAR cluster detection + alternating slalom) |
| 4. Detect colored object shape + distance | `vision_detector.py` (HSV + `approxPolyDP` + LiDAR/pinhole distance) |
| 5. Journey summary | `path_recorder.py` `/journey_summary` + `mission_controller.py` terminal print |
| 6. LiDAR obstacle avoidance | `waypoint_controller.py` (forward cone scan, stop/slow/steer) |
| 7. Bluetooth gamepad | `gamepad_controller.py` (X=auto, O=manual, L2+R2 dead-man) |

---

## Output Files

```
~/mission_photos/
  wp_<n>_<timestamp>.jpg      # one photo per waypoint

~/mission_data/
  path_<timestamp>.csv        # driven path poses
  map_<timestamp>.pgm         # SLAM occupancy grid
  map_<timestamp>.yaml        # map metadata
```
