# ros2-nav2-object-detection

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10-blue) ![License](https://img.shields.io/badge/License-MIT-green)

A ROS 2 (Humble) monorepo implementing autonomous robot navigation in a Gazebo Classic simulation. The system covers robot modelling with URDF/Xacro, SLAM-based mapping, AMCL localization, Nav2 path planning, YOLOv8 object detection, a programmatic navigation goal service, and frontier-based autonomous exploration — developed as a university assignment.

## Repository Structure

```
ros2-nav2-object-detection/
├── robot_navigation/               # Main package: simulation, Nav2, detection, goals
│   ├── config/
│   │   ├── ekf.yaml                # Robot localization (EKF)
│   │   ├── nav2_params.yaml        # Full Nav2 stack configuration
│   │   ├── slam.yaml               # SLAM Toolbox parameters
│   │   ├── navigate_waypoint.yaml  # Predefined navigation waypoints
│   │   ├── joystick.yaml           # Joystick teleop config
│   │   ├── twist_mux.yaml          # Velocity multiplexer config
│   │   └── config.rviz             # RViz layout
│   ├── launch/
│   │   ├── display.launch.py       # Gazebo + RViz + robot spawner
│   │   ├── main.launch.py          # Full simulation with teleop
│   │   ├── map.launch.py           # SLAM mapping mode
│   │   └── amcl.launch.py          # AMCL + Nav2 autonomous navigation
│   ├── urdf/
│   │   ├── bot_description.urdf    # Main robot model
│   │   └── smars.urdf              # Alternative SMARS robot model
│   ├── meshes/                     # Robot visual meshes (.obj / .mtl)
│   ├── models/
│   │   └── best.pt                 # YOLOv8 trained model
│   ├── map/
│   │   └── two_rooms_map.*         # Pre-built map (pgm + yaml)
│   ├── world/
│   │   └── trsa_two_rooms.world    # Gazebo simulation world
│   └── robot_navigation/
│       ├── navigate_server.py      # /start_navigation service server
│       ├── navigate_client.py      # Interactive CLI navigation client
│       └── object_detection.py     # YOLOv8 detection node
│
├── frontier_explorer/              # Autonomous frontier exploration package
│   ├── launch/
│   │   └── explorer.launch.py      # Launches SLAM + explorer node
│   └── frontier_explorer/
│       ├── navigation_explorer.py  # Main frontier exploration node
│       ├── frontier_helper.py      # Frontier detection algorithms
│       └── occupancy_grid.py       # Occupancy grid utilities
│
└── robot_navigation_interfaces/    # Custom ROS 2 service definitions
    └── srv/
        └── SendGoal.srv            # x, y, orientation_z, orientation_w → success, message
```

## System Architecture

```
Gazebo Simulation
      │
      ├── /scan (LiDAR)
      ├── /odom
      ├── /depth_camera/image_raw
      │
      ▼
robot_localization (EKF)  →  /odometry/filtered
      │
      ├── SLAM mode:   slam_toolbox  →  /map
      │
      └── Nav mode:    AMCL  →  localization
                       Nav2  →  path planning + control
                          ▲
                          │ /start_navigation service
                    navigate_server  ←──  navigate_client (CLI)
                          │
                   navigate_to_pose (action)

object_detection  ←  /depth_camera/image_raw
      │
      └──  /camera/object_detection  (annotated feed in RViz)

frontier_explorer  →  detect frontiers  →  navigate_to_pose (action)
      │
      └──  auto_map (saved on completion)
```

## Packages

### `robot_navigation`
The main package. Handles simulation bringup, robot description, Nav2 configuration, object detection, and the navigation goal service.

**Nodes:**
| Node | Description |
|------|-------------|
| `navigate_server` | Provides the `/start_navigation` service; sends goals to Nav2 action server |
| `navigate_client` | Interactive CLI to send predefined or custom goals from `navigate_waypoint.yaml` |
| `object_detection` | Subscribes to the depth camera, runs YOLOv8 inference, publishes annotated images |

**Velocity priority (twist_mux):**
| Source | Topic | Priority |
|--------|-------|----------|
| Nav2 | `/cmd_vel_nav` | 1 (lowest) |
| Keyboard teleop | `/teleop/cmd_vel` | 2 |
| Joystick | `/joy/cmd_vel` | 3 (highest) |

### `frontier_explorer`
Autonomous exploration using frontier detection. The robot identifies boundaries between known free space and unknown space, navigates to them, and repeats until no frontiers remain. Automatically saves the map on completion.

**Key parameters** (top of `navigation_explorer.py`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `OCC_THRESHOLD` | 10 | Max occupancy to consider a cell free |
| `MIN_FRONTIER_SIZE` | 5 | Min frontier cells to be a valid goal |
| `PROGRESS_TIMEOUT` | 5.0s | Time before declaring robot stuck |
| `MIN_PROGRESS` | 0.5m | Minimum movement to count as progress |
| `RECOVERY_RADIUS` | 0.5m | Search radius for recovery goals |

### `robot_navigation_interfaces`
Custom service definition used by `navigate_server` and `navigate_client`.

```
# SendGoal.srv
float64 x
float64 y
float64 orientation_z
float64 orientation_w
---
bool success
string message
```

## Requirements

- ROS 2 Humble
- Ubuntu 22.04
- Gazebo Classic
- Python packages: `ultralytics`, `opencv-python`, `numpy<2`, `transforms3d`

Install dependencies:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-xacro \
                 ros-humble-slam-toolbox \
                 ros-humble-robot-localization \
                 ros-humble-nav2-bringup \
                 ros-humble-twist-mux \
                 ros-humble-teleop-twist-joy \
                 ros-humble-joy

pip install ultralytics "numpy<2"
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_navigation_interfaces robot_navigation frontier_explorer
source install/setup.bash
```

> Build `robot_navigation_interfaces` first as the other packages depend on it.

## Usage

### 1. Launch simulation

```bash
# Source Gazebo first
. /usr/share/gazebo/setup.bash

ros2 launch robot_navigation main.launch.py
```

### 2a. SLAM mapping (manual teleoperation)

```bash
ros2 launch robot_navigation map.launch.py

# In a separate terminal, keyboard teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/teleop/cmd_vel

# Save the map when done:
ros2 run nav2_map_server map_saver_cli -f two_rooms_map
```

### 2b. Autonomous mapping (frontier exploration)

```bash
ros2 launch frontier_explorer explorer.launch.py
```
The explorer will navigate autonomously and save `auto_map` when complete.

### 3. Autonomous navigation (AMCL + Nav2)

```bash
ros2 launch robot_navigation amcl.launch.py
```

### 4. Send navigation goals

```bash
ros2 run robot_navigation navigate_client
```

The interactive CLI lets you select predefined waypoints from `navigate_waypoint.yaml` or define new ones using RViz's **2D Goal Pose** tool.

### 5. Object detection

The `object_detection` node launches automatically with `amcl.launch.py`. View the annotated feed in RViz by adding an Image display on topic `/camera/object_detection`.
