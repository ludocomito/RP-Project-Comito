# RP-Project-Comito

This project was realized for the exam of Robot Programming A.Y. 2025/26. It implements a ROS 2 based viewer that emulates RViz capablilities.

## Simple RVIZ

ROS 2 Humble package for a small RViz-like 2D viewer.

The viewer shows:

- `/map` as an occupancy grid
- laser scans
- mobile bases as circles, using TF
- localization particles when a `PoseArray` is published

The viewer publishes:

- `/initialpose` as `geometry_msgs/PoseWithCovarianceStamped`
- `/move_base/goal` as `geometry_msgs/PoseStamped`

## Build

Run from this repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

## Run The Demo

```bash
ros2 launch rp_simple_rviz demo.launch.py
```

This starts:

- `map_server_node`, which publishes `/map`
- `demo_robot_node`, which publishes TF, `/laser_1/scan`, and `/particle_cloud`
- `simple_rviz_node`, the only node that opens a window

Controls in the viewer window:

- `i`: click to publish `/initialpose`
- `g`: click to publish `/move_base/goal`
- `w`, `a`, `s`, `d`: send short `/robot_1/cmd_vel` pulses
- `x`: stop
- `Esc`: close

<!--
## Run Against Another ROS Graph

```bash
ros2 launch rp_simple_rviz simple_rviz.launch.py

