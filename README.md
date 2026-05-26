# RP-Project-Comito

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

## Run The Single-Window Demo

```bash
ros2 launch rp_simple_rviz demo.launch.py
```

This starts:

- `map_server_node`, which publishes `/map`
- `demo_robot_node`, which publishes TF, `/laser_1/scan`, and `/particle_cloud`
- `simple_rviz_node`, the only node that opens a window

Controls in the viewer window:

- `i`: drag to publish `/initialpose`
- `g`: drag to publish `/move_base/goal`
- `w`, `a`, `s`, `d`: send short `/robot_1/cmd_vel` pulses
- `x`: stop
- `Esc`: close

## Run Against Another ROS Graph

```bash
ros2 launch rp_simple_rviz simple_rviz.launch.py
```

Default topics and frames are in `config/simple_rviz.yaml`.

The course `rp_simulator` can still be launched with:

```bash
ros2 launch rp_simple_rviz with_simulator.launch.py
```

That simulator opens its own canvas, so use `demo.launch.py` when you want only
the Simple RVIZ window.
