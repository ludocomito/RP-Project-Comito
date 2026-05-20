# RP-Project-Comito

Source ros2 Humble:
`source /opt/ros/humble/setup.bash`

## Simple RVIZ
- Write a program to control a mobile robot in ROS and display
simple systems(rviz simple clone).
- The program is able to show
    - a map (received from the map server),
    - Laser scans
    - mobile bases (as a circle). All items are displayed according to their transform.
    - particles of localization
- The program is able to issue
    - issue /initialpose message, to initialize the localizer
    - and /move_base/goal messages to set a planner destination