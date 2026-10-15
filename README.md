# Bug2 Navigation and Laser Scan Perception

This ROS 1 project contains two components for a simulated Husky robot: Bug2-style goal seeking with obstacle avoidance, and line extraction from laser scans using RANSAC.

## Contents

- `lab3/src/bug2.py` subscribes to front laser scans and odometry, then publishes velocity commands to `/husky_velocity_controller/cmd_vel`.
- `lab3/src/ransac.py` converts laser ranges into planar points and publishes visualization markers.
- `lab3/launch/bug2.launch` starts the navigation node with initial and goal coordinates.
- `lab3/launch/perception.launch` starts the RANSAC node and RViz with `lab3/rviz/lab3.rviz`.

## Requirements

ROS 1 with catkin, `rospy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `tf`, NumPy, and a Husky simulation that publishes the expected scan and odometry topics. The package manifest does not list every Python import used by the scripts, so install/check runtime dependencies in the target ROS environment.

## Build and launch

Place `lab3` in a catkin workspace's `src/` directory, then:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Start a compatible Husky simulation first. In another sourced terminal, run either:

```bash
roslaunch lab3 bug2.launch
```

or

```bash
roslaunch lab3 perception.launch
```

The Bug2 launch file sets the start and goal parameters to `(0, 0)` and `(4.5, 9.0)` respectively. Update them in the launch file for a different simulation setup.

## Notes

The launch and source files are coursework code and have not been validated against a current ROS distribution. Check topic names, executable permissions, and scan geometry against your simulator before use. The RANSAC node contains implementation issues that may prevent it from running as written; inspect and repair it before relying on its output.
