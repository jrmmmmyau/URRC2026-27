# Quick run commands

From the workspace root, build once in ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to start navigation_server
```

In each terminal before running commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Odometry + navigation + path following (fake map)

```bash
# Terminal 1
ros2 launch start nav_odom.py

# Terminal 2: type help for map/goal commands
ros2 run navigation_server test_map_publisher
```

## AprilTag detection + localization

Requires the RealSense camera; also starts simulation.

```bash
ros2 launch start apriltag_test.launch.py
```

Run this separately from nav_odom.py: both start simulation. AprilTag pose is not yet connected to navigation.

## Depth perception only

Requires depth images and matching calibration already publishing. The AprilTag test launch disables depth.

```bash
ros2 launch perception depth_observation.launch.py
```

## View node connections

```bash
ros2 run rqt_graph rqt_graph
```

Full ROS build and live execution still need verification on the ROS machine.
