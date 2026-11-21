# Run Commands

From the repository root, build once:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to start navigation_server
source install/setup.bash
```

Source ROS and the workspace in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Odometry + navigation

Use separate terminals. The map publisher accepts interactive commands; type `help` after it starts.

```bash
ros2 launch start nav_odom.py
ros2 run navigation_server test_map_publisher
```

## AprilTag test

Requires the RealSense camera. This launch also starts simulation.

```bash
ros2 launch start apriltag_test.launch.py
```

## Gazebo terrain + teleop

Choose `1` to `6` or `large`:

```bash
ros2 launch start terrain_teleop.launch.py world:=large
```

## Standalone depth observations

Requires a camera publishing depth and camera-info topics:

```bash
ros2 launch perception depth_observation.launch.py
```

Full ROS/Gazebo execution must be checked on the Ubuntu Jazzy machine.
