# URRC Autonomy

ROS 2 Jazzy workspace for Melbot simulation, odometry, navigation, and AprilTag testing.

## Build

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to start navigation_server
source install/setup.bash
```

Run these commands again in each new terminal before using ROS packages.

## Run

Start odometry and navigation, then start the interactive test-map publisher in another terminal:

```bash
ros2 launch start nav_odom.py
ros2 run navigation_server test_map_publisher
```

Start the simulation, RealSense color camera, AprilTag perception, and localization:

```bash
ros2 launch start apriltag_test.launch.py
```

Launch terrain with teleop. Choose `world:=1` through `world:=6`, or `world:=large` for the 50 x 50 m test map:

```bash
ros2 launch start terrain_teleop.launch.py world:=large
```

See [RUN.md](RUN.md) for the command list, [map/README.md](map/README.md) for map tests, and [src/perception/README.md](src/perception/README.md) for standalone perception commands.
