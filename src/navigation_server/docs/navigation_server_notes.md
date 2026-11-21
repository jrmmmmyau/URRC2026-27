# Navigation Commands

Build and source from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to navigation_server start
source install/setup.bash
```

Run in separate terminals. The map publisher is interactive; type `help` for map and goal commands:

```bash
ros2 launch start nav_odom.py
ros2 run navigation_server test_map_publisher
```

Navigation currently uses authored test maps; the perception package does not create or update `/map`.
