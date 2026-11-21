# Perception Run Commands

Build and source from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to perception apriltag_localization start
source install/setup.bash
```

Run the combined AprilTag test (requires RealSense):

```bash
ros2 launch start apriltag_test.launch.py
```

Run depth observations with an existing depth camera:

```bash
ros2 launch perception depth_observation.launch.py
```

Navigation uses its separate authored test map; see [RUN.md](../RUN.md).
