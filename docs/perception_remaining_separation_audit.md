# AprilTag Test Commands

Build and source from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to start
source install/setup.bash
```

Run the combined simulation, RealSense color camera, AprilTag detector, tag observer, and localization:

```bash
ros2 launch start apriltag_test.launch.py
```

The RealSense camera is required. For navigation commands, see [RUN.md](../RUN.md).
