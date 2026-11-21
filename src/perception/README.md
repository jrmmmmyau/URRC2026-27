# Perception Commands

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to perception apriltag_localization start
source install/setup.bash
```

## AprilTag pipeline

Combined simulation, RealSense color camera, detector, tag observer, and localization (requires the RealSense camera):

```bash
ros2 launch start apriltag_test.launch.py
```

For an already-running camera and TF tree, start these separately in sourced terminals:

```bash
ros2 launch perception apriltag_detection.launch.py
ros2 launch perception tag_observation.launch.py
ros2 launch apriltag_localization apriltag.launch.py
```
