# URRC 2026-27 Autonomy

University of Rochester Robotics Club — Lunabotics autonomy stack.

See [RUN.md](RUN.md) for quick launch commands.

## What's in here

- `src/description/` — robot URDF and Gazebo model plugins
- `src/driver/` — swerve-drive command and feedback node
- `src/perception/` — image rectification, AprilTag detection, robot-relative tag observations, and depth observations
- `src/apriltag_localization/` — robot pose estimation from tag observations and known field tag locations
- `src/navigation_server/` — existing navigation and authored test-map tools
- `src/start/` — launch files for starting simulation and the robot
- `config/` — future parameter configs (Nav2, EKF, SLAM, etc.)

## Setup

1. Ubuntu 24.04 (VM or native)
2. ROS 2 Jazzy (`ros-jazzy-desktop`)

## AprilTag pipeline

The pipeline is split into perception and localization. Perception starts the image rectifier, AprilTag detector, and tag observer. The observer combines `/detections` with TF and publishes `/perception/tag/observations`. Localization consumes those prepared robot-relative observations and known field tag locations to publish `/tag_pose`.

With the declared ROS, Gazebo, and RealSense dependencies installed, build and run the existing combined test setup from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to start
source install/setup.bash
ros2 launch start apriltag_test.launch.py
```

This entry point starts the simulation, the real camera's color stream, perception, and localization. It retains the existing camera settings and requires the RealSense camera. The simulation and real-camera time configuration still needs runtime validation.

To run the processing components independently, start these commands in separate terminals after sourcing the ROS and workspace environments:

```bash
ros2 launch perception apriltag_detection.launch.py
ros2 launch perception tag_observation.launch.py
ros2 launch apriltag_localization apriltag.launch.py
```

The independent launches require an existing camera input and the mounting TF needed by the perception tag observer. Choose either the combined entry point or the separate launches to avoid duplicate nodes.

**Launch behavior change:** `ros2 launch apriltag_localization apriltag.launch.py` now starts localization only. Use the `start` command above for the combined test setup.

Detector settings are in `src/perception/config/apriltag.yaml`; tag-frame naming, output frame, and TF timeout belong in `src/perception/config/tag_observer.yaml`. Known field tag positions and localization settings remain in `src/apriltag_localization/config/localization.yaml`. The detector output and `/tag_pose` are preserved; localization now receives the new observation topic. It depends on perception's public message definitions, with no dependency on its private C++ implementation.

Both observer and localization launches accept `base_frame`, `observations_topic`, and `use_sim_time`. Set them consistently; the combined launch forwards the same `base_frame` and `observations_topic` to both. Independent executable users must supply the equivalent topic remapping.


See the [perception README](src/perception/README.md) for input topics, TF responsibilities, and validation limits.

## Standalone depth perception

Perception converts depth images into camera-coordinate observation points. The default output is `/perception/depth/points` (`sensor_msgs/msg/PointCloud2`), with FLOAT64 XYZ coordinates in meters and the image's timestamp and optical frame.

```bash
colcon build --base-paths src --packages-select perception
source install/setup.bash
ros2 launch perception depth_observation.launch.py
```

The camera must already publish depth images and matching calibration. Topic arguments are documented in the [perception README](src/perception/README.md). The existing AprilTag test setup still disables depth and does not start this node.

The imported mapping package and the subsequently added depth world-coordinate mode have been removed. Depth perception no longer receives `/tag_pose`, interpolates robot poses, or reads mounting TF. AprilTag perception still uses its existing tag/mounting TF relationships, and localization still publishes `/tag_pose`.

## Navigation with the existing test map

There is no local or global mapping implementation supplied by this separation work. Navigation's existing `test_map_publisher` loads authored maps and publishes `/map`; perception does not update it. Keep the current fake-map workflow. After building and sourcing navigation in the target ROS environment, its existing instructions use separate terminals:

```bash
ros2 launch navigation_server nav_test.launch.py
ros2 run navigation_server test_map_publisher
```

The publisher accepts interactive input; type `help` for its commands. Navigation source, maps, settings, and launch files are unchanged. These commands describe the existing workflow, not a new runtime certification.

## Validation status

The optional synthetic test harnesses were removed at the user's request. Full ROS compilation, message generation, launch, and live camera checks remain pending. See [RUN.md](RUN.md) for commands and the [updated scope and plan](docs/perception_separation_plan.md) for task boundaries.
