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

<<<<<<< HEAD
```bash
ros2 launch start terrain_teleop.launch.py world:=1
```

| Number | Terrain |
| --- | --- |
| `1` | Central Ridge |
| `2` | Rolling Dunes |
| `3` | Diagonal Ridge |
| `4` | Twin Mounds |
| `5` | Broad Basin |
| `6` | Cross Slope |

For example, to launch the Broad Basin terrain:

```bash
ros2 launch start terrain_teleop.launch.py world:=5
```

If `world` is omitted, terrain `1` is used:

```bash
ros2 launch start terrain_teleop.launch.py
```

The terrain launcher starts Gazebo, spawns Melbot, starts the ROS–Gazebo bridges, and runs the driver node. It also uses the OGRE 1 renderer to avoid the shader crash seen on some integrated graphics systems.

For a **50 × 50 m testing terrain**:

```bash
ros2 launch start terrain_teleop.launch.py world:=large
```

This expands terrain 1's layout, with four times the vertical relief and larger
rocks and craters. It keeps the same obstacle count spread across the larger
area, so obstacles are sparse. Melbot stays its normal size. The ground and
collision surface share the same scaling; the starting ground is at z=1.60 m.
Regenerate this world after changing terrain 1 with
`python3 map/scripts/generate_large_terrain.py`.

The six terrain variants and the flat Lunabotics arena include the historical
seven-rock line joining the south perimeter to the central column at the
construction-zone entrance. This closes the short route south of the column.
Scattered rocks vary from 30 to 40 cm in local width, and the numbered worlds
have real craters with 30, 40, and 50 cm rim diameters and different depths.

## Launch the flat Lunabotics arena

To launch the simpler flat arena with Melbot and the driver:

```bash
ros2 launch start teleop.launch.py
```

This launch file uses `map/worlds/lunabotics_arena.sdf`.
The flat arena has varied-size crater markers, not actual holes.

## Launch the original arena world only

The original, unchanged arena is stored at `map/worlds/arena.sdf`. After building and sourcing the workspace, it can be opened without spawning the robot:

```bash
gz sim install/description/share/description/worlds/arena.sdf -r
```

## Starting a new terminal

The workspace must be sourced in every new terminal before using its ROS packages:

```bash
cd ~/URRC2026-27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## After pulling new changes

Rebuild and source the workspace so Gazebo uses the newest world files:

```bash
cd ~/URRC2026-27
git pull
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to start
source install/setup.bash
```

Close any existing Gazebo window before launching again.

## Troubleshooting

### `Package not found`

Source ROS and then this workspace in the same terminal:
=======
With the declared ROS, Gazebo, and RealSense dependencies installed, build and run the existing combined test setup from the repository root:
>>>>>>> origin

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
