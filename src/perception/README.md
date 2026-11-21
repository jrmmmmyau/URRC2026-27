# URRC Perception

This ROS 2 package owns image rectification, AprilTag detection, and depth observation processing for the URRC autonomy stack.

## Current status

Detector startup/configuration and robot-relative tag-observation preparation are separated from localization. Standalone depth processing publishes camera-coordinate points. The imported mapper and new depth world-coordinate conversion have been removed at the user's request. The detector-only launch still starts the existing `image_proc` and `apriltag_ros` executables.

Robot pose estimation remains in `apriltag_localization`. The `start` package combines camera startup, perception, and localization.

## Responsibilities

- Rectify camera images and run AprilTag detection.
- Publish tag detections and observed camera-to-tag TF.
- Read those detections and mounting/observed TF to publish complete robot-relative tag observations for localization.
- Convert sampled depth measurements into standalone camera-coordinate observations.

Robot pose estimation belongs to localization. Map building is outside this package and the current separation task. Camera mounting relationships are provided by the robot model or configured TF publishers.

Depth output uses camera optical coordinates only. It has no localization-pose input or depth TF conversion. Tag perception retains the TF queries extracted from the original localization input layer. Localization retains the field tag map and robot pose calculation.

See the [perception separation plan](../../docs/perception_separation_plan.md) in the source workspace for the interfaces, migration sequence, and approval checkpoints. That relative link is intended for the source checkout; the design document is not installed with this package.

## Files

| File | Purpose |
| --- | --- |
| `package.xml` | Package identity, build type, and launch/runtime dependencies |
| `CMakeLists.txt` | Build depth/tag observers, generate public messages, and install resources |
| `launch/apriltag_detection.launch.py` | Start image rectification and AprilTag detection |
| `config/apriltag.yaml` | Tag family, physical sizes, detector settings, and observation frame names |
| `src/tag_observer.cpp` | Prepare ordered robot-relative tag observations from detections and TF |
| `msg/TagObservation.msg`, `msg/TagObservationArray.msg` | Public double-precision tag observation contract |
| `launch/tag_observation.launch.py`, `config/tag_observer.yaml` | Independent observer startup and coordinate/TF settings |
| `src/depth_observer.cpp` | Subscribe to depth/calibration and publish processed observations |
| `include/perception/` | Depth sampling and FLOAT64 encoding helpers |
| `launch/depth_observation.launch.py` | Start depth processing with configurable topic connections |
| `config/depth.yaml` | Depth sampling interval, selected row, and maximum depth |
| `LICENSE` | License retained from the source localization package |

## Requirements

Use the repository's target environment: Ubuntu 24.04 with ROS 2 Jazzy, `ament_cmake`, and `colcon` available.

The package uses `ament_cmake` to build and install its resources. The C++17 depth node needs `rclcpp`, `sensor_msgs`, `cv_bridge`, and OpenCV. The manifest declares OpenCV using the rosdep key `libopencv-dev`. Launch/runtime dependencies include `ament_index_python`, `launch`, `launch_ros`, `image_proc`, and `apriltag_ros`. The tag observer additionally uses `apriltag_msgs`, `geometry_msgs`, `std_msgs`, `tf2`, and `tf2_ros`. Its public messages use `geometry_msgs` and `std_msgs`, with `rosidl_default_generators`/`rosidl_default_runtime` for generation and export. Install the declared dependencies in the ROS environment before building.

## Discover and build

Run these commands from the repository root in a terminal with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src --packages-select perception
colcon build --base-paths src --packages-select perception
source install/setup.bash
ros2 pkg prefix perception
```

The package list should identify `perception` as `ros.ament_cmake`. After a successful build and sourcing the install environment, `ros2 pkg prefix perception` should print its installation prefix.

## Run tag perception independently

With the camera already publishing the input topics below, run:

```bash
ros2 launch perception apriltag_detection.launch.py
```

| Direction | Topic | Meaning |
| --- | --- | --- |
| Input | `/camera/top_camera/color/image_raw` | Raw color image |
| Input | `/camera/top_camera/color/camera_info` | Camera calibration |
| Intermediate | `/image_rect` | Rectified image sent to the detector |
| Output | `/detections` | AprilTag observations |
| Output | `/tf` | Observed camera-to-tag transforms, including `tag_0` with the current configuration |

The detector node is explicitly named `apriltag` to match the YAML parameter root. The configuration preserves family `36h11`, tag ID 0, frame `tag_0`, and physical tag size 0.16 meters.

This launch starts the two perception nodes. Camera startup and robot mounting TF are supplied by the surrounding system. The detector computes camera-relative tag observations; the perception tag observer reads the mounting TF needed to relate those observations to the robot.

## Prepare robot-relative tag observations

Start the observer in another sourced terminal alongside the detector:

```bash
ros2 launch perception tag_observation.launch.py
```

The observer consumes `/detections` and reads the existing TF tree. It publishes `/perception/tag/observations` as `perception/msg/TagObservationArray`. Each array contains:

- `header.stamp`: the original detection time, even if TF returns a different timestamp.
- `header.frame_id`: the coordinate frame of all observations, default `base_link`.
- `observations`: ordered entries containing an `int64` tag ID and a `geometry_msgs/Transform` expressing that tag frame in the header frame. Translations are meters; rotations are ROS x/y/z/w quaternions. Values retain double precision.

The observer does not publish new TF edges or know tag field positions. Successful entries retain detection order, including repeated IDs. Failed TF lookups skip that tag; an empty array causes no localization output. Unknown field IDs are still valid perception observations and are ignored by localization. The extra lookups for such IDs can add latency compared with the original localizer and require runtime measurement.

`config/tag_observer.yaml` sets `output_frame: base_link`, `tag_frame_prefix: tag_`, and `tf_timeout: 0.1`. The prefix matches the detector's `tag_0` frame. The observer's direct-executable prefix default is now `tag_`, matching the configured pipeline; the old localizer executable default was `tag36h11:`. Settings are read at startup. TF queries preserve the old direction and detection timestamp, including TF's convention that a zero timestamp requests the latest available transform. This extraction does not redefine zero as an exact historical time.

The launch accepts `base_frame`, `detections_topic`, `observations_topic`, and `use_sim_time`. Set the robot frame, observation topic, and clock consistently with localization. The executable uses relative `detections` and `tag/observations`; direct users must supply remappings equivalent to the launch. For example:

```bash
ros2 run perception tag_observer --ros-args -r tag/observations:=/perception/tag/observations
```

Detection input, observation output, and localization input use depth-10 reliable/volatile QoS. Verify compatibility with the installed detector in ROS. The new message boundary adds transport and is not a promise of identical live scheduling.

## Run localization or the combined pipeline

Build `apriltag_localization` as well before starting localization separately:

```bash
colcon build --base-paths src --packages-select perception apriltag_localization
source install/setup.bash
ros2 launch apriltag_localization apriltag.launch.py
```

The localization launch now starts only `tag_localizer`. It requires prepared tag observations in its configured `base_frame`; it no longer subscribes to detections or TF. A different observation frame is rejected. Localization uses perception's public message definitions, while keeping the field-tag map, pose equation, covariance, jump handling, and `/tag_pose` output. The observer must be running as well, unless observations are supplied by replay or a test publisher. Field tag positions and localization settings are in `apriltag_localization/config/localization.yaml`; detector settings belong in this package.

For the existing combined test setup, build the workspace packages needed by `start` and run:

```bash
colcon build --base-paths src --packages-up-to start
source install/setup.bash
ros2 launch start apriltag_test.launch.py
```

That entry point starts the existing simulation, a RealSense color stream, the detector, the tag observer, and localization exactly once. Its `base_frame` and `observations_topic` arguments are forwarded to both the observer and localization. It requires the corresponding simulation and camera dependencies and the real camera. Use this combined command instead of the old localization command when you need detector startup too. Use either the combined entry point or the separate launches to avoid duplicate nodes.

The existing mix of simulation and real-camera timing still needs runtime validation. Depth remains disabled in this test entry point.

## Run depth perception independently

With a depth camera already publishing images and calibration, run:

```bash
ros2 launch perception depth_observation.launch.py
```

The launch defaults to `/camera/camera/depth/image_rect_raw` and `/camera/camera/depth/camera_info`, preserving the source depth-topic defaults. Override them with `depth_image_topic` and `camera_info_topic`. Its output defaults to `/perception/depth/points`, configurable through `points_topic`. Set `use_sim_time` consistently with the source when using simulation or replay.

The executable internally uses relative topics `depth/image`, `depth/camera_info`, and `depth/points`; the launch supplies the default remappings above.

`config/depth.yaml` preserves the original settings: `pixel_step: 4`, `row_fraction: 0.5`, and `max_range: 8.0`. The maximum applies to forward depth Z in meters, not radial distance. Settings are read at startup; restart after changing configuration.

The node accepts `16UC1` depth in millimeters and `32FC1` depth in meters. It requires calibration with matching image dimensions and the same nonempty optical frame, finite intrinsics, and positive `fx` and `fy`. The original horizontal calculation uses `CameraInfo.K`; vertical coordinates additionally use `fy` and `cy`. Actual rectified camera calibration must be checked before deployment, including whether K and P describe the same image geometry.

Only finite, positive depths up to the maximum are accepted. Invalid depths are skipped, preserving left-to-right sample order. A successfully processed image with no valid depths still emits a zero-point cloud. Missing/mismatched calibration, unsupported encoding, empty images, or invalid intrinsics cause the image to be skipped with throttled diagnostics.

Observations use standard `sensor_msgs/msg/PointCloud2` with little-endian `FLOAT64` x/y/z fields at offsets 0/8/16, `point_step: 24`, `height: 1`, and `is_dense: true`. The image's timestamp and optical frame are copied into the cloud header. Optical directions are X right, Y down, Z forward; units are meters.

FLOAT64 preserves the source's double-precision calculations without introducing FLOAT32 rounding. Consumers must support this FLOAT64 XYZ layout.

These observations are standalone outputs for recording or future consumers. Navigation continues using its own fake map on `/map`; this node does not modify that map. No mounting TF or localization pose is needed for depth output.

## What changed during separation

- Image rectification and AprilTag detection still use the existing external executables and detector settings; only package/configuration/startup ownership changed.
- `tag_observer` contains the TF lookup previously inside localization. The new public message carries the detected tag ID, its transform relative to the robot, and the detection timestamp. Localization consumes this message and keeps its existing pose calculation and result handling.
- `depth_observer` exposes the extracted sampling/filtering as a separate program, using the same center-row sampling, pixel stride, depth limit, unit conversion, and horizontal X/Z calculation. The previous direct function call is replaced by a PointCloud2 output.
- The XYZ interface includes vertical Y calculated from `fy` and `cy`; the source routine used only horizontal X and depth Z. Matching image/calibration dimensions and frames, finite intrinsics, and valid parameter checks were also added. These are retained differences from the source routine, not a claim of byte-identical extraction. FLOAT64 avoids rounding the source's double-precision positions.
- The depth world transform, pose interpolation/history, pending-frame queue, retry timer, pose input, extra world message, and world-mode settings have been removed. This package does not implement mapping algorithms.

Keep future code changes limited to those needed for separation and interfaces. Do not add estimation, filtering, or mapping algorithms as part of packaging work.

## Validation status

The optional synthetic test harnesses were removed at the user's request. Earlier local check results do not establish a successful ROS build or live run.

- Local checks cover Python syntax, YAML/XML structure, dependency declarations, and preservation of detector settings and localization parameters.
- Package discovery can be checked with `colcon list`; it is separate from build and runtime verification.
- All new code, configuration, comments, and documentation use English.
- The current local environment lacks `ament_cmake`, `launch`, and `launch_ros`. The Step 2 build attempt stopped at the missing build dependency. ROS node compilation, cv_bridge decoding, real launch, message delivery, TF lookup, pose output, and depth output still require verification in ROS 2 Jazzy.

## Next step

The earlier mapping/world-mode extension is withdrawn. Review the remaining separation before any further package or launch edits. Full ROS build and live validation remain pending; local and global mapping are outside this task.
