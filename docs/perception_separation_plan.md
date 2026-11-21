# Perception Separation: Current Scope and Plan

## Scope correction

This is a reorganization task. Move existing perception responsibilities into one package, create the necessary CMake/package definitions, and adjust configuration, messages, and launch files only where separation requires it. Preserve existing algorithms. Do not develop unfinished features from reference branches or treat those branches as validated designs.

All changes stay in `jerome/create_perception`. Other branches are read-only. All repository text must be English. Preserve user MATLAB/CSV data and unrelated navigation, driver, model, and odometry code. Each further implementation step requires user review and confirmation.

The original committed checkout at `c4bb1577c49964f7c9fb7d12b09be3d35e324e92` has no `src/mapping` package. The package imported during this chat, its added world-ray algorithm, configuration, launch, and map-specific tests have been removed. The depth world-coordinate extension has also been removed, including localization-pose input, pose interpolation, pose history, pending-frame queue, retry timer, and `DepthObservation` message.

Neither local mapping nor global mapping is implemented by this task. Navigation continues using its existing authored/fake-map workflow on `/map`. Perception does not update that map.

## Retained responsibilities

| Component | Responsibility |
| --- | --- |
| Camera driver or simulation | Supply images and calibration |
| Robot model/TF producers | Supply mounting relationships |
| Perception detector launch | Start the existing image rectifier and AprilTag detector |
| Perception tag observer | Prepare robot-relative tag observations using the TF lookup extracted from localization |
| Localization | Combine those observations with known field tag positions and retain the original robot-pose calculation |
| Perception depth observer | Publish sampled points in camera optical coordinates |
| Navigation test-map publisher | Publish authored maps using the existing workflow |
| Start package | Compose the existing AprilTag test setup |

The original AprilTag pipeline already measured tags relative to the camera, used mounting TF to express them relative to the robot, and calculated the robot's field pose. Localization published that result as `/tag_pose`, not a world-to-robot TF. This behavior remains. Removing the depth world's pose input does not remove localization's pose output.

## Current interfaces

| Connection | Data |
| --- | --- |
| Camera to tag detector | Color image and camera calibration |
| Detector to tag observer | `/detections` plus observed tag TF and existing mounting TF |
| Tag observer to localization | `/perception/tag/observations`, `perception/msg/TagObservationArray` |
| Localization output | `/tag_pose`, `geometry_msgs/msg/PoseWithCovarianceStamped` |
| Camera to depth observer | Depth image and matching calibration |
| Depth observer output | `/perception/depth/points`, `sensor_msgs/msg/PointCloud2` in camera optical coordinates |
| Navigation test-map publisher output | `/map`, independent of perception |

Tag observations preserve successful detection order and measurement timestamps, with double-precision transforms. Localization retains its known-tag lookup, pose equation, covariance, jump handling, history, and pose publication. Tag observation preparation retains the original TF direction, timestamp, and configured 0.1-second wait. A zero tag timestamp retains TF's existing latest-transform convention.

Separating the tag input requires a new message boundary. The observer may look up IDs unknown to localization before localization rejects them; this can affect latency. The configured prefix is `tag_`, matching the active detector, while the historical direct localizer executable default was `tag36h11:`. These interface differences are documented rather than presented as identical live behavior.

Depth output preserves the source's center-row selection, every-fourth-pixel default, 8-meter forward-depth limit, millimeter/meter conversion, and horizontal X/Z projection. Invalid depth samples are skipped. Points retain image time and the optical frame, with FLOAT64 XYZ fields at offsets 0/8/16 and 24 bytes per point. A valid image without valid samples emits an empty cloud.

The existing extraction also computes vertical Y from `fy`/`cy` to populate the XYZ interface and checks image/calibration correspondence and finite intrinsics. These are explicit differences from the source's X/Z-only routine. They do not build a map or estimate robot motion. No further algorithm changes are included in the cleanup.

Depth processing has no TF listener, `/tag_pose` subscription, pose interpolation, or coordinate-mode selector. It does not publish world points or occupancy grids.

## Progress after the correction

| Earlier step | Current disposition |
| --- | --- |
| 1: Inspect and define ownership | Retained, narrowed to extraction and packaging |
| 2: Create perception package | Retained |
| 3: Separate detector startup/configuration | Retained |
| 4: Import mapping package | Reversed; imported package removed |
| 5: Extract depth processing | Standalone camera-depth output retained; mapper connection removed |
| 5A: Separate tag observation preparation | Retained; original localization calculation preserved |
| 6: Add world conversion and moving-origin mapping | Reversed; additions removed |
| 7 onward: Complete system | Not started; previous mapping integration plan withdrawn |

Further work should first review the retained extraction against the original. Any package or launch adjustment must use the existing behavior and receive the user's step approval. Local/global mapping algorithms are separate future work.

## Validation and practical limits

```bash
python3 tests/tag_separation/run.py --sanitize
python3 tests/tag_separation/check_contract.py
python3 tests/perception_depth/run.py --sanitize
python3 tests/perception_depth/check_contract.py
```

The tag suite compares 212 frames through extracted original/current callbacks and checks that the localization calculation/output block remains unchanged. The depth suite tests retained sampling and cloud encoding without a mapper. Static checks cover package declarations, syntax, topics, and absence of removed world-mode connections. See the test READMEs for exact scope.

These tests use local C++ helpers and explicit test doubles. They do not establish ROS node compilation, generated-interface compatibility, actual tf2/DDS behavior, launch execution, or live camera performance. ROS 2 Jazzy validation remains pending. Earlier counts from the deleted mapping suites are historical results, not coverage of the current checkout.

The existing AprilTag test launch still disables depth and retains its simulation/real-camera setup. Navigation's intentional dependency declarations remain untouched. Keep its test-map workflow; do not substitute a perception-produced map.
