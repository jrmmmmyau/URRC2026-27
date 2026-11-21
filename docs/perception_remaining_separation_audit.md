# Perception Separation Audit

## Current status

The detector and tag-observation input layer have been separated from localization into the existing perception package. Standalone camera-depth observation processing is retained. At the user's direction, the imported mapping package and the added depth world-coordinate mode have been removed. Local/global mapping remain outside this task; navigation continues using its existing fake maps.

This audit concerns the current checkout and locally inspected reference source, not completed algorithms on other branches. All other branches remain read-only. The goal is extraction and packaging, with source changes limited to the interfaces needed to separate responsibilities.

## Retained extraction

| Original responsibility | Current owner/change |
| --- | --- |
| Rectifier and AprilTag detector startup | `src/perception/launch/apriltag_detection.launch.py` |
| Detector family, physical tag size, tuning and observed-frame naming | `src/perception/config/apriltag.yaml` |
| Detection subscription, tag-frame naming, TF buffer/listener and robot-relative lookup formerly inside localization | `src/perception/src/tag_observer.cpp` |
| Prepared tag ID/transform/time output | `TagObservation.msg` and `TagObservationArray.msg` in perception |
| Known field tag positions, robot pose calculation, jump handling, covariance and pose output | Retained in `src/apriltag_localization` |
| Depth image sampling/filtering and camera-relative projection taken from reference source | Standalone `src/perception/src/depth_observer.cpp` and its helpers |
| Independent startup and build definitions | Perception CMake, package manifest, configuration and launch files |
| Combined AprilTag test startup | Existing `src/start/launch/apriltag_test.launch.py`, with separate detector, observer and localization nodes |

Localization receives the new prepared tag message, then uses its original pose calculation/output block. It retains `/tag_pose` in its configured world frame. Perception has no package dependency on localization. The original depth world-pose subscription added during Step 6 has been removed.

## Other inspected components

| Location | Disposition |
| --- | --- |
| Robot descriptions and camera mounting geometry | Retained in `src/description`; these supply geometry rather than interpret observations |
| Camera drivers and startup | Retained with the driver/system composition |
| Motor/steering programs and camera actuator command | Retained in `src/driver` |
| IMU integration and odometry | Retained in `src/imu_driver` |
| Navigation planner, verifier, costmap processing and path following | Retained in `src/navigation_server` |
| Navigation PGM publishers, test maps, map editors and generators | Retained; these use authored maps and are not camera perception |
| Navigation dependency declarations and older source copies | Retained; no cleanup or algorithm changes in this task |
| User `apriltag.m` and `tag_pose*.csv` | Preserved |

The source audit found no additional production camera image decoder, tag detector, depth projection, lidar processor, or object classifier outside the identified perception extraction. A component using TF, image files, or sensor messages is not automatically perception.

## Removed additions

The package imported as `src/mapping`, its build/configuration/launch resources, camera/world map update algorithms, and map-specific regression fixtures/tests are removed. Nothing is imported into navigation to replace them.

Depth world projection, pose interpolation/history, pending-frame handling, retry timer, localization-pose subscription, world settings, and `DepthObservation.msg` are removed. The retained tag observer still reads the original tag/mounting TF relationships; it does not estimate world pose.

## Verification

The optional synthetic test harnesses and original-code comparison fixture were removed at the user's request. Previously reported test results are historical; no test runners remain in the checkout. Runtime source is unchanged by this removal.

The separation necessarily changes message transport and startup. The observer may attempt TF lookup for unknown field IDs before localization rejects them, and its prefix default matches the configured `tag_` detector. Depth XYZ encoding also retains the previously added vertical coordinate and image/calibration validation. These differences are documented in the perception README and require live validation where applicable.

Full ROS build, generated messages, actual TF, QoS, latency and camera execution remain unverified locally. See the [current plan](perception_separation_plan.md) and package READMEs. Further work requires user review; do not resume the withdrawn mapping integration plan.
