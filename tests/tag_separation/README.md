# Tag Observation Separation Checks

Run from the repository root:

```bash
python3 tests/tag_separation/run.py --sanitize
python3 tests/tag_separation/check_contract.py
python3 tests/perception_depth/run.py --sanitize
```

The C++ runner requires Python 3 and a C++17 compiler; override the compiler with `CXX` if necessary. The static contract checker additionally uses PyYAML and catkin_pkg. Temporary binaries are built outside the source tree and removed after use.

## Baseline and executed code

`fixtures/reference_tag_localizer.cpp` is the exact pre-extraction localizer from main commit `c4bb1577c49964f7c9fb7d12b09be3d35e324e92`. SHA-256: `c41f55f36d0c78dd37a59b5c6631fcf47103d9cb55154c7834cbab7f025938ff`.

The runner checks that hash, extracts the original localizer callback and the current observer/localizer callbacks directly from their source files, and compiles all three into the same harness. It also checks that the entire localization calculation/output block remains byte for byte identical: the pose equation, jump handling, covariance, timestamp/frame assignment, and history updates.

The harness supplies explicit test doubles for ROS message structures, publishers, time/duration wrappers, TF lookup, and unit-quaternion transform arithmetic. It executes the actual callback bodies, not copies of their control flow. The transform arithmetic is a local model; this does not validate the real tf2 library or ROS message generation.

## Coverage

- 212 input frames compare every published pose/header/covariance and the final localization history after each frame.
- Known translation and rotation examples, plus deterministic 3D transform sequences.
- Accepted motion, rejected jumps, and the original update of last-pose state even when a jump is rejected.
- Multiple, repeated, and unknown IDs; empty detections; complete and partial TF lookup failures.
- Correct query target/source, tag prefix, measurement timestamp, and timeout arguments.
- Zero timestamps are passed through, preserving the previous TF API request semantics; the stub does not emulate a live TF cache.
- Observation order, double-precision transform copying, and detection timestamp preservation even when the lookup result header has a different timestamp.
- Custom robot frames and rejection of wrong/empty observation frames before localization state changes.
- Public message fields, consistent detector/observer tag naming, launch topic/frame wiring, exactly one detector/observer/localizer include, and package dependency direction.

The depth checks separately cover standalone camera-coordinate observations.

## ROS checks still required

The local tests do not compile node constructors against ROS, execute the actual tf2 implementation, generate message code, serialize messages, exercise DDS/QoS, execute launch descriptions, or measure camera/TF latency. Address/undefined-behavior sanitizers cover the compiled local harness, not a ROS process.

In the target ROS 2 Jazzy environment, install the declared dependencies and build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to apriltag_localization
source install/setup.bash
ros2 interface show perception/msg/TagObservationArray
ros2 interface show perception/msg/TagObservation
```

Then run detector, observer, and localization using the perception README. For a controlled test without a camera, publish a known static `base_link -> tag_0` transform and an AprilTagDetectionArray on `/detections`; compare `/perception/tag/observations` and `/tag_pose` against the known field tag configuration. Supply consistent timestamps and clock settings. Repeat with missing TF, multiple tags, and a changing transform to verify measurement-time lookup. Test discovery/QoS and the additional observation hop's latency separately.

The observer may look up IDs that localization's field map does not contain. Those lookups did not occur in the old world-map-filtered callback; their wait time is an intentional boundary consequence to measure. A successful local numerical comparison does not imply identical live delivery or latency.

The intentionally retained navigation manifest condition is outside these checks and must not be silently repaired as part of tag extraction.
