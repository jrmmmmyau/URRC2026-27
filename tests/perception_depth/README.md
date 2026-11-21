# Standalone Camera Depth Checks

Run from the repository root:

```bash
python3 tests/perception_depth/run.py --sanitize
python3 tests/perception_depth/check_contract.py
```

The runner requires Python 3 and a C++17 compiler. Set `CXX` to select a compiler. It builds in a temporary directory with optional address/undefined-behavior sanitizers. The static checker also requires PyYAML and catkin_pkg.

The C++ suite compiles the production `depth_processing.hpp` and `observation_cloud.hpp` with plain message containers. It checks row selection, pixel stride and order, invalid and maximum-range depths, horizontal and vertical projection, invalid calibration/settings, empty observations, message fields, timestamp/frame preservation and exact FLOAT64 encoding. No mapper or world-mode implementation is needed.

The static checker checks manifests, launch syntax, YAML, depth topic connections, preservation of the source header, and absence of the removed pose subscription, world transform, mode arguments, world message and mapper package.

These checks do not compile full ROS nodes, decode real images with cv_bridge, generate ROS interfaces, or exercise TF, DDS, QoS or launch execution. They do not claim equivalence of a map-building algorithm. The removed `depth_separation` and `world_depth` suites are no longer part of this checkout.

In ROS 2 Jazzy, build `perception`, start a camera with matching depth/calibration, and run `ros2 launch perception depth_observation.launch.py`. Verify 16UC1 millimeters and 32FC1 meters, the actual calibration geometry, point order, frame/time preservation, invalid-image handling and empty output. See the perception README for topics and configuration. Camera-relative points should be published independently of localization or mounting TF. Navigation should continue receiving its existing test map on `/map`.
