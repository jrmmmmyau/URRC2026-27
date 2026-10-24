# URRC 2026–27 Gazebo Simulation

This repository contains the Melbot ROS 2 simulation and six Gazebo terrain variants based on the NASA Lunabotics arena.

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic and the ROS–Gazebo (`ros_gz`) packages
- `colcon`, `rosdep`, and Xacro (`ros-jazzy-xacro`)

## First-time setup

Open a terminal in the repository and build the workspace:

```bash
cd ~/URRC2026-27
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to start
source install/setup.bash
```

## Launch a terrain world

Choose a terrain using a number from `1` through `6`:

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

The terrain launcher starts Gazebo, spawns the camera/IMU-equipped Melbot,
starts the ROS–Gazebo bridges, and runs the driver node. The GUI still uses
OGRE 1 for compatibility with integrated graphics. The separate server-side
RGB-D sensor renderer requires OGRE 2 support; forcing the server to OGRE 1
will prevent the RGB-D cameras from working.

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

The older command `ros2 launch start sim.launch.py` is also supported and
includes this same flat-arena launcher.

## Cameras and IMU

Both map launchers expand the new Xacro robot description. The robot includes
front and top RGB-D cameras, the rotating top-camera mount, and an IMU.
The IMU intentionally has no visible mesh; check its data rather than looking
for an IMU box in Gazebo.

The ROS bridges expose `/imu` plus `/front_camera/image`,
`/front_camera/depth_image`, `/front_camera/camera_info`, and
`/front_camera/points`. The top camera has the same topic suffixes under
`/top_camera`. The launchers also bridge the world clock to `/clock` and use
simulation time. The URDF branch's driver rotates the top-camera mount through
`/cmd_vel/camera`.

After launching, check data from a second terminal sourced to the same workspace:

```bash
ros2 topic echo /imu --once
ros2 topic hz /front_camera/image
```

Topic presence alone does not prove that sensor messages are arriving. Use
the commands above to check actual messages and rates. Close any old Gazebo
process and rebuild `description`, `driver`, and `start` after this update:

```bash
colcon build --packages-up-to start
source install/setup.bash
```

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

```bash
source /opt/ros/jazzy/setup.bash
source ~/URRC2026-27/install/setup.bash
ros2 pkg prefix description
```

The final command should print a path ending in `URRC2026-27/install/description`.

### Gazebo still shows older terrain or missing rocks

Rebuild the packages that install the worlds and launch files, then source the result:

```bash
cd ~/URRC2026-27
colcon build --packages-select description start
source install/setup.bash
```

You can confirm that the current terrain asset is installed with:

```bash
grep '<size>6.88 5.00' install/description/share/description/worlds/lunabotics_terrain_01.sdf
```

The output should contain:

```xml
<size>6.88 5.00 0.85</size>
```

The heightfield size alone does not identify the rock-placement fix. Confirm
the installed barrier also reaches the column:

```bash
grep 'rock_construction_7' install/description/share/description/worlds/lunabotics_terrain_01.sdf
```

Close and relaunch Gazebo after rebuilding; an open simulation does not reload
changed world files. Terrain rocks must be placed in Gazebo's world-height
frame (nominal grade 0.40 m), not the design-height frame (nominal grade 0 m).

### Invalid terrain number

The `world` argument only accepts the values `1`, `2`, `3`, `4`, `5`, and `6`.

## Map files

Worlds, terrain assets, and generation scripts are organized under [`map/`](map/README.md). Regenerating the maps is not required just to launch the simulation.

## Regression checks

```bash
python3 -m unittest discover -s map/scripts -p 'test_*.py' -v
python3 -m unittest discover -s src/start/test -p 'test_*.py' -v
```

The sensor checks validate launch wiring, world-level sensor systems, and
Xacro expansion when `xacro` is installed. They do not replace a live Gazebo
camera, IMU, and teleop test.
