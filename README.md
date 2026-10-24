# URRC 2026–27 Gazebo Simulation

This repository contains the Melbot ROS 2 simulation and six Gazebo terrain variants based on the NASA Lunabotics arena.

## Requirements

- Ubuntu with ROS 2 Jazzy
- Gazebo Harmonic and the ROS–Gazebo (`ros_gz`) packages
- `colcon`

## First-time setup

Open a terminal in the repository and build the workspace:

```bash
cd ~/URRC2026-27
source /opt/ros/jazzy/setup.bash
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

The terrain launcher starts Gazebo, spawns Melbot, starts the ROS–Gazebo bridges, and runs the driver node. It also uses the OGRE 1 renderer to avoid the shader crash seen on some integrated graphics systems.

## Launch the flat Lunabotics arena

To launch the simpler flat arena with Melbot and the driver:

```bash
ros2 launch start teleop.launch.py
```

This launch file uses `map/worlds/lunabotics_arena.sdf`.

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

### Gazebo still shows an older or flatter terrain

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

### Invalid terrain number

The `world` argument only accepts the values `1`, `2`, `3`, `4`, `5`, and `6`.

## Map files

Worlds, terrain assets, and generation scripts are organized under [`map/`](map/README.md). Regenerating the maps is not required just to launch the simulation.
