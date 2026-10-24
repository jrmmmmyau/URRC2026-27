# Lunabotics Gazebo maps

The map package is organized by purpose:

- `worlds/` contains launchable SDF worlds. `arena.sdf` is the original world,
  moved here without content changes. The numbered files are six generated
  terrain and obstacle-layout variants.
- `assets/shared/` contains rocks and zone-label meshes used by every variant.
- `assets/terrain_01/` through `terrain_06/` contain each world's matching
  collision heightmap, render mesh, and texture maps.
- `templates/` contains the common terrain-world SDF source.
- `scripts/` contains deterministic asset and world generators.

Generate all six numbered worlds:

```bash
python3 map/scripts/generate_zone_labels.py
python3 map/scripts/generate_lunabotics_worlds.py
```

After building and sourcing the ROS workspace, launch a numbered variant:

```bash
ros2 launch start terrain_teleop.launch.py world:=1
ros2 launch start terrain_teleop.launch.py world:=6
```

Broad terrain grades are capped at 15 degrees. Crater walls are discrete
competition obstacles and are not included in that broad-grade limit.
