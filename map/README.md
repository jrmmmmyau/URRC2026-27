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

Generate all six numbered worlds and refresh the flat arena's obstacles
(requires Python 3 and Pillow):

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

Each launchable Lunabotics world includes a seven-rock line at the
construction-zone west entrance. Its collision footprints overlap from the
south perimeter duct to the column, closing the short approach south of the
column. Barrier rocks have 36-40 cm local widths and varied heights.

Scattered rocks use six distinct local widths between 30 and 40 cm and heights
between 15 and 26 cm above their origins. Their visual scales and box collisions
are derived from the same OBJ bounds. Sizes rotate between the numbered maps.
Rock origins sit 2 cm above the local terrain; their lower mesh vertices embed
slightly into it. Gazebo's nominal terrain grade is 0.40 m, unlike the generator's
0 m design grade. The flat arena stays at z=0.

Each numbered map has three true craters with 30, 40, and 50 cm rim-to-rim
diameters and design depths between 15 and 34 cm. Craters are part of both the
render mesh and collision heightmap, not dark surface decals. The flat arena
retains its simplified crater markers, now in the same three diameters.
The archived original `worlds/arena.sdf` remains unchanged.

After regeneration, run the geometry regression tests:

```bash
python3 -m unittest discover -s map/scripts -p 'test_*.py' -v
```

These independently sample the actual mesh triangles to catch buried/floating
rocks, verify collision/visual bounds and heightmap alignment, check crater
depth variation, resolve asset paths, and check the connected barrier. They do
not replace a Gazebo rendering and driving test.
