# Terrain Maps

Build and source the workspace from the repository root, then launch a terrain:

```bash
ros2 launch start terrain_teleop.launch.py world:=1
ros2 launch start terrain_teleop.launch.py world:=large
```

Use `world:=1` through `world:=6` for the six standard terrains; `world:=large` loads the sparse 50 x 50 m testing terrain.

Regenerate the standard maps and large map, then run geometry tests:

```bash
python3 map/scripts/generate_lunabotics_worlds.py
python3 map/scripts/generate_large_terrain.py
python3 -m unittest discover -s map/scripts -p 'test_*.py' -v
```

Worlds are in `worlds/`; shared and terrain assets are in `assets/`.
