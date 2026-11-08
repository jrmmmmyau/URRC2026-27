#!/usr/bin/env python3
"""Create a sparse 50 x 50 m testing version of Lunabotics terrain 1.

Run after generate_lunabotics_worlds.py when refreshing the source terrain.
The heightmap and mesh are shared; their world scales expand the terrain.
"""

from pathlib import Path
from xml.etree import ElementTree as ET


MAP_ROOT = Path(__file__).resolve().parent.parent
SOURCE = MAP_ROOT / 'worlds/lunabotics_terrain_01.sdf'
OUTPUT = MAP_ROOT / 'worlds/lunabotics_terrain_large.sdf'
WIDTH = LENGTH = 50.0
SCALE_X = WIDTH / 6.88
SCALE_Y = LENGTH / 5.0
SCALE_Z = 4.0
NOMINAL_GRADE = .40 * SCALE_Z
SPAWN = (SCALE_X, .75 * SCALE_Y, NOMINAL_GRADE + .05)


def scaled_text(text, factors):
    return ' '.join(f'{float(value) * factor:.12f}'
                    for value, factor in zip(text.split(), factors))


def generate_world(output=OUTPUT):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(SOURCE, parser=parser)
    world = tree.getroot().find('world')
    world.insert(0, ET.Comment(
        ' 50 x 50 m testing terrain; sparse original layout, '
        'larger obstacles and 4x vertical relief. '
    ))
    for child in world:
        if child.tag is ET.Comment and 'The PNG encodes' in child.text:
            child.text = (
                '\n      Terrain 1 assets are stretched to 50 x 50 m with 4x vertical relief.\n'
                '      Nominal grade is z=1.60 m. Mesh and collision heights use the\n'
                '      same scaling, including compensation for image-peak normalization.\n'
                '      Relative paths keep the installed world relocatable.\n    '
            )
    # Retain the world name expected by the terrain launcher's joint bridge.
    world.set('name', 'lunabotics_terrain_arena')
    world.find('light/pose').text = '25 25 35 0 0 0'

    for model in world.findall('model'):
        name = model.get('name')
        if name == 'bp1_heightmap_terrain':
            model.find('pose').text = '25 25 0 0 0 0'
            heightmap = model.find('./link/collision/geometry/heightmap')
            size = heightmap.find('size')
            size.text = scaled_text(size.text, (SCALE_X, SCALE_Y, SCALE_Z))
            mesh = model.find('./link/visual/geometry/mesh')
            ET.SubElement(mesh, 'scale').text = scaled_text('1 1 1', (SCALE_X, SCALE_Y, SCALE_Z))
            continue

        rock = name.startswith('rock_')
        local_xy = max(SCALE_X, SCALE_Y) if rock else None
        for pose in model.iter('pose'):
            values = list(map(float, pose.text.split()))
            model_pose = pose is model.find('pose')
            # Rock-local bounds must scale along the same axes as their mesh.
            values[0] *= SCALE_X if model_pose or not rock else local_xy
            values[1] *= SCALE_Y if model_pose or not rock else local_xy
            if rock:
                values[2] *= SCALE_Z
                if model_pose:
                    # Keep exaggerated rocks upright rather than tipping their
                    # widened footprints through the floor.
                    values[3:5] = [0.0, 0.0]
            else:
                # Walls, column and markings retain their original height
                # above nominal grade as the terrain elevation increases.
                values[2] += NOMINAL_GRADE - .40
            pose.text = ' '.join(f'{value:.12f}' for value in values)

        for geometry in model.iter('geometry'):
            box = geometry.find('box/size')
            if box is not None:
                factors = (local_xy, local_xy, SCALE_Z) if rock else (SCALE_X, SCALE_Y, 1)
                box.text = scaled_text(box.text, factors)
            mesh = geometry.find('mesh')
            if mesh is not None:
                scale = mesh.find('scale')
                if scale is None:
                    scale = ET.SubElement(mesh, 'scale')
                    scale.text = '1 1 1'
                factors = (local_xy, local_xy, SCALE_Z) if rock else (SCALE_X, SCALE_Y, 1)
                scale.text = scaled_text(scale.text, factors)
            cylinder = geometry.find('cylinder')
            if cylinder is not None:
                radius = cylinder.find('radius')
                radius.text = f'{float(radius.text) * min(SCALE_X, SCALE_Y):.12f}'
                if name == 'perimeter_ducts':
                    # The cylinders' local z axes point along the arena edges.
                    length = cylinder.find('length')
                    along_x = float(length.text) > 5
                    length.text = f'{float(length.text) * (SCALE_X if along_x else SCALE_Y):.12f}'
    ET.indent(tree, space='  ')
    tree.write(output, encoding='UTF-8', xml_declaration=True)
    return output


if __name__ == '__main__':
    print(generate_world())
