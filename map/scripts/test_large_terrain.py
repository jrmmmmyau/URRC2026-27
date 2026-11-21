"""Geometry checks for the sparse 50 x 50 m testing terrain."""

import math
from pathlib import Path
import tempfile
import unittest
from xml.etree import ElementTree as ET

from PIL import Image

import generate_large_terrain as large
import generate_lunabotics_worlds as original
from test_lunabotics_worlds import (
    TerrainMesh, collision_footprint, numbers, overlaps, transform, vertices,
)


class LargeTerrainTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = large.OUTPUT
        cls.world = ET.parse(cls.path).getroot().find('world')
        cls.source = ET.parse(large.SOURCE).getroot().find('world')
        cls.terrain = cls.world.find("model[@name='bp1_heightmap_terrain']")
        cls.mesh = TerrainMesh(cls.world, cls.path)
        scale = numbers(cls.terrain, './link/visual/geometry/mesh/scale')
        cls.mesh.points = [tuple(p * s for p, s in zip(point, scale))
                           for point in cls.mesh.points]
        cls.mesh.x_min, cls.mesh.y_max, _ = cls.mesh.points[0]
        cls.mesh.dx = (cls.mesh.points[cls.mesh.count - 1][0] - cls.mesh.x_min) / (cls.mesh.count - 1)
        cls.mesh.dy = (cls.mesh.y_max - cls.mesh.points[-1][1]) / (cls.mesh.count - 1)

    def test_dimensions_ground_contact_and_all_asset_paths(self):
        heightmap = self.terrain.find('./link/collision/geometry/heightmap')
        size = numbers(heightmap, 'size')
        self.assertEqual(size[:2], (50.0, 50.0))
        self.assertAlmostEqual(self.mesh.points[0][0], -25)
        self.assertAlmostEqual(self.mesh.points[-1][0], 25)
        self.assertAlmostEqual(self.mesh.points[0][1], 25)
        self.assertAlmostEqual(self.mesh.points[-1][1], -25)
        with Image.open(self.path.parent / heightmap.findtext('uri')) as image:
            peak = image.getextrema()[1]
            for x, y, z in self.mesh.points:
                column = round((x / 50 + .5) * (image.width - 1))
                row = round((.5 - y / 50) * (image.height - 1))
                collision_z = image.getpixel((column, row)) * size[2] / peak
                self.assertAlmostEqual(z, collision_z, delta=5e-6)
        for uri in self.world.findall('.//uri'):
            self.assertTrue((self.path.parent / uri.text).is_file(), uri.text)

    def test_sparse_layout_and_safe_robot_spawn(self):
        models = self.world.findall('model')
        self.assertEqual({m.get('name') for m in models},
                         {m.get('name') for m in self.source.findall('model')})
        self.assertEqual(sum(m.get('name').startswith('rock_') for m in models), 13)
        x, y, z = large.SPAWN
        self.assertAlmostEqual(self.mesh.height(x, y), 1.60, delta=1e-6)
        self.assertAlmostEqual(z - self.mesh.height(x, y), .05, delta=1e-6)
        for model in models:
            if not model.get('name').startswith('rock_'):
                continue
            px, py, *_ = numbers(model, 'pose')
            self.assertTrue(0 < px < 50 and 0 < py < 50)
            self.assertGreater(math.hypot(px - x, py - y), 4)

    def test_large_craters_are_real_depressions_and_slopes_remain(self):
        depths = []
        for x, y, radius, _, _ in original.VARIANTS[0].craters:
            cx, cy = x * large.SCALE_X, y * large.SCALE_Y
            rim = [self.mesh.height(cx + radius * large.SCALE_X * math.cos(i * math.tau / 64),
                                    cy + radius * large.SCALE_Y * math.sin(i * math.tau / 64))
                   for i in range(64)]
            depth = sum(rim) / len(rim) - self.mesh.height(cx, cy)
            self.assertGreater(depth, .40)
            depths.append(depth)
        self.assertGreater(max(depths) - min(depths), .40)
        self.assertGreater(max(p[2] for p in self.mesh.points) - min(p[2] for p in self.mesh.points), 1)

    def test_rocks_are_grounded_and_the_barrier_still_closes(self):
        rock_mesh = vertices(original.ASSETS_ROOT / 'shared/lunar_rock.obj')
        for model in self.world.findall('model'):
            if not model.get('name').startswith('rock_'):
                continue
            scale = numbers(model, './link/visual/geometry/mesh/scale')
            pose = numbers(model, 'pose')
            local = [tuple(p * s for p, s in zip(point, scale)) for point in rock_mesh]
            center = numbers(model, './link/collision/pose')[:3]
            size = numbers(model, './link/collision/geometry/box/size')
            for axis in range(3):
                self.assertAlmostEqual(min(p[axis] for p in local), center[axis] - size[axis] / 2, delta=2e-5)
                self.assertAlmostEqual(max(p[axis] for p in local), center[axis] + size[axis] / 2, delta=2e-5)
            points = [transform(p, pose) for p in local]
            clearances = [z - self.mesh.height(x, y) for x, y, z in points]
            self.assertLess(min(clearances), .10, model.get('name'))
            self.assertGreater(max(clearances), .20, model.get('name'))
        rocks = [self.world.find(f"model[@name='{name}']") for name in original.CONSTRUCTION_ROCK_NAMES]
        footprints = [collision_footprint(rock) for rock in rocks]
        column = collision_footprint(self.world.find("model[@name='central_support_column']"))
        for a, b in zip(footprints, footprints[1:] + [column]):
            self.assertTrue(overlaps(a, b))
        duct = self.world.find("model[@name='perimeter_ducts']/link/collision[@name='south_collision']")
        y = numbers(duct, 'pose')[1]
        radius = float(duct.findtext('./geometry/cylinder/radius'))
        self.assertLess(min(p[1] for p in footprints[0]), y + radius)

    def test_generator_reproduces_checked_in_world(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'world.sdf'
            large.generate_world(output)
            self.assertEqual(output.read_bytes(), self.path.read_bytes())


if __name__ == '__main__':
    unittest.main()
