#!/usr/bin/env python3
"""Geometry regression tests: python3 -m unittest discover -s map/scripts -p 'test_*.py'."""

import math
import unittest
from xml.etree import ElementTree as ET

from PIL import Image

import generate_lunabotics_worlds as generator


def numbers(element, path):
    return tuple(map(float, element.findtext(path).split()))


def vertices(path):
    return [tuple(map(float, line.split()[1:4]))
            for line in path.read_text().splitlines() if line.startswith("v ")]


def transform(point, pose):
    x, y, z, roll, pitch, yaw = pose
    vx, vy, vz = point
    # Rotate roll, then pitch, then yaw, matching SDF's fixed-axis RPY.
    vy, vz = math.cos(roll) * vy - math.sin(roll) * vz, math.sin(roll) * vy + math.cos(roll) * vz
    vx, vz = math.cos(pitch) * vx + math.sin(pitch) * vz, -math.sin(pitch) * vx + math.cos(pitch) * vz
    vx, vy = math.cos(yaw) * vx - math.sin(yaw) * vy, math.sin(yaw) * vx + math.cos(yaw) * vy
    return x + vx, y + vy, z + vz


class TerrainMesh:
    """Sample the actual OBJ triangles, independently of the generator's height helper."""

    def __init__(self, world, world_path):
        model = world.find("./model[@name='bp1_heightmap_terrain']")
        self.pose = numbers(model, "pose")
        self.points = vertices(world_path.parent / model.findtext("./link/visual/geometry/mesh/uri"))
        self.count = math.isqrt(len(self.points))
        assert self.count ** 2 == len(self.points)
        self.x_min, self.y_max, _ = self.points[0]
        self.dx = (self.points[self.count - 1][0] - self.x_min) / (self.count - 1)
        self.dy = (self.y_max - self.points[-1][1]) / (self.count - 1)

    def height(self, x, y):
        column = (x - self.pose[0] - self.x_min) / self.dx
        row = (self.y_max - y + self.pose[1]) / self.dy
        left = max(0, min(self.count - 2, math.floor(column)))
        top = max(0, min(self.count - 2, math.floor(row)))
        u, v = column - left, row - top
        a = self.points[top * self.count + left][2]
        b = self.points[top * self.count + left + 1][2]
        c = self.points[(top + 1) * self.count + left][2]
        d = self.points[(top + 1) * self.count + left + 1][2]
        if u + v <= 1:
            return self.pose[2] + a + u * (b - a) + v * (c - a)
        return self.pose[2] + d + (1 - u) * (c - d) + (1 - v) * (b - d)


def collision_footprint(model):
    center = numbers(model, "./link/collision/pose") if model.find("./link/collision/pose") is not None else (0,) * 6
    width, length, _ = numbers(model, "./link/collision/geometry/box/size")
    pose = numbers(model, "pose")
    return [transform(transform((sx * width / 2, sy * length / 2, 0), center), pose)[:2]
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]


def overlaps(first, second):
    for polygon in (first, second):
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            axis = (a[1] - b[1], b[0] - a[0])
            one = [x * axis[0] + y * axis[1] for x, y in first]
            two = [x * axis[0] + y * axis[1] for x, y in second]
            if max(one) < min(two) or max(two) < min(one):
                return False
    return True


class WorldGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.worlds = []
        for variant in generator.VARIANTS:
            path = generator.WORLDS_ROOT / f"lunabotics_terrain_{variant.number:02d}.sdf"
            world = ET.parse(path).getroot().find("world")
            cls.worlds.append((variant, path, world, TerrainMesh(world, path)))
        cls.flat_path = generator.WORLDS_ROOT / "lunabotics_arena.sdf"
        cls.flat = ET.parse(cls.flat_path).getroot().find("world")
        cls.rock_vertices = vertices(generator.ASSETS_ROOT / "shared/lunar_rock.obj")

    def test_height_sampler_uses_gazebo_not_design_coordinates(self):
        for pixel, expected in ((0, 0.0), (120, .40), (255, .85)):
            pixels = [pixel] * generator.IMAGE_SIZE ** 2
            for x, y in ((0, 0), (3.44, 2.5), (6.88, 5.0)):
                self.assertAlmostEqual(generator.gazebo_surface_height(pixels, x, y), expected)

    def test_asset_uris_resolve(self):
        for path, world in [(p, w) for _, p, w, _ in self.worlds] + [(self.flat_path, self.flat)]:
            with self.subTest(world=path.name):
                names = [model.get("name") for model in world.findall("model")]
                self.assertEqual(len(names), len(set(names)))
                for uri in world.findall(".//uri"):
                    self.assertTrue((path.parent / uri.text).is_file(), uri.text)

    def test_render_mesh_matches_collision_heightmap(self):
        for _, path, world, mesh in self.worlds:
            model = world.find("./model[@name='bp1_heightmap_terrain']")
            heightmap = model.find("./link/collision/geometry/heightmap")
            size = numbers(heightmap, "size")
            with Image.open(path.parent / heightmap.findtext("uri")) as image:
                peak = image.getextrema()[1]
                for x, y, z in mesh.points:
                    column = round((x / size[0] + .5) * (image.width - 1))
                    row = round((.5 - y / size[1]) * (image.height - 1))
                    # Gazebo's DART heightmap scales by the image peak, not 255.
                    collision_z = image.getpixel((column, row)) * size[2] / peak
                    self.assertAlmostEqual(z, collision_z, delta=1e-6,
                                           msg=path.name)

    def test_spawn_collision_matches_visible_ground_in_every_terrain(self):
        for _, path, world, mesh in self.worlds:
            with self.subTest(world=path.name):
                model = world.find("./model[@name='bp1_heightmap_terrain']")
                heightmap = model.find("./link/collision/geometry/heightmap")
                size = numbers(heightmap, "size")
                with Image.open(path.parent / heightmap.findtext("uri")) as image:
                    peak = image.getextrema()[1]
                    self.assertGreater(peak, 0)
                    self.assertLess(peak, 255, "fixture must exercise peak normalization")
                    column = round(1.0 / size[0] * (image.width - 1))
                    row = round((1.0 - .75 / size[1]) * (image.height - 1))
                    collision_z = numbers(model, "pose")[2] + image.getpixel((column, row)) * size[2] / peak
                self.assertAlmostEqual(collision_z, .40, delta=1e-6)
                self.assertAlmostEqual(collision_z, mesh.height(1.0, .75), delta=1e-6)

    def test_rocks_are_visible_not_buried_or_floating(self):
        cases = [(p, w, m.height) for _, p, w, m in self.worlds]
        cases.append((self.flat_path, self.flat, lambda x, y: 0.0))
        for path, world, surface in cases:
            rocks = [model for model in world.findall("model") if model.get("name").startswith("rock_")]
            self.assertEqual(len(rocks), 13 if path != self.flat_path else 7)
            for model in rocks:
                with self.subTest(world=path.name, rock=model.get("name")):
                    scale = numbers(model, "./link/visual/geometry/mesh/scale")
                    pose = numbers(model, "pose")
                    local = [tuple(value * factor for value, factor in zip(point, scale)) for point in self.rock_vertices]
                    points = [transform(point, pose) for point in local]
                    clearances = [z - surface(x, y) for x, y, z in points]
                    self.assertGreater(max(clearances), .12, "rock buried below visible terrain")
                    self.assertLess(min(clearances), .04, "rock floating above terrain")
                    self.assertAlmostEqual(pose[2] - surface(*pose[:2]), .02, delta=.006)
                    center = numbers(model, "./link/collision/pose")[:3]
                    size = numbers(model, "./link/collision/geometry/box/size")
                    for axis in range(3):
                        self.assertAlmostEqual(min(v[axis] for v in local), center[axis] - size[axis] / 2, delta=2e-6)
                        self.assertAlmostEqual(max(v[axis] for v in local), center[axis] + size[axis] / 2, delta=2e-6)

    def test_rock_sizes_vary_in_every_world(self):
        for _, path, world, _ in self.worlds:
            widths, heights = [], []
            for name in generator.ROCK_NAMES:
                model = world.find(f"./model[@name='{name}']")
                width, _, height = numbers(model, "./link/collision/geometry/box/size")
                widths.append(round(width, 3))
                heights.append(height)
            with self.subTest(world=path.name):
                self.assertEqual(len(set(widths)), 6)
                self.assertGreater(max(heights) - min(heights), .1)
        radii = [float(model.findtext("./link/visual/geometry/sphere/radius"))
                 for model in self.flat.findall("model") if model.get("name").startswith("boulder_")]
        self.assertEqual(len(set(radii)), 6)
        for model in self.flat.findall("model"):
            if model.get("name").startswith("boulder_"):
                self.assertEqual(model.findtext("./link/visual/geometry/sphere/radius"),
                                 model.findtext("./link/collision/geometry/sphere/radius"))

    def test_holes_have_three_real_sizes_and_depths(self):
        for variant, path, _, mesh in self.worlds:
            depths = []
            self.assertEqual(sorted(round(2 * crater[2], 2) for crater in variant.craters), [.30, .40, .50])
            for x, y, radius, _, _ in variant.craters:
                rim = [mesh.height(x + radius * math.cos(i * math.tau / 64), y + radius * math.sin(i * math.tau / 64))
                       for i in range(64)]
                depth = sum(rim) / len(rim) - mesh.height(x, y)
                self.assertGreater(depth, .10, (path.name, x, y, depth))
                depths.append(depth)
            self.assertGreater(max(depths) - min(depths), .10)
        for index, radius in enumerate((.15, .20, .25), start=1):
            model = self.flat.find(f"./model[@name='crater_obstacle_{index}']")
            self.assertEqual(float(model.findtext("./link/visual/geometry/cylinder/radius")), radius)
            self.assertEqual(float(model.findtext("./link/collision/geometry/cylinder/radius")), radius)

    def test_barrier_has_no_planar_gap_from_south_duct_to_column(self):
        for path, world in [(p, w) for _, p, w, _ in self.worlds] + [(self.flat_path, self.flat)]:
            rocks = [world.find(f"./model[@name='{name}']") for name in generator.CONSTRUCTION_ROCK_NAMES]
            polygons = [collision_footprint(model) for model in rocks]
            south = world.find("./model[@name='perimeter_ducts']/link/collision[@name='south_collision']")
            x, y, *_ = numbers(south, "pose")
            radius = float(south.findtext("./geometry/cylinder/radius"))
            half_length = float(south.findtext("./geometry/cylinder/length")) / 2
            duct = [(x - half_length, y - radius), (x + half_length, y - radius),
                    (x + half_length, y + radius), (x - half_length, y + radius)]
            column = collision_footprint(world.find("./model[@name='central_support_column']"))
            chain = [duct] + polygons + [column]
            for index, (first, second) in enumerate(zip(chain, chain[1:])):
                self.assertTrue(overlaps(first, second), (path.name, "barrier gap", index))


if __name__ == "__main__":
    unittest.main()
