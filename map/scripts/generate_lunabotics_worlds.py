#!/usr/bin/env python3
"""Generate six deterministic terrain worlds and refresh flat-world obstacles.

Broad terrain grades are limited to 15 degrees. Competition crater walls are
discrete obstacles and intentionally excluded from that broad-grade limit.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
import random
from pathlib import Path
from xml.etree import ElementTree as ET

from PIL import Image, ImageFilter


ARENA_X = 6.88
ARENA_Y = 5.00
IMAGE_SIZE = 257
MESH_STEP = 2
MIN_HEIGHT = -0.40
HEIGHT_RANGE = 0.85
MAX_BROAD_SLOPE_DEGREES = 15.0
TARGET_BROAD_SLOPE_DEGREES = 12.5  # margin for 8-bit height quantization

MAP_ROOT = Path(__file__).resolve().parent.parent
ASSETS_ROOT = MAP_ROOT / "assets"
WORLDS_ROOT = MAP_ROOT / "worlds"
TEMPLATE_PATH = MAP_ROOT / "templates" / "lunabotics_terrain_template.sdf"

ROCK_NAMES = (
    "rock_obstacle_1", "rock_obstacle_2", "rock_obstacle_3",
    "rock_obstacle_4", "rock_excavation_1", "rock_excavation_2",
)
CONSTRUCTION_ROCK_NAMES = tuple(f"rock_construction_{index}" for index in range(1, 8))
# Join the south perimeter duct to the column's south face (y=2.275 m).
CONSTRUCTION_ROCK_POSITIONS = tuple((3.44, .28 + .31 * index) for index in range(7))
# Local width, length, and height above the rock origin, in metres.
ROCK_SIZES = (
    (.30, .27, .15), (.35, .31, .20), (.40, .35, .26),
    (.32, .28, .17), (.38, .33, .23), (.34, .30, .19),
)
CONSTRUCTION_ROCK_SIZES = (
    (.38, .36, .24), (.36, .35, .22), (.40, .37, .28),
    (.37, .36, .23), (.39, .35, .26), (.38, .36, .25),
    (.40, .37, .27),
)


@dataclass(frozen=True)
class Variant:
    number: int
    title: str
    seed: int
    profile: str
    # crater: x, y, rim radius, center depth, rim height
    craters: tuple[tuple[float, float, float, float, float], ...]
    # rock: x, y, roll, pitch, yaw
    rocks: tuple[tuple[float, float, float, float, float], ...]


VARIANTS = (
    Variant(1, "Central Ridge", 202701, "ridge",
            ((3.25, 3.80, .15, .16, .035), (4.55, 2.85, .20, .25, .050), (6.00, 3.70, .25, .34, .065)),
            ((3.05, 4.48, .08, -.05, .45), (3.95, 2.05, -.06, .04, 1.20), (5.35, 4.30, .04, .07, 2.10), (6.25, 2.20, -.05, -.04, .85), (.70, 3.70, .03, -.06, 1.60), (1.82, 2.70, -.04, .05, 2.65))),
    Variant(2, "Rolling Dunes", 202702, "waves",
            ((3.10, 3.25, .20, .24, .050), (4.70, 4.05, .25, .32, .065), (6.05, 2.75, .15, .15, .035)),
            ((2.90, 4.35, .05, -.06, .30), (4.10, 2.05, -.04, .06, 1.55), (5.45, 3.30, .06, .03, 2.40), (6.15, 4.35, -.05, -.05, .95), (.65, 2.70, .03, -.04, 1.30), (1.75, 4.10, -.05, .04, 2.80))),
    Variant(3, "Diagonal Ridge", 202703, "diagonal",
            ((3.15, 4.10, .25, .33, .065), (4.85, 2.40, .15, .16, .035), (6.10, 3.55, .20, .25, .050)),
            ((2.85, 2.00, .06, -.04, .60), (4.05, 3.50, -.05, .05, 1.40), (5.30, 4.40, .04, .06, 2.25), (6.15, 2.10, -.06, -.03, .80), (.75, 4.10, .03, -.05, 1.75), (1.85, 2.60, -.04, .05, 2.55))),
    Variant(4, "Twin Mounds", 202704, "twins",
            ((2.95, 2.90, .15, .15, .035), (4.40, 4.15, .25, .34, .065), (5.95, 2.65, .20, .24, .050)),
            ((3.00, 4.40, .07, -.05, .35), (4.15, 2.05, -.05, .05, 1.65), (5.30, 3.60, .05, .06, 2.30), (6.30, 4.20, -.04, -.06, 1.05), (.60, 3.30, .04, -.04, 1.45), (1.90, 4.40, -.05, .03, 2.75))),
    Variant(5, "Broad Basin", 202705, "basin",
            ((3.10, 3.90, .20, .25, .050), (4.70, 2.00, .15, .16, .035), (6.15, 3.10, .25, .32, .065)),
            ((2.90, 2.10, .05, -.05, .50), (4.05, 3.00, -.06, .04, 1.35), (5.35, 4.25, .04, .07, 2.50), (6.25, 4.50, -.05, -.04, .90), (.80, 2.80, .03, -.06, 1.55), (1.70, 4.00, -.04, .05, 2.70))),
    Variant(6, "Cross Slope", 202706, "cross",
            ((3.00, 2.00, .25, .33, .065), (4.60, 3.65, .20, .24, .050), (6.00, 2.40, .15, .15, .035)),
            ((3.00, 4.40, .06, -.05, .40), (4.00, 2.60, -.05, .06, 1.50), (5.40, 2.00, .05, .04, 2.20), (6.20, 4.10, -.06, -.04, 1.00), (.70, 3.70, .03, -.05, 1.65), (1.80, 2.60, -.04, .05, 2.60))),
)


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge0 == edge1:
        return float(value >= edge1)
    t = max(0.0, min(1.0, (value - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def gaussian(x: float, y: float, cx: float, cy: float, sx: float, sy: float) -> float:
    return math.exp(-(((x - cx) / sx) ** 2 + ((y - cy) / sy) ** 2))


def relief_height(profile: str, x: float, y: float) -> float:
    """Return an unscaled broad relief profile inside the obstacle region."""
    u = (x - 2.50) / (6.88 - 2.50)
    v = (y - 1.50) / (5.00 - 1.50)
    # Wide fades let the terrain gain visible height while remaining within
    # the requested grade limit at zone boundaries.
    x_window = smoothstep(2.50, 3.25, x) * (1.0 - smoothstep(6.13, 6.88, x))
    y_window = smoothstep(1.50, 2.25, y) * (1.0 - smoothstep(3.85, 4.60, y))
    window = x_window * y_window

    if profile == "ridge":
        value = .22 * math.sin(math.pi * u) + .08 * gaussian(u, v, .72, .30, .18, .22) - .07 * gaussian(u, v, .22, .30, .16, .24)
    elif profile == "waves":
        value = .15 * math.sin(2.0 * math.pi * u + .35) * math.cos(math.pi * v) + .07 * math.sin(2.0 * math.pi * v)
    elif profile == "diagonal":
        distance = v - (.15 + .68 * u)
        value = .23 * math.exp(-((distance / .16) ** 2)) - .09 * math.exp(-(((distance - .30) / .20) ** 2))
    elif profile == "twins":
        value = .20 * gaussian(u, v, .28, .62, .22, .25) + .19 * gaussian(u, v, .76, .30, .22, .22) - .11 * gaussian(u, v, .52, .48, .20, .20)
    elif profile == "basin":
        radius = math.hypot((u - .55) / .85, v - .50)
        value = -.19 * gaussian(u, v, .55, .50, .34, .30) + .12 * math.exp(-(((radius - .42) / .14) ** 2))
    elif profile == "cross":
        value = .18 * (u - .50) + .11 * math.sin(math.pi * v) * math.cos(2.0 * math.pi * u)
    else:
        raise ValueError(f"Unknown terrain profile: {profile}")
    return window * value


def flatten_factor(x: float, y: float) -> float:
    """Blend start, berm, and perimeter infrastructure to nominal grade."""
    factor = 1.0
    if x <= 2.24 and y <= 1.74:
        factor *= smoothstep(0.0, .24, max(x - 2.00, y - 1.50))

    berm_dx = max(4.28 - x, 0.0, x - 6.48)
    berm_dy = max(.45 - y, 0.0, y - 1.35)
    berm_distance = math.hypot(berm_dx, berm_dy)
    if berm_distance <= .24:
        factor *= smoothstep(0.0, .24, berm_distance)

    edge_distance = min(x, ARENA_X - x, y, ARENA_Y - y)
    factor *= smoothstep(0.0, .20, edge_distance)
    return factor


def crater_height(craters: tuple[tuple[float, float, float, float, float], ...], x: float, y: float) -> float:
    height = 0.0
    for cx, cy, radius, depth, rim_height in craters:
        distance = math.hypot(x - cx, y - cy)
        height -= depth * math.exp(-((distance / (.62 * radius)) ** 4))
        height += rim_height * math.exp(-(((distance - radius) / (.16 * radius)) ** 2))
    return height


def max_gradient(elevations: list[float]) -> float:
    dx = ARENA_X / (IMAGE_SIZE - 1)
    dy = ARENA_Y / (IMAGE_SIZE - 1)
    maximum = 0.0
    for row in range(1, IMAGE_SIZE - 1):
        for column in range(1, IMAGE_SIZE - 1):
            index = row * IMAGE_SIZE + column
            gx = (elevations[index + 1] - elevations[index - 1]) / (2.0 * dx)
            gy = (elevations[index - IMAGE_SIZE] - elevations[index + IMAGE_SIZE]) / (2.0 * dy)
            maximum = max(maximum, math.hypot(gx, gy))
    return maximum


def build_base_surface(variant: Variant, noise_pixels: list[int]) -> tuple[list[float], float, float]:
    undulations: list[float] = []
    relief: list[float] = []
    factors: list[float] = []
    for row in range(IMAGE_SIZE):
        y = ARENA_Y * (1.0 - row / (IMAGE_SIZE - 1))
        for column in range(IMAGE_SIZE):
            x = ARENA_X * column / (IMAGE_SIZE - 1)
            noise = (noise_pixels[row * IMAGE_SIZE + column] - 127.5) / 127.5
            # Keep micro-relief small so the 15-degree budget is spent on the
            # world-specific ridge / dune / basin rather than pixel-scale noise.
            undulations.append(.004 * math.sin(1.35 * x + .55 * y) + .003 * math.sin(2.10 * y - .35 * x) + .002 * math.sin(2.85 * x + 1.70 * y) + .003 * noise)
            relief.append(relief_height(variant.profile, x, y))
            factors.append(flatten_factor(x, y))

    target = math.tan(math.radians(TARGET_BROAD_SLOPE_DEGREES))
    scale = 1.0
    surface: list[float] = []
    for _ in range(10):
        surface = [(u + scale * r) * f for u, r, f in zip(undulations, relief, factors)]
        gradient = max_gradient(surface)
        if gradient <= 1e-9:
            break
        ratio = target / gradient
        if .98 <= ratio <= 1.02:
            break
        scale *= max(.45, min(1.75, ratio))
    return surface, scale, math.degrees(math.atan(max_gradient(surface)))


def build_normals(elevations: list[float]) -> tuple[list[tuple[int, int, int]], list[tuple[float, float, float]]]:
    dx = ARENA_X / (IMAGE_SIZE - 1)
    dy = ARENA_Y / (IMAGE_SIZE - 1)
    pixels: list[tuple[int, int, int]] = []
    vectors: list[tuple[float, float, float]] = []
    for row in range(IMAGE_SIZE):
        rb, ra = max(0, row - 1), min(IMAGE_SIZE - 1, row + 1)
        for column in range(IMAGE_SIZE):
            cb, ca = max(0, column - 1), min(IMAGE_SIZE - 1, column + 1)
            dz_dx = (elevations[row * IMAGE_SIZE + ca] - elevations[row * IMAGE_SIZE + cb]) / ((ca - cb) * dx)
            dz_dy = -(elevations[ra * IMAGE_SIZE + column] - elevations[rb * IMAGE_SIZE + column]) / ((ra - rb) * dy)
            nx, ny, nz = -dz_dx, -dz_dy, 1.0
            magnitude = math.sqrt(nx * nx + ny * ny + nz * nz)
            vector = (nx / magnitude, ny / magnitude, nz / magnitude)
            vectors.append(vector)
            pixels.append(tuple(round(127.5 * (component + 1.0)) for component in vector))
    return pixels, vectors


def write_mesh(path: Path, elevations: list[float], normals: list[tuple[float, float, float]]) -> None:
    selected = list(range(0, IMAGE_SIZE, MESH_STEP))
    if selected[-1] != IMAGE_SIZE - 1:
        selected.append(IMAGE_SIZE - 1)
    lines = ["# Lunabotics terrain generated from matching collision heightmap", "o lunabotics_terrain"]
    for row in selected:
        y = ARENA_Y / 2.0 - ARENA_Y * row / (IMAGE_SIZE - 1)
        for column in selected:
            x = -ARENA_X / 2.0 + ARENA_X * column / (IMAGE_SIZE - 1)
            z = elevations[row * IMAGE_SIZE + column] - MIN_HEIGHT
            lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
    for row in selected:
        for column in selected:
            nx, ny, nz = normals[row * IMAGE_SIZE + column]
            lines.append(f"vn {nx:.7f} {ny:.7f} {nz:.7f}")
    columns = len(selected)
    for row in range(len(selected) - 1):
        for column in range(columns - 1):
            a = row * columns + column + 1
            b, c, d = a + 1, a + columns, a + columns + 1
            lines.append(f"f {a}//{a} {c}//{c} {b}//{b}")
            lines.append(f"f {b}//{b} {c}//{c} {d}//{d}")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def generate_assets(variant: Variant) -> tuple[list[int], float, float]:
    random.seed(variant.seed)
    noise = Image.new("L", (17, 17))
    noise.putdata([random.randrange(256) for _ in range(17 * 17)])
    noise = noise.resize((IMAGE_SIZE, IMAGE_SIZE), Image.Resampling.BICUBIC).filter(ImageFilter.GaussianBlur(radius=5.0))
    noise_pixels = list(noise.get_flattened_data())
    base, scale, broad_angle = build_base_surface(variant, noise_pixels)

    elevations: list[float] = []
    for row in range(IMAGE_SIZE):
        y = ARENA_Y * (1.0 - row / (IMAGE_SIZE - 1))
        for column in range(IMAGE_SIZE):
            x = ARENA_X * column / (IMAGE_SIZE - 1)
            value = base[row * IMAGE_SIZE + column] + crater_height(variant.craters, x, y)
            elevations.append(max(MIN_HEIGHT, min(MIN_HEIGHT + HEIGHT_RANGE, value)))
    pixels = [round(255 * (value - MIN_HEIGHT) / HEIGHT_RANGE) for value in elevations]
    quantized = [MIN_HEIGHT + HEIGHT_RANGE * value / 255.0 for value in pixels]

    asset_dir = ASSETS_ROOT / f"terrain_{variant.number:02d}"
    asset_dir.mkdir(parents=True, exist_ok=True)
    heightmap = Image.new("L", (IMAGE_SIZE, IMAGE_SIZE)); heightmap.putdata(pixels); heightmap.save(asset_dir / "heightmap.png", optimize=True)
    diffuse_pixels = []
    for index, elevation in enumerate(quantized):
        noise_value = (noise_pixels[index] - 127.5) / 127.5
        shade = round(15.0 * noise_value + 18.0 * min(0.0, elevation))
        diffuse_pixels.append((max(0, min(255, 91 + shade)), max(0, min(255, 82 + shade)), max(0, min(255, 70 + shade))))
    diffuse = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE)); diffuse.putdata(diffuse_pixels); diffuse.save(asset_dir / "diffuse.png", optimize=True)
    normal_pixels, normal_vectors = build_normals(quantized)
    normal = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE)); normal.putdata(normal_pixels); normal.save(asset_dir / "normal.png", optimize=True)
    write_mesh(asset_dir / "terrain.obj", quantized, normal_vectors)
    return pixels, scale, broad_angle


def gazebo_surface_height(pixels: list[int], x: float, y: float) -> float:
    """Sample world z, not design elevation (which is 0.40 m lower).

    Gazebo interprets black as z=0 and white as z=HEIGHT_RANGE. The OBJ
    uses that same frame; adding MIN_HEIGHT here would bury every rock.
    """
    column = x / ARENA_X * (IMAGE_SIZE - 1)
    row = (1.0 - y / ARENA_Y) * (IMAGE_SIZE - 1)
    left, top = math.floor(column), math.floor(row)
    right, bottom = min(left + 1, IMAGE_SIZE - 1), min(top + 1, IMAGE_SIZE - 1)
    fx, fy = column - left, row - top
    upper = (1.0 - fx) * pixels[top * IMAGE_SIZE + left] + fx * pixels[top * IMAGE_SIZE + right]
    lower = (1.0 - fx) * pixels[bottom * IMAGE_SIZE + left] + fx * pixels[bottom * IMAGE_SIZE + right]
    return HEIGHT_RANGE * ((1.0 - fy) * upper + fy * lower) / 255.0


@lru_cache(maxsize=1)
def rock_mesh_bounds() -> tuple[tuple[float, float], ...]:
    lines = (ASSETS_ROOT / "shared" / "lunar_rock.obj").read_text().splitlines()
    vertices = [tuple(map(float, line.split()[1:4])) for line in lines if line.startswith("v ")]
    return tuple((min(axis), max(axis)) for axis in zip(*vertices))


def resize_rock(model: ET.Element, dimensions: tuple[float, float, float]) -> None:
    """Scale the visual and its safe box collision from the same OBJ bounds."""
    bounds = rock_mesh_bounds()
    width, length, height = dimensions
    scales = (width / (bounds[0][1] - bounds[0][0]),
              length / (bounds[1][1] - bounds[1][0]), height / bounds[2][1])
    mesh_scale = model.find("./link/visual/geometry/mesh/scale")
    collision_pose = model.find("./link/collision/pose")
    collision_size = model.find("./link/collision/geometry/box/size")
    assert mesh_scale is not None and collision_pose is not None and collision_size is not None
    mesh_scale.text = " ".join(f"{value:.6f}" for value in scales)
    center = [(low + high) * scale / 2.0 for (low, high), scale in zip(bounds, scales)]
    size = [(high - low) * scale for (low, high), scale in zip(bounds, scales)]
    collision_pose.text = " ".join(f"{value:.6f}" for value in center) + " 0 0 0"
    collision_size.text = " ".join(f"{value:.6f}" for value in size)


def generate_world(variant: Variant, pixels: list[int]) -> Path:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(TEMPLATE_PATH, parser=parser)
    world = tree.getroot().find("world")
    assert world is not None
    world.insert(0, ET.Comment(f" Variant {variant.number}: {variant.title}; broad grades <= {MAX_BROAD_SLOPE_DEGREES:.0f} degrees "))
    terrain = world.find("./model[@name='bp1_heightmap_terrain']")
    assert terrain is not None
    collision_uri = terrain.find("./link/collision/geometry/heightmap/uri")
    visual_uri = terrain.find("./link/visual/geometry/mesh/uri")
    assert collision_uri is not None and visual_uri is not None
    collision_uri.text = f"../assets/terrain_{variant.number:02d}/heightmap.png"
    visual_uri.text = f"../assets/terrain_{variant.number:02d}/terrain.obj"

    for index, (name, rock) in enumerate(zip(ROCK_NAMES, variant.rocks)):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        x, y, roll, pitch, yaw = rock
        surface = gazebo_surface_height(pixels, x, y)
        pose = model.find("pose")
        assert pose is not None
        pose.text = f"{x:.2f} {y:.2f} {surface + .020:.3f} {roll:.2f} {pitch:.2f} {yaw:.2f}"
        resize_rock(model, ROCK_SIZES[(index + variant.number - 1) % len(ROCK_SIZES)])

    for name, (x, y), size in zip(CONSTRUCTION_ROCK_NAMES, CONSTRUCTION_ROCK_POSITIONS, CONSTRUCTION_ROCK_SIZES):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        surface = gazebo_surface_height(pixels, x, y)
        pose = model.find("pose")
        assert pose is not None
        orientation = pose.text.split()[3:]
        pose.text = f"{x:.2f} {y:.2f} {surface + .020:.3f} {' '.join(orientation)}"
        resize_rock(model, size)

    ET.indent(tree, space="  ")
    output = WORLDS_ROOT / f"lunabotics_terrain_{variant.number:02d}.sdf"
    tree.write(output, encoding="UTF-8", xml_declaration=True)
    return output


def update_flat_world() -> Path:
    """Keep the flat arena's obstacles consistent without raising its grade."""
    output = WORLDS_ROOT / "lunabotics_arena.sdf"
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(output, parser=parser)
    world = tree.getroot().find("world")
    assert world is not None
    for name, (x, y), size in zip(CONSTRUCTION_ROCK_NAMES, CONSTRUCTION_ROCK_POSITIONS, CONSTRUCTION_ROCK_SIZES):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        pose = model.find("pose")
        assert pose is not None
        orientation = pose.text.split()[3:]
        pose.text = f"{x:.2f} {y:.2f} 0.020 {' '.join(orientation)}"
        resize_rock(model, size)

    for name, size in zip(ROCK_NAMES, ROCK_SIZES):
        model = world.find(f"./model[@name='{name.replace('rock_', 'boulder_')}']")
        assert model is not None
        radius = size[0] / 2.0
        for element in model.findall(".//sphere/radius"):
            element.text = f"{radius:.3f}"
        pose = model.find("pose")
        assert pose is not None
        values = pose.text.split()
        values[2] = f"{radius - .04:.3f}"
        pose.text = " ".join(values)

    for index, radius in enumerate((.15, .20, .25), start=1):
        model = world.find(f"./model[@name='crater_obstacle_{index}']")
        assert model is not None
        for element in model.findall(".//cylinder/radius"):
            element.text = f"{radius:.3f}"

    ET.indent(tree, space="  ")
    tree.write(output, encoding="UTF-8", xml_declaration=True)
    return output


def main() -> None:
    WORLDS_ROOT.mkdir(parents=True, exist_ok=True)
    for variant in VARIANTS:
        pixels, scale, broad_angle = generate_assets(variant)
        world = generate_world(variant, pixels)
        if broad_angle > MAX_BROAD_SLOPE_DEGREES:
            raise RuntimeError(f"Variant {variant.number} broad slope is {broad_angle:.2f} degrees")
        print(f"{world.name}: {variant.title}; relief scale={scale:.3f}; broad slope={broad_angle:.2f} deg")
    print(f"{update_flat_world().name}: refreshed barrier, rock sizes, and crater markers")


if __name__ == "__main__":
    main()
