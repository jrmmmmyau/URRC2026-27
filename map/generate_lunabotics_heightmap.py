#!/usr/bin/env python3
"""Generate deterministic BP-1 height, diffuse, and normal maps for Gazebo.

The height output is a 257 x 257 grayscale PNG (2^n + 1, as expected by
Gazebo heightmaps). Pixel intensity maps linearly to [-0.40 m, +0.15 m].
"""

from __future__ import annotations

import math
import random
from pathlib import Path

from PIL import Image, ImageFilter


ARENA_X = 6.88
ARENA_Y = 5.00
IMAGE_SIZE = 257
MIN_HEIGHT = -0.40
HEIGHT_RANGE = 0.55
RANDOM_SEED = 2027

# x, y, rim radius, center depth, rim height (metres)
CRATERS = (
    (3.25, 3.80, 0.24, 0.28, 0.055),
    (4.55, 2.85, 0.25, 0.34, 0.065),
    (6.00, 3.70, 0.23, 0.30, 0.060),
)


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if edge0 == edge1:
        return float(value >= edge1)
    t = max(0.0, min(1.0, (value - edge0) / (edge1 - edge0)))
    return t * t * (3.0 - 2.0 * t)


def crater_height(x: float, y: float) -> float:
    height = 0.0
    for cx, cy, radius, depth, rim_height in CRATERS:
        distance = math.hypot(x - cx, y - cy)
        bowl = -depth * math.exp(-((distance / (0.62 * radius)) ** 4))
        rim = rim_height * math.exp(-(((distance - radius) / (0.16 * radius)) ** 2))
        height += bowl + rim
    return height


def generate(output: Path) -> tuple[Path, Path, Path]:
    random.seed(RANDOM_SEED)

    # Upscaled, blurred seeded noise gives non-periodic broad surface variation.
    noise_size = 17
    noise = Image.new("L", (noise_size, noise_size))
    noise.putdata([random.randrange(256) for _ in range(noise_size * noise_size)])
    noise = noise.resize((IMAGE_SIZE, IMAGE_SIZE), Image.Resampling.BICUBIC)
    noise = noise.filter(ImageFilter.GaussianBlur(radius=5.0))
    noise_pixels = list(noise.get_flattened_data())

    pixels: list[int] = []
    for row in range(IMAGE_SIZE):
        # Image rows run top-to-bottom; Gazebo world +y runs bottom-to-top.
        y = ARENA_Y * (1.0 - row / (IMAGE_SIZE - 1))
        for column in range(IMAGE_SIZE):
            x = ARENA_X * column / (IMAGE_SIZE - 1)
            noise_value = (noise_pixels[row * IMAGE_SIZE + column] - 127.5) / 127.5

            undulation = (
                0.022 * math.sin(1.35 * x + 0.55 * y)
                + 0.016 * math.sin(2.10 * y - 0.35 * x)
                + 0.013 * math.sin(2.85 * x + 1.70 * y)
                + 0.022 * noise_value
            )
            elevation = undulation + crater_height(x, y)

            # Preserve level operational surfaces with a short blended margin.
            start_distance = max(x - 2.00, y - 1.50)
            start_blend = smoothstep(0.0, 0.22, start_distance)
            if x <= 2.22 and y <= 1.72:
                elevation *= start_blend

            berm_dx = max(4.28 - x, 0.0, x - 6.48)
            berm_dy = max(0.45 - y, 0.0, y - 1.35)
            berm_distance = math.hypot(berm_dx, berm_dy)
            if berm_distance <= 0.22:
                elevation *= smoothstep(0.0, 0.22, berm_distance)
            elif 4.28 <= x <= 6.48 and 0.45 <= y <= 1.35:
                elevation = 0.0

            # Meet the perimeter ducting at nominal grade without gaps.
            edge_distance = min(x, ARENA_X - x, y, ARENA_Y - y)
            elevation *= smoothstep(0.0, 0.20, edge_distance)

            normalized = (elevation - MIN_HEIGHT) / HEIGHT_RANGE
            pixels.append(round(255 * max(0.0, min(1.0, normalized))))

    output.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("L", (IMAGE_SIZE, IMAGE_SIZE))
    image.putdata(pixels)
    image.save(output, optimize=True)

    # OGRE heightmaps expect an explicit diffuse and normal texture layer.
    # The diffuse texture resembles dry crushed basalt without external assets.
    diffuse_pixels: list[tuple[int, int, int]] = []
    elevations = [MIN_HEIGHT + HEIGHT_RANGE * value / 255.0 for value in pixels]
    for index, elevation in enumerate(elevations):
        noise_value = (noise_pixels[index] - 127.5) / 127.5
        shade = round(15.0 * noise_value + 18.0 * min(0.0, elevation))
        diffuse_pixels.append(
            (
                max(0, min(255, 91 + shade)),
                max(0, min(255, 82 + shade)),
                max(0, min(255, 70 + shade)),
            )
        )

    diffuse_path = output.with_name("lunabotics_regolith_diffuse.png")
    diffuse = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE))
    diffuse.putdata(diffuse_pixels)
    diffuse.save(diffuse_path, optimize=True)

    # Derive tangent-space normals from the physical elevation gradient so
    # crater rims and gentle undulations remain legible under arena lighting.
    dx = ARENA_X / (IMAGE_SIZE - 1)
    dy = ARENA_Y / (IMAGE_SIZE - 1)
    normal_pixels: list[tuple[int, int, int]] = []
    for row in range(IMAGE_SIZE):
        row_before = max(0, row - 1)
        row_after = min(IMAGE_SIZE - 1, row + 1)
        for column in range(IMAGE_SIZE):
            column_before = max(0, column - 1)
            column_after = min(IMAGE_SIZE - 1, column + 1)
            dz_dx = (
                elevations[row * IMAGE_SIZE + column_after]
                - elevations[row * IMAGE_SIZE + column_before]
            ) / ((column_after - column_before) * dx)
            # Image rows increase toward world -y.
            dz_dy = -(
                elevations[row_after * IMAGE_SIZE + column]
                - elevations[row_before * IMAGE_SIZE + column]
            ) / ((row_after - row_before) * dy)
            nx, ny, nz = -dz_dx, -dz_dy, 1.0
            magnitude = math.sqrt(nx * nx + ny * ny + nz * nz)
            normal_pixels.append(
                tuple(round(127.5 * (component / magnitude + 1.0)) for component in (nx, ny, nz))
            )

    normal_path = output.with_name("lunabotics_regolith_normal.png")
    normal = Image.new("RGB", (IMAGE_SIZE, IMAGE_SIZE))
    normal.putdata(normal_pixels)
    normal.save(normal_path, optimize=True)
    return output, diffuse_path, normal_path


if __name__ == "__main__":
    destination = Path(__file__).resolve().parent / "heightmaps" / "lunabotics_moon_terrain.png"
    for generated_path in generate(destination):
        print(generated_path)
