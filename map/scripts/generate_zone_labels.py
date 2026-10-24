#!/usr/bin/env python3
"""Generate simple block-letter OBJ labels for the Lunabotics arena floor."""

from __future__ import annotations

from pathlib import Path


FONT = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "11001", "10101", "10011", "10011", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "X": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
}

LABELS = {
    "start": ("START", 0.040),
    "excavation": ("EXCAVATION", 0.035),
    "obstacle": ("OBSTACLE", 0.045),
    "construction": ("CONSTRUCTION", 0.035),
    "berm": ("BERM", 0.050),
}


def generate_label(word: str, cell: float, output: Path) -> None:
    width_cells = len(word) * 6 - 1
    x_offset = -0.5 * width_cells * cell
    y_offset = -3.5 * cell
    lines = [f"# Block-letter floor label: {word}", f"o label_{word.lower()}", "vn 0 0 1"]
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for letter_index, letter in enumerate(word):
        for row, pattern in enumerate(FONT[letter]):
            for column, filled in enumerate(pattern):
                if filled != "1":
                    continue
                x0 = x_offset + (letter_index * 6 + column) * cell
                x1 = x0 + 0.86 * cell
                y1 = y_offset + (7 - row) * cell
                y0 = y1 - 0.86 * cell
                first = len(vertices) + 1
                vertices.extend(((x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)))
                faces.extend(((first, first + 1, first + 2), (first, first + 2, first + 3)))

    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(f"f {a}//1 {b}//1 {c}//1" for a, b, c in faces)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="ascii")


if __name__ == "__main__":
    destination = Path(__file__).resolve().parent.parent / "assets" / "shared" / "zone_labels"
    for filename, (word, cell) in LABELS.items():
        path = destination / f"{filename}.obj"
        generate_label(word, cell, path)
        print(path)
