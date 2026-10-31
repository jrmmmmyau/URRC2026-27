#!/usr/bin/env python3

import collections
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
from pathlib import Path


# Map encoding for new maps
OCCUPIED = 0
FREE = 254
UNKNOWN = 255
MIN_GRAY = 1
MAX_GRAY = 253

MAP_MARKER = b"UR_MAP_V2"


def read_pgm(filename):
    """Read P2/P5 PGM files and convert old maps to the new encoding."""
    data = Path(filename).read_bytes()
    pos = 0
    comments = []

    def token():
        nonlocal pos

        while pos < len(data):
            if data[pos] in b" \t\r\n\f\v":
                pos += 1
            elif data[pos] == ord("#"):
                end = data.find(b"\n", pos)
                if end == -1:
                    end = len(data)
                comments.append(data[pos + 1:end].strip())
                pos = end
            else:
                break

        if pos >= len(data):
            raise ValueError("Unexpected end of PGM file.")

        start = pos

        while pos < len(data) and data[pos] not in b" \t\r\n\f\v#":
            pos += 1

        return data[start:pos].decode("ascii")

    magic = token()

    if magic not in ("P2", "P5"):
        raise ValueError("Only P2 and P5 PGM files are supported.")

    width = int(token())
    height = int(token())
    maxval = int(token())

    if width < 1 or height < 1:
        raise ValueError("Invalid map dimensions.")

    if not 1 <= maxval <= 255:
        raise ValueError("Only 8-bit PGM maps are supported.")

    if magic == "P2":
        pixels = [int(token()) for _ in range(width * height)]
    else:
        # Skip the whitespace separator after maxval.
        if pos >= len(data) or data[pos] not in b" \t\r\n\f\v":
            raise ValueError("Invalid P5 header.")

        if data[pos:pos + 2] == b"\r\n":
            pos += 2
        else:
            pos += 1

        pixels = list(data[pos:pos + width * height])

        if len(pixels) != width * height:
            raise ValueError("Incomplete PGM pixel data.")

    if any(p < 0 or p > maxval for p in pixels):
        raise ValueError("Invalid pixel value in PGM.")

    is_new_format = any(MAP_MARKER in c for c in comments)

    if not is_new_format:
        # Legacy maps:
        # black = occupied, white = free.
        # Convert to the new encoding.
        converted = []

        for pixel in pixels:
            if pixel == 0:
                converted.append(OCCUPIED)
            elif pixel == maxval:
                converted.append(FREE)
            else:
                value = maxval - pixel
                converted.append(max(MIN_GRAY, min(MAX_GRAY, value)))

        pixels = converted

    return width, height, pixels


def write_pgm(filename, width, height, pixels):
    """Save the new encoding directly into a P5 PGM file."""
    header = (
        "P5\n"
        "# UR_MAP_V2: 0=occupied, 1..253=gray, "
        "254=free, 255=unknown\n"
        f"{width} {height}\n"
        "255\n"
    ).encode("ascii")

    Path(filename).write_bytes(header + bytes(pixels))


class MapEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 2 PGM Map Editor")
        self.root.geometry("1100x800")

        self.width = 200
        self.height = 200
        self.pixels = [FREE] * (self.width * self.height)
        self.filename = None

        self.undo_stack = []
        self.stroke_changes = {}
        self.last_cell = None
        self.drawing = False

        self.brush_size = tk.IntVar(value=1)
        self.gray_value = tk.IntVar(value=128)
        self.tool = tk.StringVar(value="occupied")
        self.cell_size = 5

        self._build_ui()
        self._redraw()
        self._update_status()

        self.root.bind("<Control-z>", lambda _e: self.undo())
        self.root.bind("<Control-s>", lambda _e: self.save())
        self.root.bind("<Control-o>", lambda _e: self.open_map())

    def _build_ui(self):
        toolbar = ttk.Frame(self.root, padding=8)
        toolbar.pack(fill=tk.X)

        ttk.Button(toolbar, text="New", command=self.new_map).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Open", command=self.open_map).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="Save", command=self.save).pack(side=tk.LEFT)
        ttk.Button(toolbar, text="Save As", command=self.save_as).pack(side=tk.LEFT, padx=4)
        ttk.Button(toolbar, text="Undo", command=self.undo).pack(side=tk.LEFT, padx=8)

        tools = ttk.LabelFrame(self.root, text="Tools", padding=8)
        tools.pack(fill=tk.X, padx=8, pady=4)

        options = [
            ("Occupied", "occupied"),
            ("Free / Erase", "free"),
            ("Unknown", "unknown"),
            ("Grayscale", "gray"),
            ("Paint Bucket", "bucket"),
        ]

        for label, value in options:
            ttk.Radiobutton(
                tools,
                text=label,
                variable=self.tool,
                value=value
            ).pack(side=tk.LEFT, padx=4)

        ttk.Label(tools, text="Brush size:").pack(side=tk.LEFT, padx=(15, 4))

        ttk.Spinbox(
            tools,
            from_=1,
            to=50,
            width=4,
            textvariable=self.brush_size
        ).pack(side=tk.LEFT)

        ttk.Label(tools, text="Gray value:").pack(side=tk.LEFT, padx=(15, 4))

        ttk.Scale(
            tools,
            from_=MIN_GRAY,
            to=MAX_GRAY,
            variable=self.gray_value,
            orient=tk.HORIZONTAL,
            length=140
        ).pack(side=tk.LEFT)

        self.gray_label = ttk.Label(tools, text="128", width=4)
        self.gray_label.pack(side=tk.LEFT)

        self.gray_value.trace_add(
            "write",
            lambda *_: self.gray_label.configure(
                text=str(int(self.gray_value.get()))
            )
        )

        canvas_frame = ttk.Frame(self.root, padding=8)
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            canvas_frame,
            background="#cccccc",
            highlightthickness=1
        )

        hbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.HORIZONTAL,
            command=self.canvas.xview
        )

        vbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.VERTICAL,
            command=self.canvas.yview
        )

        self.canvas.configure(
            xscrollcommand=hbar.set,
            yscrollcommand=vbar.set
        )

        self.canvas.grid(row=0, column=0, sticky="nsew")
        vbar.grid(row=0, column=1, sticky="ns")
        hbar.grid(row=1, column=0, sticky="ew")

        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.status = tk.StringVar()
        ttk.Label(
            self.root,
            textvariable=self.status,
            padding=8
        ).pack(fill=tk.X)

        self.canvas.bind("<Button-1>", self._mouse_down)
        self.canvas.bind("<B1-Motion>", self._mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._mouse_up)

    def _color(self, value):
        if value == UNKNOWN:
            return "#bd91db"
        if value == FREE:
            return "#ffffff"
        if value == OCCUPIED:
            return "#111111"

        return f"#{value:02x}{value:02x}{value:02x}"

    def _redraw(self):
        self.canvas.delete("all")

        cell = self.cell_size
        self.canvas.configure(
            scrollregion=(0, 0, self.width * cell, self.height * cell)
        )

        # Render cells as a single image for responsive drawing.
        self.image = tk.PhotoImage(
            width=self.width,
            height=self.height
        )

        for y in range(self.height):
            row = "{" + " ".join(
                self._color(self.pixels[y * self.width + x])
                for x in range(self.width)
            ) + "}"
            self.image.put(row, to=(0, y))

        self.canvas.create_image(
            0, 0,
            image=self.image,
            anchor=tk.NW
        )

        self.canvas.scale("all", 0, 0, cell, cell)

        # Reapply the intended map display size.
        self.canvas.delete("all")

        self.image = tk.PhotoImage(
            width=self.width,
            height=self.height
        )

        for y in range(self.height):
            for x in range(self.width):
                value = self.pixels[y * self.width + x]
                color = self._color(value)
                self.image.put(color, (x, y))

        self.canvas.create_image(
            0, 0,
            image=self.image,
            anchor=tk.NW
        )

        self.canvas.configure(
            scrollregion=(0, 0, self.width * cell, self.height * cell)
        )

        self.canvas.scale("all", 0, 0, cell, cell)

        # Draw unknown cells with a visible question mark when zoomed in.
        if cell >= 12:
            for y in range(self.height):
                for x in range(self.width):
                    if self.pixels[y * self.width + x] == UNKNOWN:
                        self.canvas.create_text(
                            x * cell + cell / 2,
                            y * cell + cell / 2,
                            text="?",
                            fill="#3b174f"
                        )

    def _update_cell(self, index):
        x = index % self.width
        y = index // self.width
        cell = self.cell_size

        # A local redraw keeps edits quick.
        color = self._color(self.pixels[index])

        self.canvas.create_rectangle(
            x * cell,
            y * cell,
            (x + 1) * cell,
            (y + 1) * cell,
            fill=color,
            outline=color
        )

        if self.pixels[index] == UNKNOWN and cell >= 12:
            self.canvas.create_text(
                x * cell + cell / 2,
                y * cell + cell / 2,
                text="?",
                fill="#3b174f"
            )

    def _cell_at(self, event):
        cell = self.cell_size
        x = int(self.canvas.canvasx(event.x) // cell)
        y = int(self.canvas.canvasy(event.y) // cell)

        if 0 <= x < self.width and 0 <= y < self.height:
            return x, y

        return None

    def _selected_value(self):
        mode = self.tool.get()

        if mode == "free":
            return FREE
        if mode == "unknown":
            return UNKNOWN
        if mode == "gray":
            return max(MIN_GRAY, min(MAX_GRAY, int(self.gray_value.get())))

        return OCCUPIED

    def _remember(self, index):
        if index not in self.stroke_changes:
            self.stroke_changes[index] = self.pixels[index]

    def _paint(self, x, y):
        size = max(1, min(50, int(self.brush_size.get())))
        value = self._selected_value()

        low = -(size // 2)

        for dy in range(low, low + size):
            for dx in range(low, low + size):
                xx = x + dx
                yy = y + dy

                if not (0 <= xx < self.width and 0 <= yy < self.height):
                    continue

                index = yy * self.width + xx

                if self.pixels[index] != value:
                    self._remember(index)
                    self.pixels[index] = value
                    self._update_cell(index)

    def _bucket_fill(self, x, y):
        start = y * self.width + x
        target = self.pixels[start]
        replacement = self._selected_value()

        if target == replacement:
            return

        changes = {}
        queue = collections.deque([start])
        visited = {start}

        while queue:
            index = queue.popleft()

            if self.pixels[index] != target:
                continue

            changes[index] = target
            self.pixels[index] = replacement
            self._update_cell(index)

            cx = index % self.width
            cy = index // self.width

            for nx, ny in (
                (cx - 1, cy),
                (cx + 1, cy),
                (cx, cy - 1),
                (cx, cy + 1)
            ):
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    ni = ny * self.width + nx

                    if ni not in visited and self.pixels[ni] == target:
                        visited.add(ni)
                        queue.append(ni)

        if changes:
            self.undo_stack.append(changes)
            self._update_status()

    def _mouse_down(self, event):
        cell = self._cell_at(event)

        if cell is None:
            return

        if self.tool.get() == "bucket":
            self._bucket_fill(*cell)
            return

        self.drawing = True
        self.stroke_changes = {}
        self.last_cell = cell
        self._paint(*cell)

    def _mouse_drag(self, event):
        if not self.drawing:
            return

        cell = self._cell_at(event)

        if cell is None or cell == self.last_cell:
            return

        self.last_cell = cell
        self._paint(*cell)

    def _mouse_up(self, _event):
        if not self.drawing:
            return

        self.drawing = False

        if self.stroke_changes:
            self.undo_stack.append(self.stroke_changes.copy())

        self.stroke_changes.clear()
        self.last_cell = None
        self._update_status()

    def undo(self):
        if not self.undo_stack:
            self.root.bell()
            return

        changes = self.undo_stack.pop()

        for index, old_value in changes.items():
            self.pixels[index] = old_value
            self._update_cell(index)

        self._update_status()

    def new_map(self):
        width = simpledialog.askinteger(
            "New map", "Width in cells:",
            initialvalue=self.width, minvalue=1, maxvalue=2000
        )

        if width is None:
            return

        height = simpledialog.askinteger(
            "New map", "Height in cells:",
            initialvalue=self.height, minvalue=1, maxvalue=2000
        )

        if height is None:
            return

        self.width = width
        self.height = height
        self.pixels = [FREE] * (width * height)
        self.filename = None
        self.undo_stack.clear()
        self._redraw()
        self._update_status()

    def open_map(self):
        filename = filedialog.askopenfilename(
            filetypes=[("PGM images", "*.pgm"), ("All files", "*.*")]
        )

        if not filename:
            return

        try:
            self.width, self.height, self.pixels = read_pgm(filename)
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return

        self.filename = filename
        self.undo_stack.clear()
        self._redraw()
        self._update_status()

    def save(self):
        if self.filename is None:
            self.save_as()
        else:
            self._save_to(self.filename)

    def save_as(self):
        filename = filedialog.asksaveasfilename(
            defaultextension=".pgm",
            filetypes=[("PGM images", "*.pgm"), ("All files", "*.*")]
        )

        if filename:
            self._save_to(filename)

    def _save_to(self, filename):
        try:
            write_pgm(filename, self.width, self.height, self.pixels)
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc)) 
            return

        self.filename = filename
        self._update_status()

        messagebox.showinfo(
            "Saved",
            "Map saved in the new encoding:\n\n"
            "0 = occupied\n"
            "1–253 = grayscale occupancy\n"
            "254 = free\n"
            "255 = unknown"
        )

    def _update_status(self):
        unknown = self.pixels.count(UNKNOWN)
        free = self.pixels.count(FREE)
        occupied = self.pixels.count(OCCUPIED)

        name = Path(self.filename).name if self.filename else "Untitled"

        self.status.set(
            f"{name} | {self.width} × {self.height} | "
            f"Free: {free} | Occupied: {occupied} | "
            f"Unknown: {unknown} | Undo steps: {len(self.undo_stack)}"
        )


def main():
    root = tk.Tk()
    MapEditor(root)
    root.mainloop()


if __name__ == "__main__":
    main()