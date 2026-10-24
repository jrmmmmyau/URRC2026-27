import tkinter as tk
from tkinter import filedialog, messagebox
import math


class MapGenerator:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Occupancy Map Generator")
        self.root.configure(bg="black")

        # Default map settings
        self.map_width = 100
        self.map_height = 100
        self.cell_size = 8

        # Grayscale value currently being painted
        self.paint_value = 0

        # 0 = free, 255 = definitely occupied
        self.map_data = [
            [255 for _ in range(self.map_width)]
            for _ in range(self.map_height)
        ]

        self.drawing = False
        self.eraser = False

        self.build_controls()
        self.build_canvas()

        self.draw_map()

    # ---------------------------------------------------------
    # GUI
    # ---------------------------------------------------------

    def build_controls(self):
        controls = tk.Frame(self.root, bg="black")
        controls.pack(fill=tk.X, padx=5, pady=5)

        # Width
        tk.Label(controls, text="Width:").grid(
            row=0, column=0, padx=3
        )

        self.width_entry = tk.Entry(controls, width=6)
        self.width_entry.insert(0, str(self.map_width))
        self.width_entry.grid(row=0, column=1, padx=3)

        # Height
        tk.Label(controls, text="Height:").grid(
            row=0, column=2, padx=3
        )

        self.height_entry = tk.Entry(controls, width=6)
        self.height_entry.insert(0, str(self.map_height))
        self.height_entry.grid(row=0, column=3, padx=3)

        tk.Button(
            controls,
            text="New Map",
            command=self.new_map
        ).grid(row=0, column=4, padx=5)

        # Paint value
        tk.Label(
            controls,
            text="Value (0-255):"
        ).grid(row=0, column=5, padx=3)

        self.value_entry = tk.Entry(
            controls,
            width=6
        )
        self.value_entry.insert(0, "0")
        self.value_entry.grid(row=0, column=6, padx=3)

        tk.Button(
            controls,
            text="Set Value",
            command=self.set_paint_value
        ).grid(row=0, column=7, padx=5)

        # Eraser
        self.eraser_button = tk.Button(
            controls,
            text="Eraser: OFF",
            command=self.toggle_eraser
        )
        self.eraser_button.grid(row=0, column=8, padx=5)

        # Save / load
        tk.Button(
            controls,
            text="Save PGM",
            command=self.save_pgm
        ).grid(row=0, column=9, padx=5)

        tk.Button(
            controls,
            text="Open PGM",
            command=self.open_pgm
        ).grid(row=0, column=10, padx=5)

        # Status
        self.status_label = tk.Label(
            self.root,
            text="Value: 255"
        )
        self.status_label.pack(anchor="w", padx=8)

    def build_canvas(self):
        self.canvas = tk.Canvas(
            self.root,
            background="white"
        )

        self.canvas.pack(
            padx=5,
            pady=5,
            fill=tk.BOTH,
            expand=True
        )

        self.canvas.bind(
            "<ButtonPress-1>",
            self.start_drawing
        )

        self.canvas.bind(
            "<B1-Motion>",
            self.draw
        )

        self.canvas.bind(
            "<ButtonRelease-1>",
            self.stop_drawing
        )

    # ---------------------------------------------------------
    # Map creation
    # ---------------------------------------------------------

    def new_map(self):
        try:
            width = int(self.width_entry.get())
            height = int(self.height_entry.get())

            if width <= 0 or height <= 0:
                raise ValueError

            if width > 500 or height > 500:
                messagebox.showerror(
                    "Invalid size",
                    "Maximum map size is 500 x 500."
                )
                return

        except ValueError:
            messagebox.showerror(
                "Invalid size",
                "Width and height must be positive integers."
            )
            return

        self.map_width = width
        self.map_height = height

        self.map_data = [
            [255 for _ in range(width)]
            for _ in range(height)
        ]

        self.calculate_cell_size()
        self.draw_map()

    def calculate_cell_size(self):
        # Keep the map reasonably sized on screen.
        max_width = 1000
        max_height = 700

        width_size = max_width // self.map_width
        height_size = max_height // self.map_height

        self.cell_size = max(
            1,
            min(width_size, height_size)
        )

    # ---------------------------------------------------------
    # Painting
    # ---------------------------------------------------------

    def set_paint_value(self):
        try:
            value = int(self.value_entry.get())

            if value < 0 or value > 255:
                raise ValueError

            self.paint_value = value
            self.eraser = False

            self.eraser_button.config(
                text="Eraser: OFF"
            )

            self.status_label.config(
                text=f"Value: {self.paint_value}"
            )

        except ValueError:
            messagebox.showerror(
                "Invalid value",
                "Value must be an integer from 0 to 255."
            )

    def toggle_eraser(self):
        self.eraser = not self.eraser

        if self.eraser:
            self.eraser_button.config(
                text="Eraser: ON"
            )
            self.status_label.config(
                text="Eraser: Value 255"
            )
        else:
            self.eraser_button.config(
                text="Eraser: OFF"
            )
            self.status_label.config(
                text=f"Value: {self.paint_value}"
            )

    def start_drawing(self, event):
        self.drawing = True
        self.paint_at_mouse(event.x, event.y)

    def draw(self, event):
        if self.drawing:
            self.paint_at_mouse(event.x, event.y)

    def stop_drawing(self, event):
        self.drawing = False

    def paint_at_mouse(self, mouse_x, mouse_y):
        x = mouse_x // self.cell_size
        y = mouse_y // self.cell_size

        if (
            x < 0 or
            x >= self.map_width or
            y < 0 or
            y >= self.map_height
        ):
            return

        if self.eraser:
            value = 0
        else:
            value = self.paint_value

        self.map_data[y][x] = value

        self.draw_cell(x, y)

    # ---------------------------------------------------------
    # Drawing
    # ---------------------------------------------------------

    def grayscale(self, value):
        return f"#{value:02x}{value:02x}{value:02x}"

    def draw_cell(self, x, y):
        x1 = x * self.cell_size
        y1 = y * self.cell_size

        x2 = x1 + self.cell_size
        y2 = y1 + self.cell_size

        value = self.map_data[y][x]

        self.canvas.create_rectangle(
            x1,
            y1,
            x2,
            y2,
            fill=self.grayscale(value),
            outline=""
        )

    def draw_map(self):
        self.canvas.delete("all")

        self.canvas.config(
            width=self.map_width * self.cell_size,
            height=self.map_height * self.cell_size,
            scrollregion=(
                0,
                0,
                self.map_width * self.cell_size,
                self.map_height * self.cell_size
            )
        )

        for y in range(self.map_height):
            for x in range(self.map_width):
                self.draw_cell(x, y)

    # ---------------------------------------------------------
    # PGM saving
    # ---------------------------------------------------------

    def save_pgm(self):
        filename = filedialog.asksaveasfilename(
            title="Save PGM map",
            defaultextension=".pgm",
            filetypes=[
                ("PGM files", "*.pgm"),
                ("All files", "*.*")
            ]
        )

        if not filename:
            return

        try:
            with open(filename, "wb") as file:
                file.write(b"P5\n")
                file.write(
                    f"{self.map_width} {self.map_height}\n"
                    .encode()
                )
                file.write(b"255\n")

                for y in range(self.map_height):
                    for x in range(self.map_width):
                        value = self.map_data[y][x]
                        file.write(bytes([value]))

            messagebox.showinfo(
                "Map saved",
                f"Saved:\n{filename}"
            )

        except Exception as e:
            messagebox.showerror(
                "Save error",
                str(e)
            )

    # ---------------------------------------------------------
    # PGM loading
    # ---------------------------------------------------------

    def next_token(self, file):
        token = b""

        while True:
            char = file.read(1)

            if not char:
                raise RuntimeError(
                    "Unexpected end of PGM file."
                )

            if char.isspace():
                if token:
                    return token.decode()
                continue

            if char == b"#":
                file.readline()
                continue

            token += char

    def open_pgm(self):
        filename = filedialog.askopenfilename(
            title="Open PGM map",
            filetypes=[
                ("PGM files", "*.pgm"),
                ("All files", "*.*")
            ]
        )

        if not filename:
            return

        try:
            with open(filename, "rb") as file:

                magic = self.next_token(file)

                if magic != "P5":
                    raise RuntimeError(
                        "Only binary P5 PGM files are supported."
                    )

                width = int(
                    self.next_token(file)
                )

                height = int(
                    self.next_token(file)
                )

                max_value = int(
                    self.next_token(file)
                )

                if max_value != 255:
                    raise RuntimeError(
                        "PGM must use max value 255."
                    )

                pixel_count = width * height

                raw_data = file.read(pixel_count)

                if len(raw_data) != pixel_count:
                    raise RuntimeError(
                        "PGM file contains insufficient data."
                    )

                self.map_width = width
                self.map_height = height

                self.map_data = [
                    [
                        raw_data[
                            y * width + x
                        ]
                        for x in range(width)
                    ]
                    for y in range(height)
                ]

                self.width_entry.delete(0, tk.END)
                self.width_entry.insert(
                    0,
                    str(width)
                )

                self.height_entry.delete(0, tk.END)
                self.height_entry.insert(
                    0,
                    str(height)
                )

                self.calculate_cell_size()
                self.draw_map()

                messagebox.showinfo(
                    "Map loaded",
                    f"Loaded:\n{filename}"
                )

        except Exception as e:
            messagebox.showerror(
                "Load error",
                str(e)
            )


def main():
    root = tk.Tk()

    app = MapGenerator(root)

    root.mainloop()


if __name__ == "__main__":
    main()