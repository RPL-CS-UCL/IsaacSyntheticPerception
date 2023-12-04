
import tkinter as tk
from tkinter import filedialog, colorchooser, simpledialog, ttk
from PIL import Image, ImageTk, ImageDraw
import hashlib
import numpy as np

class HeightmapEditor:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, bg='white')
        self.h_scroll = tk.Scrollbar(root, orient='horizontal', command=self.canvas.xview)
        self.h_scroll.pack(side='bottom', fill='x')
        self.v_scroll = tk.Scrollbar(root, orient='vertical', command=self.canvas.yview)
        self.v_scroll.pack(side='right', fill='y')
        self.canvas.configure(xscrollcommand=self.h_scroll.set, yscrollcommand=self.v_scroll.set)
        self.canvas.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        # Setup buttons
        load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
        load_btn.pack(side=tk.TOP)

        undo_btn = tk.Button(root, text="Undo Last Region", command=self.undo_last_region)
        undo_btn.pack(side=tk.TOP)

        clear_btn = tk.Button(root, text="Clear All Regions", command=self.clear_all_regions)
        clear_btn.pack(side=tk.TOP)

        save_btn = tk.Button(root, text="Save Edited Heightmap", command=self.save_image)
        save_btn.pack(side=tk.TOP)

        zoom_in_btn = tk.Button(root, text="Zoom In", command=lambda: self.zoom(1.2))
        zoom_in_btn.pack(side=tk.TOP)

        zoom_out_btn = tk.Button(root, text="Zoom Out", command=lambda: self.zoom(0.8))
        zoom_out_btn.pack(side=tk.TOP)

        self.id_entry = ttk.Entry(root, width=10)

        self.id_entry.insert(0,1)
        self.id_entry.pack(side=tk.TOP)
        self.parent_id_entry = ttk.Entry(root, width=10)
        self.parent_id_entry.insert(0,-1)
        self.parent_id_entry.pack(side=tk.TOP)

        # Initialize a dictionary to store parent-child relationships
        self.parent_child_dict = {}

        # Setup mouse events for drawing and panning
        self.canvas.bind("<ButtonPress-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)
        self.canvas.bind("<ButtonPress-2>", self.start_pan)
        self.canvas.bind("<B2-Motion>", self.pan)
        self.canvas.bind("<ButtonRelease-2>", self.end_pan)

        self.original_image = None
        self.image = None
        self.tk_image = None
        self.scale = 1.0
        self.region_id = 1
        self.regions = {}  # Dictionary to store regions

        self.is_panning = False
        self.pan_start_x = 0
        self.pan_start_y = 0

    def load_heightmap(self):
        file_path = filedialog.askopenfilename()
        if file_path:
            self.original_image = Image.open(file_path).convert("I")
            self.scale = 1.0
            self.image = self.original_image.copy()
            self.tk_image = ImageTk.PhotoImage(self.image)
            self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)
            self.canvas.config(scrollregion=self.canvas.bbox('all'))
            self.regions.clear()
            self.region_id = 1

    def undo_last_region(self):
        if self.regions:
            last_region_id = max(self.regions.keys())
            del self.regions[last_region_id]
            self.redraw_image()

    def clear_all_regions(self):
        self.regions.clear()
        self.redraw_image()

    def save_image(self):

        arr = self.create_region_array()
        np.save("/home/jon/Desktop/hm_mask.npy", np.array(arr))
        np.save("/home/jon/Desktop/hm_sv.npy", np.array(self.original_image.convert('I')))
        print(np.array(self.image.convert('L')).shape)
        print(np.array(arr).shape)

        self.update_canvas_with_new_image()
        return
        arr = self.create_region_array()
        print(arr)
        return
        file_path = filedialog.asksaveasfilename(defaultextension=".png")
        if file_path:
            self.image.save(file_path)

    def zoom(self, scale_factor):
        if self.original_image:
            self.scale *= scale_factor
            new_size = (int(self.original_image.width * self.scale), int(self.original_image.height * self.scale))
            self.image = self.original_image.resize(new_size, Image.ANTIALIAS)
            self.tk_image = ImageTk.PhotoImage(self.image)
            self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)
            self.canvas.config(scrollregion=self.canvas.bbox('all'))
            self.redraw_image()

    def start_draw(self, event):
        if self.is_panning:
            return
        x, y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        self.start_x, self.start_y = self.canvas_to_image(x, y)
        self.current_region = [(self.start_x, self.start_y)]
        self.region_id = int(self.id_entry.get() or self.region_id)
        self.color = self.generate_color(self.region_id)

    def draw(self, event):
        if self.is_panning:
            return
        x, y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        img_x, img_y = self.canvas_to_image(x, y)
        self.canvas.create_line(self.start_x * self.scale, self.start_y * self.scale, img_x * self.scale, img_y * self.scale, fill=self.rgb_to_hex(self.color))
        self.start_x, self.start_y = img_x, img_y
        self.current_region.append((self.start_x, self.start_y))

    def end_draw(self, event):
        if self.is_panning or not self.current_region:
            return
        self.current_region.append((self.current_region[0][0], self.current_region[0][1]))  # Close the loop
        self.regions[self.region_id] = self.current_region
        self.fill_region(self.current_region, self.color)
        self.region_id += 1
        parent_id = int(self.parent_id_entry.get() or -1)
        if parent_id == -1:
            self.parent_child_dict[self.region_id] = []
        else:
            if parent_id not in self.parent_child_dict:
                self.parent_child_dict[parent_id] = []
            self.parent_child_dict[parent_id].append(self.region_id)

    def start_pan(self, event):
        self.is_panning = True
        self.canvas.scan_mark(event.x, event.y)

    def pan(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def end_pan(self, event):
        self.is_panning = False

    def fill_region(self, region, color):
        draw = ImageDraw.Draw(self.image)
        scaled_region = [(x * self.scale, y * self.scale) for x, y in region]
        draw.polygon(scaled_region, fill=self.rgb_to_hex(color))
        self.tk_image = ImageTk.PhotoImage(self.image)
        self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)

    def redraw_image(self):
        self.image = self.original_image.resize((int(self.original_image.width * self.scale), int(self.original_image.height * self.scale)), Image.ANTIALIAS)
        for region_id, region in self.regions.items():
            self.fill_region(region, self.generate_color(region_id))
        self.tk_image = ImageTk.PhotoImage(self.image)
        self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)

    def canvas_to_image(self, canvas_x, canvas_y):
        return int(canvas_x / self.scale), int(canvas_y / self.scale)

    def generate_color(self, id):
        hash_object = hashlib.md5(str(id).encode())
        hex_dig = hash_object.hexdigest()
        red = int(hex_dig[0:2], 16)
        green = int(hex_dig[2:4], 16)
        blue = int(hex_dig[4:6], 16)
        return (red, green, blue)
    def create_region_array(self):
        # Initialize a 2D array with zeros
        width, height = self.original_image.size
        region_array = [[0 for _ in range(width)] for _ in range(height)]

        # Create a mask for each region and assign the region ID to the pixels
        for region_id, region in self.regions.items():
            mask = Image.new('L', (width, height), 0)
            ImageDraw.Draw(mask).polygon(region, outline=1, fill=1)
            mask_array = list(mask.getdata())
            for y in range(height):
                for x in range(width):
                    if mask_array[y * width + x]:
                        region_array[y][x] = region_id

        return region_array
    def draw_image_from_array(self, region_array):
        height = len(region_array)
        width = len(region_array[0]) if height > 0 else 0

        # Create a new image
        new_image = Image.new("RGB", (width, height), "white")
        pixels = new_image.load()

        # Iterate over the region array and set the pixel colors
        for y in range(height):
            for x in range(width):
                region_id = region_array[y][x]
                if region_id != 0:
                    color = self.generate_color(region_id)
                    pixels[x, y] = (int(color[0]), int(color[1]), int(color[2]))
        
        return new_image
    def update_canvas_with_new_image(self):
        # Clear the canvas
        self.canvas.delete("all")

        # Create a 2D array from the current regions
        region_array = self.create_region_array()

        # Create a new image from the 2D array
        new_image = self.draw_image_from_array(region_array)

        # Convert the PIL image to a format Tkinter canvas can use
        self.tk_image = ImageTk.PhotoImage(new_image)

        # Display the new image on the canvas
        self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)
        self.canvas.config(scrollregion=self.canvas.bbox('all'))

    def rgb_to_hex(self, rgb):
        return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Heightmap Editor")
    app = HeightmapEditor(root)
    root.mainloop()
# import tkinter as tk
# from tkinter import filedialog, messagebox, ttk
# from PIL import Image, ImageTk, ImageDraw
# import hashlib
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup horizontal and vertical scrollbars for the canvas
#         self.h_scroll = tk.Scrollbar(root, orient='horizontal', command=self.canvas.xview)
#         self.h_scroll.pack(side='bottom', fill='x')
#         self.v_scroll = tk.Scrollbar(root, orient='vertical', command=self.canvas.yview)
#         self.v_scroll.pack(side='right', fill='y')
#         self.canvas.configure(xscrollcommand=self.h_scroll.set, yscrollcommand=self.v_scroll.set)
#
#         # Setup buttons
#         load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
#         load_btn.pack(side=tk.LEFT)
#
#         undo_btn = tk.Button(root, text="Undo Last Region", command=self.undo_last_region)
#         undo_btn.pack(side=tk.LEFT)
#
#         clear_btn = tk.Button(root, text="Clear All Regions", command=self.clear_all_regions)
#         clear_btn.pack(side=tk.LEFT)
#
#         save_btn = tk.Button(root, text="Save Edited Heightmap", command=self.save_image)
#         save_btn.pack(side=tk.LEFT)
#
#         zoom_in_btn = tk.Button(root, text="Zoom In", command=lambda: self.zoom(1.2))
#         zoom_in_btn.pack(side=tk.LEFT)
#
#         zoom_out_btn = tk.Button(root, text="Zoom Out", command=lambda: self.zoom(0.8))
#         zoom_out_btn.pack(side=tk.LEFT)
#
#         self.id_entry = ttk.Entry(root, width=10)
#         self.id_entry.pack(side=tk.LEFT)
#
#         # Setup mouse events for drawing and panning
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#         self.canvas.bind("<ButtonPress-2>", self.start_pan)
#         self.canvas.bind("<B2-Motion>", self.pan)
#         self.canvas.bind("<ButtonRelease-2>", self.end_pan)
#
#         self.original_image = None
#         self.image = None
#         self.tk_image = None
#         self.scale = 1.0
#         self.region_id = 1
#         self.regions = {}  # Dictionary to store regions
#
#         self.is_panning = False
#         self.pan_start_x = 0
#         self.pan_start_y = 0
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.original_image = Image.open(file_path)
#             self.scale = 1.0
#             self.image = self.original_image.copy()
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor='nw', image=self.tk_image)
#             self.canvas.config(scrollregion=self.canvas.bbox('all'))
#             self.regions.clear()
#             self.region_id = 1
#
#     # ... [Other methods: undo_last_region, clear_all_regions, save_image, zoom, start_draw, draw, end_draw, etc.] ...
#
#     def start_pan(self, event):
#         self.canvas.scan_mark(event.x, event.y)
#         self.is_panning = True
#
#     def pan(self, event):
#         self.canvas.scan_dragto(event.x, event.y, gain=1)
#
#     def end_pan(self, event):
#         self.is_panning = False
#
#     def undo_last_region(self):
#         if self.regions:
#             last_region_id = max(self.regions.keys())
#             del self.regions[last_region_id]
#             self.redraw_image()
#
#     def clear_all_regions(self):
#         self.regions.clear()
#         self.redraw_image()
#
#     def save_image(self):
#         file_path = filedialog.asksaveasfilename(defaultextension=".png")
#         if file_path:
#             self.image.save(file_path)
#
#     def zoom(self, scale_factor):
#         if self.original_image:
#             self.scale *= scale_factor
#             self.image = self.original_image.resize((int(self.original_image.width * self.scale), int(self.original_image.height * self.scale)), Image.ANTIALIAS)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
#             self.redraw_image()
#
#     def start_draw(self, event):
#         self.start_x, self.start_y = self.canvas_to_image(event.x, event.y)
#         self.current_region = [(self.start_x, self.start_y)]
#         self.region_id = int(self.id_entry.get() or self.region_id)
#         self.color = self.generate_color(self.region_id)
#
#     def draw(self, event):
#         img_x, img_y = self.canvas_to_image(event.x, event.y)
#         self.canvas.create_line(self.start_x * self.scale, self.start_y * self.scale, img_x * self.scale, img_y * self.scale, fill=self.rgb_to_hex(self.color))
#         self.start_x, self.start_y = img_x, img_y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1]))  # Close the loop
#         self.regions[self.region_id] = self.current_region
#         self.fill_region(self.current_region, self.color)
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         draw = ImageDraw.Draw(self.image)
#         scaled_region = [(x * self.scale, y * self.scale) for x, y in region]
#         draw.polygon(scaled_region, fill=self.rgb_to_hex(color))
#         self.tk_image = ImageTk.PhotoImage(self.image)
#         self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def redraw_image(self):
#         self.image = self.original_image.resize((int(self.original_image.width * self.scale), int(self.original_image.height * self.scale)), Image.ANTIALIAS)
#         for region_id, region in self.regions.items():
#             self.fill_region(region, self.generate_color(region_id))
#         self.tk_image = ImageTk.PhotoImage(self.image)
#         self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def canvas_to_image(self, canvas_x, canvas_y):
#         return int(canvas_x / self.scale), int(canvas_y / self.scale)
#
#     def generate_color(self, id):
#         hash_object = hashlib.md5(str(id).encode())
#         hex_dig = hash_object.hexdigest()
#         red = int(hex_dig[0:2], 16)
#         green = int(hex_dig[2:4], 16)
#         blue = int(hex_dig[4:6], 16)
#         return (red, green, blue)
#
#     def rgb_to_hex(self, rgb):
#         return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()
# import tkinter as tk
# from tkinter import filedialog, messagebox, ttk
# from PIL import Image, ImageTk, ImageDraw
# import hashlib
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup buttons
#         load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
#         load_btn.pack(side=tk.LEFT)
#
#         undo_btn = tk.Button(root, text="Undo Last Region", command=self.undo_last_region)
#         undo_btn.pack(side=tk.LEFT)
#
#         clear_btn = tk.Button(root, text="Clear All Regions", command=self.clear_all_regions)
#         clear_btn.pack(side=tk.LEFT)
#
#         save_btn = tk.Button(root, text="Save Edited Heightmap", command=self.save_image)
#         save_btn.pack(side=tk.LEFT)
#
#         zoom_in_btn = tk.Button(root, text="Zoom In", command=lambda: self.zoom(1.2))
#         zoom_in_btn.pack(side=tk.LEFT)
#
#         zoom_out_btn = tk.Button(root, text="Zoom Out", command=lambda: self.zoom(0.8))
#         zoom_out_btn.pack(side=tk.LEFT)
#
#         self.id_entry = ttk.Entry(root, width=10)
#         self.id_entry.pack(side=tk.LEFT)
#
#         # Setup mouse events for drawing
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#
#         self.image = None
#         self.original_image = None
#         self.tk_image = None
#         self.scale = 1.0
#         self.region_id = 1
#         self.regions = {}  # Dictionary to store regions
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.original_image = Image.open(file_path)
#             self.image = self.original_image.copy()
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def undo_last_region(self):
#         if self.regions:
#             last_region_id = max(self.regions.keys())
#             del self.regions[last_region_id]
#             self.reload_image()
#             self.region_id -= 1
#
#     def clear_all_regions(self):
#         self.regions.clear()
#         self.reload_image()
#         self.region_id = 1
#
#     def save_image(self):
#         file_path = filedialog.asksaveasfilename(defaultextension=".png")
#         if file_path:
#             self.image.save(file_path)
#
#     def zoom(self, scale_factor):
#         if self.original_image:
#             self.scale *= scale_factor
#             new_size = (int(self.original_image.width * self.scale), int(self.original_image.height * self.scale))
#             self.image = self.original_image.resize(new_size, Image.ANTIALIAS)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
#             self.reload_image()  # Redraw regions on the scaled image
#
#     def start_draw(self, event):
#         self.start_x, self.start_y = self.canvas_to_image(event.x, event.y)
#         self.current_region = [(self.start_x, self.start_y)]
#         self.region_id = int(self.id_entry.get() or self.region_id)
#         self.color = self.generate_color(self.region_id)
#
#     def draw(self, event):
#         img_x, img_y = self.canvas_to_image(event.x, event.y)
#         self.canvas.create_line(self.start_x * self.scale, self.start_y * self.scale, img_x * self.scale, img_y * self.scale, fill=self.rgb_to_hex(self.color))
#         self.start_x, self.start_y = img_x, img_y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1])) # Close the loop
#         self.fill_region(self.current_region, self.color)
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         if self.image:
#             draw = ImageDraw.Draw(self.image)
#             scaled_region = [(x * self.scale, y * self.scale) for x, y in region]
#             draw.polygon(scaled_region, fill=self.rgb_to_hex(color))
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.regions[self.region_id] = scaled_region
#
#     def reload_image(self):
#         self.image = self.original_image.resize((int(self.original_image.width * self.scale), int(self.original_image.height * self.scale)), Image.ANTIALIAS)
#         for region_id, region in self.regions.items():
#             color = self.generate_color(region_id)
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=self.rgb_to_hex(color))
#         self.tk_image = ImageTk.PhotoImage(self.image)
#         self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def canvas_to_image(self, canvas_x, canvas_y):
#         # Convert canvas coordinates to image coordinates
#         return int(canvas_x / self.scale), int(canvas_y / self.scale)
#
#     def generate_color(self, id):
#         hash_object = hashlib.md5(str(id).encode())
#         hex_dig = hash_object.hexdigest()
#         red = int(hex_dig[0:2], 16)
#         green = int(hex_dig[2:4], 16)
#         blue = int(hex_dig[4:6], 16)
#         return (red, green, blue)
#
#     def rgb_to_hex(self, rgb):
#         return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()
# import tkinter as tk
# from tkinter import filedialog
# from PIL import Image, ImageTk, ImageDraw
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup mouse events for drawing
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#
#         # Load button
#         load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
#         load_btn.pack()
#
#         self.image = None
#         self.tk_image = None
#         self.region_id = 1
#         self.regions = {}  # Dictionary to store regions
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.image = Image.open(file_path)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def start_draw(self, event):
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region = [(self.start_x, self.start_y)]
#
#     def draw(self, event):
#         self.canvas.create_line(self.start_x, self.start_y, event.x, event.y, fill='blue')
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1])) # Close the loop
#         self.fill_region(self.current_region, (255, 0, 0)) # Fill color red
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         if self.image:
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=color)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.regions[self.region_id] = region
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()

# import tkinter as tk
# from tkinter import filedialog, messagebox
# from PIL import Image, ImageTk, ImageDraw
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup mouse events for drawing
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#
#         # Load button
#         load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
#         load_btn.pack(side=tk.LEFT)
#
#         # Undo button
#         undo_btn = tk.Button(root, text="Undo Last Region", command=self.undo_last_region)
#         undo_btn.pack(side=tk.LEFT)
#
#         # Clear button
#         clear_btn = tk.Button(root, text="Clear All Regions", command=self.clear_all_regions)
#         clear_btn.pack(side=tk.LEFT)
#
#         # Save button
#         save_btn = tk.Button(root, text="Save Edited Heightmap", command=self.save_image)
#         save_btn.pack(side=tk.LEFT)
#
#         self.image = None
#         self.tk_image = None
#         self.region_id = 1
#         self.regions = {}  # Dictionary to store regions
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.image = Image.open(file_path)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def start_draw(self, event):
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region = [(self.start_x, self.start_y)]
#
#     def draw(self, event):
#         self.canvas.create_line(self.start_x, self.start_y, event.x, event.y, fill='blue')
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1])) # Close the loop
#         self.fill_region(self.current_region, (255, 0, 0)) # Fill color red
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         if self.image:
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=color)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.regions[self.region_id] = region
#
#     def undo_last_region(self):
#         if self.regions:
#             last_region_id = max(self.regions.keys())
#             del self.regions[last_region_id]
#             self.reload_image()
#             self.region_id -= 1
#
#     def clear_all_regions(self):
#         self.regions.clear()
#         self.reload_image()
#         self.region_id = 1
#
#     def reload_image(self):
#         self.image = self.image.copy()  # Reload original image
#         for region_id, region in self.regions.items():
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=(255, 0, 0))
#         self.tk_image = ImageTk.PhotoImage(self.image)
#         self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def save_image(self):
#         file_path = filedialog.asksaveasfilename(defaultextension=".png")
#         if file_path:
#             self.image.save(file_path)
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()
#
# import tkinter as tk
# from tkinter import filedialog, colorchooser, simpledialog
# from tkinter import ttk
# from PIL import Image, ImageTk, ImageDraw
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup mouse events for drawing
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#
#         # Zoom buttons
#         zoom_in_btn = tk.Button(root, text="Zoom In", command=lambda: self.zoom(1.2))
#         zoom_in_btn.pack(side=tk.LEFT)
#
#         zoom_out_btn = tk.Button(root, text="Zoom Out", command=lambda: self.zoom(0.8))
#         zoom_out_btn.pack(side=tk.LEFT)
#
#         # Color and ID selection
#         self.color_btn = tk.Button(root, text="Select Color", command=self.select_color)
#         self.color_btn.pack(side=tk.LEFT)
#
#         self.id_entry = ttk.Entry(root, width=10)
#         self.id_entry.pack(side=tk.LEFT)
#
#         self.color = (255, 0, 0) # Default color
#         self.region_id = 1 # Default ID
#
#         self.image = None
#         self.tk_image = None
#         self.scale = 1.0
#         self.regions = {}  # Dictionary to store regions
#
#     def zoom(self, scale_factor):
#         if self.image:
#             self.scale *= scale_factor
#             new_size = (int(self.image.width * self.scale), int(self.image.height * self.scale))
#             resized_image = self.image.resize(new_size, Image.ANTIALIAS)
#             self.tk_image = ImageTk.PhotoImage(resized_image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
#
#     def select_color(self):
#         color_code = colorchooser.askcolor(title="Choose color")
#         if color_code:
#             self.color = color_code[0]
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.image = Image.open(file_path)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def start_draw(self, event):
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region = [(self.start_x, self.start_y)]
#         self.region_id = int(self.id_entry.get() or self.region_id)
#
#     def draw(self, event):
#         self.canvas.create_line(self.start_x, self.start_y, event.x, event.y, fill=self.rgb_to_hex(self.color))
#         self.start_x, self.start_y = event.x, event.y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1])) # Close the loop
#         self.fill_region(self.current_region, self.color)
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         if self.image:
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=self.rgb_to_hex(color))
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.regions[self.region_id] = region
#
#     def rgb_to_hex(self, rgb):
#         return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()
#
#
# import tkinter as tk
# from tkinter import filedialog, colorchooser, messagebox, ttk
# from PIL import Image, ImageTk, ImageDraw
#
# class HeightmapEditor:
#     def __init__(self, root):
#         self.root = root
#         self.canvas = tk.Canvas(root, bg='white')
#         self.canvas.pack(fill=tk.BOTH, expand=True)
#
#         # Setup buttons
#         load_btn = tk.Button(root, text="Load Heightmap", command=self.load_heightmap)
#         load_btn.pack(side=tk.LEFT)
#
#         undo_btn = tk.Button(root, text="Undo Last Region", command=self.undo_last_region)
#         undo_btn.pack(side=tk.LEFT)
#
#         clear_btn = tk.Button(root, text="Clear All Regions", command=self.clear_all_regions)
#         clear_btn.pack(side=tk.LEFT)
#
#         save_btn = tk.Button(root, text="Save Edited Heightmap", command=self.save_image)
#         save_btn.pack(side=tk.LEFT)
#
#         zoom_in_btn = tk.Button(root, text="Zoom In", command=lambda: self.zoom(1.2))
#         zoom_in_btn.pack(side=tk.LEFT)
#
#         zoom_out_btn = tk.Button(root, text="Zoom Out", command=lambda: self.zoom(0.8))
#         zoom_out_btn.pack(side=tk.LEFT)
#
#         self.color_btn = tk.Button(root, text="Select Color", command=self.select_color)
#         self.color_btn.pack(side=tk.LEFT)
#
#         self.id_entry = ttk.Entry(root, width=10)
#         self.id_entry.pack(side=tk.LEFT)
#
#         # Setup mouse events for drawing
#         self.canvas.bind("<ButtonPress-1>", self.start_draw)
#         self.canvas.bind("<B1-Motion>", self.draw)
#         self.canvas.bind("<ButtonRelease-1>", self.end_draw)
#
#         self.image = None
#         self.tk_image = None
#         self.scale = 1.0
#         self.region_id = 1
#         self.color = (255, 0, 0)  # Default color
#         self.regions = {}  # Dictionary to store regions
#
#     def load_heightmap(self):
#         file_path = filedialog.askopenfilename()
#         if file_path:
#             self.image = Image.open(file_path)
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#     def start_draw(self, event):
#         canvas_x, canvas_y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
#         self.start_x, self.start_y = self.canvas_to_image(canvas_x, canvas_y)
#         self.current_region = [(self.start_x, self.start_y)]
#         self.region_id = int(self.id_entry.get() or self.region_id)
#         self.color = self.generate_color(self.region_id)
#
#     def draw(self, event):
#         canvas_x, canvas_y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
#         img_x, img_y = self.canvas_to_image(canvas_x, canvas_y)
#         self.canvas.create_line(self.start_x, self.start_y, img_x, img_y, fill=self.rgb_to_hex(self.color))
#         self.start_x, self.start_y = img_x, img_y
#         self.current_region.append((self.start_x, self.start_y))
#
#     def canvas_to_image(self, canvas_x, canvas_y):
#         # Convert canvas coordinates to image coordinates
#         return int(canvas_x / self.scale), int(canvas_y / self.scale)
#
#     def undo_last_region(self):
#         if self.regions:
#             last_region_id = max(self.regions.keys())
#             del self.regions[last_region_id]
#             self.reload_image()
#             self.region_id -= 1
#
#     def clear_all_regions(self):
#         self.regions.clear()
#         self.reload_image()
#         self.region_id = 1
#
#     def save_image(self):
#         file_path = filedialog.asksaveasfilename(defaultextension=".png")
#         if file_path:
#             self.image.save(file_path)
#
#     def zoom(self, scale_factor):
#         if self.image:
#             self.scale *= scale_factor
#             new_size = (int(self.image.width * self.scale), int(self.image.height * self.scale))
#             resized_image = self.image.resize(new_size, Image.ANTIALIAS)
#             self.tk_image = ImageTk.PhotoImage(resized_image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))
#
#     def select_color(self):
#         color_code = colorchooser.askcolor(title="Choose color")
#         if color_code:
#             self.color = color_code[0]
#
#
#     def end_draw(self, event):
#         self.current_region.append((self.current_region[0][0], self.current_region[0][1])) # Close the loop
#         self.fill_region(self.current_region, self.color)
#         self.region_id += 1
#
#     def fill_region(self, region, color):
#         if self.image:
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=self.rgb_to_hex(color))
#             self.tk_image = ImageTk.PhotoImage(self.image)
#             self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#             self.regions[self.region_id] = region
#
#     def reload_image(self):
#         self.image = self.image.copy()  # Reload original image
#         for region_id, region in self.regions.items():
#             draw = ImageDraw.Draw(self.image)
#             draw.polygon(region, fill=self.rgb_to_hex(self.color))
#         self.tk_image = ImageTk.PhotoImage(self.image)
#         self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
#
#     def rgb_to_hex(self, rgb):
#         return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
#
# if __name__ == "__main__":
#     root = tk.Tk()
#     root.title("Heightmap Editor")
#     app = HeightmapEditor(root)
#     root.mainloop()
