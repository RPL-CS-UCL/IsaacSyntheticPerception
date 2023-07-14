import tkinter as tk
from tkinter import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import os
import sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from PCG import AreaMaskGenerator
from PCG import PerlinNoise
from PCG.AreaMaskGenerator import ObjectPrim, WorldHandler

# def load_objects():
#     # Code for the "Load Objects" page
#     print('Load Objects page')
#
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import numpy as np
from matplotlib import colors

import json
import tkinter.ttk as ttk
from tkinter.filedialog import askopenfilename, askdirectory, asksaveasfile


class EntryInfo:
    def __init__(self, name, threshold):
        self.name = name
        self.threshold = threshold
        self.identifier = None
        self.color = None
        self.in_region = None
        self.objects_in_zone = []
        self.is_region = True
        self.material_path = None
        self.material_scale = None

    def get_objs_as_str(self):
        return ''.join(self.objects_in_zone)



class EnvTool:
    def __init__(self) -> None:
        self.worldHandler = WorldHandler(',', '')
        self.size =256
        self.seed = 0

        self.cbar = None

    def load_objects(self):
        # Code for the "Load Objects" page
        print('Load Objects page')
        filename = askopenfilename()
        self.worldHandler._object_path = filename
        self.worldHandler._read_objects()


    def write_data(self):
        data = {}
        entry_list = self.entry_list
        for entry in entry_list:
            if entry.is_region:
                data[entry.identifier] = {}
                data[entry.identifier]['objects'] = entry.objects_in_zone
                data[entry.identifier]['zones'] = {}
                data[entry.identifier]['threshold'] = entry.threshold
                data[entry.identifier]['material_path'] = entry.material_path
                data[entry.identifier]['material_scale'] =entry.material_scale
            else:
                # we are in a zone - get the region we are in
                id = int(entry.in_zone)
                print(id)
                # if not data[id]["zones"][entry.identifier]:
                if not id in data.keys():
                    data[id]['zones'][entry.identifier] = {}
                if not entry.identifier in data[id]['zones'].keys():
                    data[id]['zones'][entry.identifier] = {}

                data[id]['zones'][entry.identifier][
                    'objects'
                ] = entry.objects_in_zone

                data[id]['zones'][entry.identifier][
                    'threshold'
                ] = entry.threshold

                data[id]['zones'][entry.identifier]['material_path'] = entry.material_path
                data[id]['zones'][entry.identifier]['material_scale'] =entry.material_scale
            
        # json.dump(data)
        full_data = {}
        full_data['seed'] = self.seed
        full_data['regions'] = data
        full_data['size'] = self.size
        # folder_path = askdirectory()
        files = [('json', "*.json")]
        folder_path = asksaveasfile(filetypes=files,defaultextension=files)
        folder_path = folder_path.name

        with open(f'{folder_path}', 'w') as f:
            json.dump(full_data, f)
        print(full_data)

    # Function to delete an entry from the list
    def delete_entry(self, entry, index):
        entry.destroy()
        self.entry_list.pop(index)
        self.update_identifiers()
        self.update_plot()

    def update_identifiers(self):
        for i, entry_info in enumerate(self.entry_list):
            entry_info.identifier = i + 1

    def add_entry(self):
        name = self.input_entry1.get()
        threshold = self.input_entry2.get()
        parent_zone = self.input_entry3.get()
        mat_path = self.input_entry_mat_path.get()
        mat_scale = self.input_entry_mat_scale.get()
        if name and threshold:
            self.entry_frame = tk.Frame(self.entries_frame)
            self.entry_frame.pack(anchor='w')

            self.entry_info = EntryInfo(name, threshold)
            self.entry_info.material_path = mat_path
            self.entry_info.material_scale = mat_scale

            for i in self.listbx.curselection():
                self.entry_info.objects_in_zone.append(self.listbx.get(i))

            self.entry_info.identifier = len(self.entry_list) #+ 1
            id = self.entry_info.identifier
            self.entry_info.color = "BLACK"#generate_random_color()
            if parent_zone != '':
                self.entry_info.in_zone = parent_zone
                self.entry_info.is_region = False
            else:
                self.entry_info.in_zone = 0
                parent_zone = 0
            self.entry_list.append(self.entry_info)

            self.entry_label = tk.Label(
                self.entry_frame,
                text=f'ID: {id}, Name: {name}, Threshold: {threshold}, parent zone: {parent_zone}, objects: {self.entry_info.get_objs_as_str()}',
                fg=self.entry_info.color,
            )
            self.entry_label.pack(side='left')

            self.delete_button = tk.Button(
                self.entry_frame,
                text='Delete',
                command=lambda entry=self.entry_frame, index=len(
                    self.entry_list
                ) - 1: self.delete_entry(entry, index),
            )
            self.delete_button.pack(side='left')
            # entries_listbox.insert(
            #     tk.END, f'Name: {name}, Threshold: {threshold}'
            # )
            self.input_entry1.delete(0, tk.END)
            self.input_entry2.delete(0, tk.END)
            self.input_entry3.delete(0, tk.END)

            self.update_plot()

    def update_plot(self):
        # fig.clear()
        self.cbar.remove()
        self.ax.clear()
        self.arr = np.zeros((self.size, self.size))
        self.past_id = 0
        for entry in self.entry_list:
            print(
                'identigier ',
                entry.identifier,
                ' in int form ',
                int(entry.identifier),
            )
            # check the parent zone. if it is not 0 we need to generate it inside this zone
            # we want to keep both tho.
            # the inside zone one must not completely overwite the parent REGION
            # in this case we dont add it to the main array we just perfrom the calculation and save it
            print("here")
            print(self.size, entry.threshold)
            self.new_arr = PerlinNoise.generate_region2(
                seed=int(entry.identifier),
                shape=(self.size, self.size),
                threshold=float(entry.threshold),
                show_plot=False,
                region_value=int(entry.identifier),
            )
            # This zone will be saved and used later
            if entry.in_zone != 0:
                self.zone_to_save = AreaMaskGenerator.append_inside_area(
                    self.arr, self.new_arr, int(entry.identifier)
                )
                self.arr = self.zone_to_save
            else:
                print('Adding region to general area')
                self.arr = AreaMaskGenerator.append_to_area(
                    self.arr, self.new_arr, int(entry.identifier)
                )


        self.i = self.ax.imshow(self.arr)

        self.cbar = self.fig.colorbar(self.i)
        cbar_ticks = [
            int(e.identifier) for e in self.entry_list
        ]  # np.linspace(0.0, 1.0, num=6, endpoint=True)
        self.cbar.set_ticks(cbar_ticks)
        self.cbar.draw_all()
        # ax.bar(x, y, color=colors)
        # ax.set_xlabel('Entry')
        # ax.set_ylabel('Threshold')
        self.canvas.draw()

    def create_regions(self):
        # Code for the "Create Regions" page
        print('Create Regions page')

        # Create a new window for the "Create Regions" page
        self.regions_window = tk.Toplevel()
        self.regions_window.title('Create Regions')

        # First column: List of entries with delete buttons
        self.entries_frame = tk.Frame(self.regions_window)
        self.entries_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

        self.yscrollbar = Scrollbar(self.entries_frame)
        self.yscrollbar.pack(side=RIGHT, fill=Y)
        self.entries_label = tk.Label(self.entries_frame, text='Entries:')
        self.entries_label.pack()
        options = ['Option 1', 'Option 2', 'Option 3', 'Option 4', 'Option 5']
        options = [str(i) for i in range(100)]
        self.selected_items = []

        # List to store entry objects
        self.entry_list = []
        # Second column: Text entries and a button
        self.inputs_frame = tk.Frame(self.regions_window)
        self.inputs_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')

        self.input_label1 = tk.Label(self.inputs_frame, text='Name:')
        self.input_label1.pack()

        self.input_entry1 = tk.Entry(self.inputs_frame)
        self.input_entry1.pack()

        self.input_label2 = tk.Label(self.inputs_frame, text='Threshold:')
        self.input_label2.pack()

        self.input_entry2 = tk.Entry(self.inputs_frame)
        self.input_entry2.pack()

        self.input_label3 = tk.Label(self.inputs_frame, text='In zone ID:')
        self.input_label3.pack()

        self.input_entry3 = tk.Entry(self.inputs_frame)
        self.input_entry3.pack()

        self.input_label_mat_path = tk.Label(self.inputs_frame, text='Material Path')
        self.input_label_mat_path.pack()

        self.input_entry_mat_path = tk.Entry(self.inputs_frame)
        self.input_entry_mat_path.pack()
        self.input_label_mat_scale = tk.Label(self.inputs_frame, text='Material Scale')
        self.input_label_mat_scale.pack()

        self.input_entry_mat_scale = tk.Entry(self.inputs_frame)
        self.input_entry_mat_scale.pack()
        ttk.Label(self.inputs_frame, text='Add objects to zone').pack()

        self.input_label4 = tk.Label(self.inputs_frame, text='Add to zone with the ID of:')
        self.input_label4.pack()

        self.yscrollbar = Scrollbar(self.inputs_frame)
        self.yscrollbar.pack(side=RIGHT, fill=Y)
        self.listbx = Listbox(
            self.inputs_frame, selectmode='multiple', yscrollcommand=self.yscrollbar.set
        )

        self.listbx.pack(padx=10, pady=10, expand=YES, fill='both')
        x = []
        for item in self.worldHandler.objects:
            x.append(item.unique_id)

        for each_item in range(len(x)):

            self.listbx.insert(END, x[each_item])
            self.listbx.itemconfig(each_item, bg='white')
        self.yscrollbar.config(command=self.listbx.yview)

        self.process_button = tk.Button(
            self.inputs_frame, text='Add entry', command=self.add_entry
        )
        self.process_button.pack()
        # Third column: Empty column
        self.third_column_frame = tk.Frame(self.regions_window)
        self.third_column_frame.grid(row=0, column=2, padx=10, pady=10, sticky='nsew')

        self.save_all_button = tk.Button(
            self.inputs_frame, text='save all', command=self.write_data
        )

        self.save_all_button.pack()
        # Example Matplotlib plot
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.third_column_frame)
        self.canvas.get_tk_widget().pack()
        self.arr = np.zeros((self.size, self.size))
        self.i = self.ax.imshow(self.arr)
        self.cbar = plt.colorbar(self.i)
        self.cbar.ax.set_autoscale_on(True)



    def set_size(self):
        print(" =========== updating size to ==============")
        self.size = int(self.input_sizeentry.get())
        if not self.size or self.size < 0:
            self.size = 256
        print(self.size)
    def set_seed(self):
        self.seed = int(self.input_seed_entry.get())

    def main_page(self):
        self.main_window = tk.Tk()
        self.main_window.geometry("500x500")
        self.main_window.title('Main Window')

        self.load_objects_button = tk.Button(
            self.main_window, text='Load Objects', command=self.load_objects
        )
        self.load_objects_button.pack()


        self.input_sizelabel= tk.Label(self.main_window, text='World Size:')
        self.input_sizelabel.pack()

        self.input_sizeentry = tk.Entry(self.main_window)
        self.input_sizeentry.pack()



        self.set_size_button = tk.Button(
            self.main_window, text='set size', command=self.set_size
        )

        self.set_size_button.pack()

        self.input_seed_label= tk.Label(self.main_window, text='seed:')
        self.input_seed_label.pack()

        self.input_seed_entry = tk.Entry(self.main_window)
        self.input_seed_entry.pack()

        self.set_seed_button = tk.Button(
            self.main_window, text='set seed', command=self.set_seed
        )

        self.set_seed_button.pack()

        self.create_regions_button = tk.Button(
            self.main_window, text='Open map creator', command=self.create_regions
        )
        self.create_regions_button.pack()

        # create_zones_button = tk.Button(
        #     main_window, text='Create Zones', command=create_zones
        # )
        # create_zones_button.pack()

        self.main_window.mainloop()


if __name__ == '__main__':
    # main_page()
    tool = EnvTool()
    tool.main_page()
