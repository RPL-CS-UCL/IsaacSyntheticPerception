import tkinter as tk

from tkinter import *
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)

print('starting')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from matplotlib.figure import Figure

# plot function is created for
# plotting the graph in
# tkinterim window
import os


import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

sys.path.append(os.path.dirname(SCRIPT_DIR))
from PCG import AreaMaskGenerator
from PCG import PerlinNoise

# def load_objects():
#     # Code for the "Load Objects" page
#     print('Load Objects page')
#
#
# def create_regions():
#     # Code for the "Create Regions" page
#     print('Create Regions page')
#
#     # Create a new window for the "Create Regions" page
#     regions_window = tk.Toplevel()
#     regions_window.title('Create Regions')
#
#     # First column: List of entries with delete buttons
#     entries_frame = tk.Frame(regions_window)
#     entries_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')
#
#     entries_label = tk.Label(entries_frame, text='Entries:')
#     entries_label.pack()
#
#     # Function to delete an entry from the list
#     def delete_entry(entry):
#         entry.destroy()
#
#     # Add some example entries
#     for i in range(5):
#         entry_frame = tk.Frame(entries_frame)
#         entry_frame.pack(anchor='w')
#
#         entry_text = f'Entry {i+1}'
#         entry_label = tk.Label(entry_frame, text=entry_text)
#         entry_label.pack(side='left')
#
#         delete_button = tk.Button(
#             entry_frame,
#             text='Delete',
#             command=lambda entry=entry_frame: delete_entry(entry),
#         )
#         delete_button.pack(side='left')
#
#     # Second column: Text entries and a button
#     inputs_frame = tk.Frame(regions_window)
#     inputs_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')
#
#     input_label1 = tk.Label(inputs_frame, text='Input 1:')
#     input_label1.pack()
#
#     input_entry1 = tk.Entry(inputs_frame)
#     input_entry1.pack()
#
#     input_label2 = tk.Label(inputs_frame, text='Input 2:')
#     input_label2.pack()
#
#     input_entry2 = tk.Entry(inputs_frame)
#     input_entry2.pack()
#
#     def process_inputs():
#         input1 = input_entry1.get()
#         input2 = input_entry2.get()
#         print('Input 1:', input1)
#         print('Input 2:', input2)
#         input_entry1.delete(0, tk.END)
#         input_entry2.delete(0, tk.END)
#
#     process_button = tk.Button(
#         inputs_frame, text='Process', command=process_inputs
#     )
#     process_button.pack()
#
#     # Third column: Empty column
#
#
# def create_zones():
#     # Code for the "Create Zones" page
#     print('Create Zones page')
#
#
# def main_page():
#     main_window = tk.Tk()
#     main_window.title('Main Window')
#
#     load_objects_button = tk.Button(
#         main_window, text='Load Objects', command=load_objects
#     )
#     load_objects_button.pack()
#
#     create_regions_button = tk.Button(
#         main_window, text='Create Regions', command=create_regions
#     )
#     create_regions_button.pack()
#
#     create_zones_button = tk.Button(
#         main_window, text='Create Zones', command=create_zones
#     )
#     create_zones_button.pack()
#
#     main_window.mainloop()
#
#
# if __name__ == '__main__':
#     main_page()
# import tkinter as tk
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#
#
# def load_objects():
#     # Code for the "Load Objects" page
#     print('Load Objects page')
#
#
# def create_regions():
#     # Code for the "Create Regions" page
#     print('Create Regions page')
#
#     # Create a new window for the "Create Regions" page
#     regions_window = tk.Toplevel()
#     regions_window.title('Create Regions')
#
#     # First column: List of entries with delete buttons
#     entries_frame = tk.Frame(regions_window)
#     entries_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')
#
#     entries_label = tk.Label(entries_frame, text='Entries:')
#     entries_label.pack()
#
#     # Function to delete an entry from the list
#     def delete_entry(entry):
#         entry.destroy()
#
#     def add_entry():
#         new_entry_text = input_entry1.get()
#         if new_entry_text:
#             entry_frame = tk.Frame(entries_frame)
#             entry_frame.pack(anchor='w')
#
#             entry_label = tk.Label(entry_frame, text=new_entry_text)
#             entry_label.pack(side='left')
#
#             delete_button = tk.Button(
#                 entry_frame,
#                 text='Delete',
#                 command=lambda entry=entry_frame: delete_entry(entry),
#             )
#             delete_button.pack(side='left')
#
#             input_entry1.delete(0, tk.END)
#
#     # Second column: Text entries and a button
#     inputs_frame = tk.Frame(regions_window)
#     inputs_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')
#
#     input_label1 = tk.Label(inputs_frame, text='Input 1:')
#     input_label1.pack()
#
#     input_entry1 = tk.Entry(inputs_frame)
#     input_entry1.pack()
#
#     process_button = tk.Button(inputs_frame, text='Process', command=add_entry)
#     process_button.pack()
#
#     # Third column: Empty column
#     third_column_frame = tk.Frame(regions_window)
#     third_column_frame.grid(row=0, column=2, padx=10, pady=10, sticky='nsew')
#     n = 256
#     forrest_region = PerlinNoise.generate_region(
#         shape=(n, n), threshold=0.5, show_plot=False
#     )
#
#     # Example Matplotlib plot
#     fig, ax = plt.subplots()
#     # ax.plot([1, 2, 3, 4, 5], [1, 4, 2, 3, 5])
#     ax.imshow(forrest_region)
#     ax.set_xlabel('X-axis')
#     ax.set_ylabel('Y-axis')
#     canvas = FigureCanvasTkAgg(fig, master=third_column_frame)
#     canvas.draw()
#     canvas.get_tk_widget().pack()
#
#
# def create_zones():
#     # Code for the "Create Zones" page
#     print('Create Zones page')
#
#
# def main_page():
#     main_window = tk.Tk()
#     main_window.title('Main Window')
#
#     load_objects_button = tk.Button(
#         main_window, text='Load Objects', command=load_objects
#     )
#     load_objects_button.pack()
#
#     create_regions_button = tk.Button(
#         main_window, text='Create Regions', command=create_regions
#     )
#     create_regions_button.pack()
#
#     create_zones_button = tk.Button(
#         main_window, text='Create Zones', command=create_zones
#     )
#     create_zones_button.pack()
#
#     main_window.mainloop()
#
#
# if __name__ == '__main__':
#     main_page()
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import random
import numpy as np
from matplotlib import colors

#
#
# class EntryInfo:
#     def __init__(self, name, threshold):
#         self.name = name
#         self.threshold = threshold
#         self.identifier = None
#         self.color = None
#
#
# def load_objects():
#     # Code for the "Load Objects" page
#     print('Load Objects page')
#
#
# def create_regions():
#     # Code for the "Create Regions" page
#     print('Create Regions page')
#
#     # Create a new window for the "Create Regions" page
#     regions_window = tk.Toplevel()
#     regions_window.title('Create Regions')
#
#     # First column: List of entries with delete buttons
#     entries_frame = tk.Frame(regions_window)
#     entries_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')
#
#     entries_label = tk.Label(entries_frame, text='Entries:')
#     entries_label.pack()
#
#     # List to store entry objects
#     entry_list = []
#
#     # Function to delete an entry from the list
#     def delete_entry(entry, index):
#         entry.destroy()
#         entry_list.pop(index)
#         update_plot()
#
#     def add_entry():
#         name = input_entry1.get()
#         threshold = input_entry2.get()
#         if name and threshold:
#             entry_frame = tk.Frame(entries_frame)
#             entry_frame.pack(anchor='w')
#
#             entry_info = EntryInfo(name, threshold)
#             entry_info.identifier = len(entry_list) + 1
#             entry_info.color = generate_random_color()
#             entry_list.append(entry_info)
#
#             entry_label = tk.Label(
#                 entry_frame,
#                 text=f'Name: {name}, Threshold: {threshold}',
#                 fg=entry_info.color,
#             )
#             entry_label.pack(side='left')
#
#             delete_button = tk.Button(
#                 entry_frame,
#                 text='Delete',
#                 command=lambda entry=entry_frame, index=len(
#                     entry_list
#                 ) - 1: delete_entry(entry, index),
#             )
#             delete_button.pack(side='left')
#
#             input_entry1.delete(0, tk.END)
#             input_entry2.delete(0, tk.END)
#
#             update_plot()
#
# def update_plot():
#     ax.clear()
#     n = 256
#     arr = np.zeros((n, n))
#     past_id = 0
#     for entry in entry_list:
#         new_arr = PerlinNoise.generate_region(
#             shape=(n, n),
#             threshold=float(entry.threshold),
#             show_plot=False,
#             region_value=int(entry.identifier),
#         )
#         print(new_arr)
#         arr = AreaMaskGenerator.append_to_area(
#             arr, new_arr, int(entry.identifier)
#         )
#         print(arr)
#     x = range(len(entry_list))
#     y = [float(entry.threshold) for entry in entry_list]
#     colors_ = [f'#{0:02x}{0:02x}{0:02x}']
#     for entry in entry_list:
#         colors_.append(entry.color)
#     cmap = colors.ListedColormap(colors_)
#     bounds = [0]
#     for entry in entry_list:
#         bounds.append(int(entry.identifier))
#     norm = colors.BoundaryNorm(bounds, cmap.N)
#     print(arr)
#     ax.imshow(
#         arr, interpolation='nearest', origin='lower', cmap=cmap, norm=norm
#     )
#     # ax.bar(x, y, color=colors)
#     # ax.set_xlabel('Entry')
#     # ax.set_ylabel('Threshold')
#     canvas.draw()
#
#     def generate_random_color():
#         r = random.randint(0, 255)
#         g = random.randint(0, 255)
#         b = random.randint(0, 255)
#         color = f'#{r:02x}{g:02x}{b:02x}'
#         return color
#
#     # Second column: Text entries and a button
#     inputs_frame = tk.Frame(regions_window)
#     inputs_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')
#
#     input_label1 = tk.Label(inputs_frame, text='Name:')
#     input_label1.pack()
#
#     input_entry1 = tk.Entry(inputs_frame)
#     input_entry1.pack()
#
#     input_label2 = tk.Label(inputs_frame, text='Threshold:')
#     input_label2.pack()
#
#     input_entry2 = tk.Entry(inputs_frame)
#     input_entry2.pack()
#
#     process_button = tk.Button(
#         inputs_frame, text='Add Entry', command=add_entry
#     )
#     process_button.pack()
#
#     # Third column: Empty column
#     third_column_frame = tk.Frame(regions_window)
#     third_column_frame.grid(row=0, column=2, padx=10, pady=10, sticky='nsew')
#
#     # Example Matplotlib plot
#     fig, ax = plt.subplots()
#     canvas = FigureCanvasTkAgg(fig, master=third_column_frame)
#     canvas.get_tk_widget().pack()
#
#
# def create_zones():
#     # Code for the "Create Zones" page
#     print('Create Zones page')
#
#
# def main_page():
#     main_window = tk.Tk()
#     main_window.title('Main Window')
#
#     load_objects_button = tk.Button(
#         main_window, text='Load Objects', command=load_objects
#     )
#     load_objects_button.pack()
#
#     create_regions_button = tk.Button(
#         main_window, text='Create Regions', command=create_regions
#     )
#     create_regions_button.pack()
#
#     create_zones_button = tk.Button(
#         main_window, text='Create Zones', command=create_zones
#     )
#     create_zones_button.pack()
#
#     main_window.mainloop()
#
#
# if __name__ == '__main__':
#     main_page()
cbar = None

import tkinter.ttk as ttk


class EntryInfo:
    def __init__(self, name, threshold):
        self.name = name
        self.threshold = threshold
        self.identifier = None
        self.color = None
        self.in_region = None
        self.objects_in_zone = []

    def get_objs_as_str(self):
        return "".join(self.objects_in_zone)


def load_objects():
    # Code for the "Load Objects" page
    print('Load Objects page')


def create_regions():
    # Code for the "Create Regions" page
    print('Create Regions page')

    # Create a new window for the "Create Regions" page
    regions_window = tk.Toplevel()
    regions_window.title('Create Regions')

    # First column: List of entries with delete buttons
    entries_frame = tk.Frame(regions_window)
    entries_frame.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')

    entries_label = tk.Label(entries_frame, text='Entries:')
    entries_label.pack()
    options = ['Option 1', 'Option 2', 'Option 3', 'Option 4', 'Option 5']
    options = [str(i) for i in range(100)]
    selected_items = []

    # List to store entry objects
    entry_list = []

    # Function to delete an entry from the list
    def delete_entry(entry, index):
        entry.destroy()
        entry_list.pop(index)
        update_identifiers()
        update_plot()

    def update_identifiers():
        for i, entry_info in enumerate(entry_list):
            entry_info.identifier = i + 1

    def add_entry():
        name = input_entry1.get()
        threshold = input_entry2.get()
        parent_zone = input_entry3.get()
        if name and threshold:
            entry_frame = tk.Frame(entries_frame)
            entry_frame.pack(anchor='w')

            entry_info = EntryInfo(name, threshold)

            for i in listbx.curselection():
                entry_info.objects_in_zone.append(listbx.get(i))

            entry_info.identifier = len(entry_list) + 1
            id = entry_info.identifier
            entry_info.color = generate_random_color()
            if parent_zone != '':
                entry_info.in_zone = parent_zone
            else:
                entry_info.in_zone = 0
                parent_zone = 0
            entry_list.append(entry_info)

            entry_label = tk.Label(
                entry_frame,
                text=f'ID: {id}, Name: {name}, Threshold: {threshold}, parent zone: {parent_zone}, objects: {entry_info.get_objs_as_str()}',
                fg=entry_info.color,
            )
            entry_label.pack(side='left')

            delete_button = tk.Button(
                entry_frame,
                text='Delete',
                command=lambda entry=entry_frame, index=len(
                    entry_list
                ) - 1: delete_entry(entry, index),
            )
            delete_button.pack(side='left')
            # entries_listbox.insert(
            #     tk.END, f'Name: {name}, Threshold: {threshold}'
            # )
            input_entry1.delete(0, tk.END)
            input_entry2.delete(0, tk.END)
            input_entry3.delete(0, tk.END)

            update_plot()

    def update_plot():
        # fig.clear()
        global cbar
        cbar.remove()
        ax.clear()
        n = 256
        arr = np.zeros((n, n))
        past_id = 0
        for entry in entry_list:
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
            new_arr = PerlinNoise.generate_region(
                shape=(n, n),
                threshold=float(entry.threshold),
                show_plot=False,
                region_value=int(entry.identifier),
            )
            # This zone will be saved and used later
            if entry.in_zone != 0:
                zone_to_save = AreaMaskGenerator.append_inside_area(
                    arr, new_arr, int(entry.identifer)
                )
            else:
                arr = AreaMaskGenerator.append_to_area(
                    arr, new_arr, int(entry.identifier)
                )
        if len(entry_list) > 100:
            # colors_ = [f'#{0:02x}{0:02x}{0:02x}']
            colors_ = []
            for entry in entry_list:
                colors_.append(entry.color)
            cmap = colors.ListedColormap(colors_)
            bounds = []
            for entry in entry_list:
                bounds.append(int(entry.identifier))
            norm = colors.BoundaryNorm(bounds, cmap.N)
            print(arr)
            ax.imshow(
                arr,
                interpolation='nearest',
                origin='lower',
                cmap=cmap,
                norm=norm,
            )
        else:
            i = ax.imshow(arr)

            cbar = fig.colorbar(i)
            cbar_ticks = [
                int(e.identifier) for e in entry_list
            ]  # np.linspace(0.0, 1.0, num=6, endpoint=True)
            cbar.set_ticks(cbar_ticks)
            cbar.draw_all()
        # ax.bar(x, y, color=colors)
        # ax.set_xlabel('Entry')
        # ax.set_ylabel('Threshold')
        canvas.draw()

    def extract_regions(arr):
        regions = []

        for entry in entry_list:
            if int(entry.in_zone) == 0:
                # remove all the non identifier values in the array and save it
                mask_indices = np.where(arr != int(entry.identifier))
                area2 = np.copy(arr)
                area2[mask_indices] = 0  # area_value
                regions.append(area2)

    def generate_random_color():
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        color = f'#{r:02x}{g:02x}{b:02x}'
        return color

    # Second column: Text entries and a button
    inputs_frame = tk.Frame(regions_window)
    inputs_frame.grid(row=0, column=1, padx=10, pady=10, sticky='nsew')

    input_label1 = tk.Label(inputs_frame, text='Name:')
    input_label1.pack()

    input_entry1 = tk.Entry(inputs_frame)
    input_entry1.pack()

    input_label2 = tk.Label(inputs_frame, text='Threshold:')
    input_label2.pack()

    input_entry2 = tk.Entry(inputs_frame)
    input_entry2.pack()

    input_label3 = tk.Label(inputs_frame, text='In zone ID:')
    input_label3.pack()

    input_entry3 = tk.Entry(inputs_frame)
    input_entry3.pack()
    # process_button = tk.Button(
    #     inputs_frame, text='Add Entry', command=add_entry
    # )
    # process_button.pack()
    # separator = ttk.Separator(inputs_frame, orient='horizontal')
    # separator.pack(fill='x')
    ttk.Label(inputs_frame, text='Add objects to zone').pack()

    input_label4 = tk.Label(inputs_frame, text='Add to zone with the ID of:')
    input_label4.pack()

    input_entry4 = tk.Entry(inputs_frame)
    input_entry4.pack()
    # combobox = ttk.Combobox(
    #     inputs_frame,
    #     values=options,
    #     width=25,
    #     state='readonly',
    #     justify='left',
    #     selectmode="multiple",
    # )
    # combobox.set('Select Options')
    # combobox.pack(padx=10, pady=10)
    yscrollbar = Scrollbar(inputs_frame)
    yscrollbar.pack(side=RIGHT, fill=Y)
    listbx = Listbox(
        inputs_frame, selectmode='multiple', yscrollcommand=yscrollbar.set
    )

    listbx.pack(padx=10, pady=10, expand=YES, fill='both')
    x = [

        'C',
        'C++',
        'C#',
        'Java',
        'Python',
        'R',
        'Go',
        'Ruby',
        'JavaScript',
        'Swift',
        'SQL',
        'Perl',
        'XML',
        'C',
        'C++',
        'C#',
        'Java',
        'Python',
        'R',
        'Go',
        'Ruby',
        'JavaScript',
        'Swift',
        'SQL',
        'Perl',
        'XML',
        'C',
        'C++',
        'C#',
        'Java',
        'Python',
        'R',
        'Go',
        'Ruby',
        'JavaScript',
        'Swift',
        'SQL',
        'Perl',
        'XML',
    ]

    for each_item in range(len(x)):

        listbx.insert(END, x[each_item])
        listbx.itemconfig(each_item, bg='white')
    yscrollbar.config(command=listbx.yview)

    process_button = tk.Button(
        inputs_frame, text='Add entry', command=add_entry
    )
    process_button.pack()
    # Third column: Empty column
    third_column_frame = tk.Frame(regions_window)
    third_column_frame.grid(row=0, column=2, padx=10, pady=10, sticky='nsew')

    # Example Matplotlib plot
    fig, ax = plt.subplots()
    canvas = FigureCanvasTkAgg(fig, master=third_column_frame)
    canvas.get_tk_widget().pack()
    n = 256
    global cbar
    arr = np.zeros((n, n))
    i = ax.imshow(arr)
    cbar = plt.colorbar(i)
    cbar.ax.set_autoscale_on(True)


def create_zones():
    # Code for the "Create Zones" page
    print('Create Zones page')


def main_page():
    main_window = tk.Tk()
    main_window.title('Main Window')

    load_objects_button = tk.Button(
        main_window, text='Load Objects', command=load_objects
    )
    load_objects_button.pack()

    create_regions_button = tk.Button(
        main_window, text='Create Regions', command=create_regions
    )
    create_regions_button.pack()

    create_zones_button = tk.Button(
        main_window, text='Create Zones', command=create_zones
    )
    create_zones_button.pack()

    main_window.mainloop()


if __name__ == '__main__':
    main_page()
