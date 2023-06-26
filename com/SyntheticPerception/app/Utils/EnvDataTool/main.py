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


def plot():

    # the figure that will contain the plot
    fig = Figure(figsize=(5, 5), dpi=100)

    # adding the subplot
    plot1 = fig.add_subplot(111)

    n = 256
    forrest_region = PerlinNoise.generate_region(
        shape=(n, n), threshold=0.5, show_plot=False
    )
    # plotting the graph
    plot1.imshow(forrest_region)

    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig, master=right_frame)
    canvas.draw()

    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().pack()  # grid(row=0,column=0, padx=5, pady=5)

    # placing the toolbar on the Tkinter window
    # canvas.get_tk_widget().pack()  # grid(row=0,column=0, padx=5, pady=5)
    # Creating Toolbar using Matplotlib

    toolbar = NavigationToolbar2Tk(canvas, right_frame)

    toolbar.update()

    canvas.get_tk_widget().pack()


# button that displays the plot
# plot_button = Button(master = window,
#                     command = plot,
#                     height = 2,
#                     width = 10,
#                     text = "Plot")

# place the button
# in main window
# plot_button.pack()


def draw_main_menu(m_window):
    load_create_base_window_button = Button(
        master=m_window,
        command=plot,
        height=2,
        width=18,
        text='Create Base Grid',
    )
    load_create_base_window_button.grid(row=0, column=0, padx=5, pady=5)


# the main Tkinter window
window = Tk()
root = window

# setting the title
window.title('Main Menu')

# dimensions of the main window
window.geometry('800x800')
left_frame = Frame(root, width=200, height=400, bg='grey')
left_frame.grid(row=0, column=0, padx=10, pady=5)

right_frame = Frame(root, width=650, height=400, bg='grey')
right_frame.grid(row=0, column=1, padx=10, pady=5)
draw_main_menu(window)
# run the gui
window.mainloop()
