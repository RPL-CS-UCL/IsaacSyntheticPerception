from tkinter import * 
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, 
NavigationToolbar2Tk)
print("starting")
  
# plot function is created for 
# plotting the graph in 
# tkinter window
def plot():
  
    # the figure that will contain the plot
    fig = Figure(figsize = (5, 5),
                 dpi = 100)
  
    # list of squares
    y = [i**2 for i in range(101)]
  
    # adding the subplot
    plot1 = fig.add_subplot(111)
  
    # plotting the graph
    plot1.plot(y)
  
    # creating the Tkinter canvas
    # containing the Matplotlib figure
    canvas = FigureCanvasTkAgg(fig,
                               master = right_frame)  
    canvas.draw()
  
    # placing the canvas on the Tkinter window
    canvas.get_tk_widget().pack()#grid(row=0,column=0, padx=5, pady=5)
  
  
    # placing the toolbar on the Tkinter window
    canvas.get_tk_widget().pack()#grid(row=0,column=0, padx=5, pady=5)

# button that displays the plot
#plot_button = Button(master = window, 
#                     command = plot,
#                     height = 2, 
#                     width = 10,
#                     text = "Plot")
  
# place the button 
# in main window
#plot_button.pack()


def draw_main_menu(m_window):
    load_create_base_window_button = Button(master=m_window, command=plot, height=2, width=18, text="Create Base Grid")
    load_create_base_window_button.grid(row=0, column=0, padx=5, pady=5)

# the main Tkinter window
window = Tk()
root = window
  
# setting the title 
window.title('Main Menu')
  
# dimensions of the main window
window.geometry("500x500")
left_frame = Frame(root, width=200, height=400, bg='grey')
left_frame.grid(row=0, column=0, padx=10, pady=5)

right_frame = Frame(root, width=650, height=400, bg='grey')
right_frame.grid(row=0, column=1, padx=10, pady=5)
draw_main_menu(window) 
# run the gui
window.mainloop()
