from matplotlib import pyplot as plt
import open3d as o3d
import numpy as np

def plot_depth(name):

    data =   np.load(f"{name}.npy", allow_pickle=True)
    plt.imshow(data, interpolation='nearest', cmap="plasma")
    plt.show()

def plot_image(name): 
    data =   np.load(f"{name}.npy", allow_pickle=True)
    plt.imshow(data, interpolation='nearest')
    plt.show()

def plot_sem(name):

    COLOR_MAP = {
        0: (0, 0, 0),
        1: (245, 150, 100),
        2: (245, 230, 100),
        3: (150, 60, 30),
        4: (180, 30, 80),
        5: (255, 0., 0),
        6: (30, 30, 255),
        7: (200, 40, 255),
        8: (90, 30, 150),
        9: (255, 0, 255),
        10: (255, 150, 255),
        11: (75, 0, 75),
        12: (75, 0., 175),
        13: (0, 200, 255),
        14: (50, 120, 255),
        15: (0, 175, 0),
        16: (0, 60, 135),
        17: (80, 240, 150),
        18: (150, 240, 255),
        19: (0, 0, 255),
    }
    for label in COLOR_MAP:
        COLOR_MAP[label] = tuple(val/255 for val in COLOR_MAP[label])
    data = np.load(f"{name}.npy", allow_pickle=True).item()["data"]
    print(data)
    
    outer = []
    for x in range(len(data)):
        inner = []
        for y in range(len(data[x])):
            inner.append(COLOR_MAP[data[x][y]])
        outer.append(inner)

    plt.imshow(outer, interpolation='nearest')
    plt.show()
    print(data)
    pass

plot_depth("depth")
plot_image("image")
plot_sem("sem")
