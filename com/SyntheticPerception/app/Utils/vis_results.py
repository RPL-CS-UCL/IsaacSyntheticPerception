
import pptk
from os.path import join
import numpy as np
import os, argparse, pickle
import open3d as o3d
import yaml
from os.path import exists, join, isfile, dirname, abspath

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import sys
np.set_printoptions(threshold=sys.maxsize)
# from main_SemanticKITTI import SemanticKITTI
import sys
def rgba2rgb( rgba, background=(255,255,255) ):
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros( (row, col, 3), dtype='float32' )
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]

    a = np.asarray( a, dtype='float32' ) / 255.0

    R, G, B = background

    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray( rgb, dtype='uint8' )

def process_clouds(pc, cols):
    print("begin proc")
    pc_out = []
    cols_out = []
    for x in range(len(pc)):
        for y in range(len(pc[x])):
            pc_out.append(pc[x][y])
            cols_out.append((cols[x][y][0]/255,cols[x][y][1]/255,cols[x][y][2]/255))
    return np.array(pc_out), np.array(cols_out)

def process_clouds2(pc, cols):
    print("begin proc")
    pc_out = []
    cols_out = []
    for x in range(len(pc)):
        for y in range(len(pc[x])):
            pc_out.append(pc[x][y])
            cols_out.append(cols[x][y])
    return np.array(pc_out), np.array(cols_out)

def lin_col_to_norm(cols):
    
    print("linear: ", cols.shape[0])
    new_cols = []
    for i in range(0,cols.shape[0],4):
        new_cols.append((cols[i]/255,cols[i+1]/255,cols[i+2]/255))
    return np.array(new_cols)

if __name__ == '__main__':

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
    seq_id = sys.argv[1:][0]
    file_id = sys.argv[1:][1]
    full_pc_path = f"{seq_id}/full_pointcloud/{file_id}.npy"
    colour_path = f"{seq_id}/rgb/{file_id}.png"
    full_pc_path = "_pc.npy"
    colour_path = "_sem.npy"
    im = np.load(colour_path, allow_pickle=True)

    full_pc = np.load(full_pc_path,allow_pickle=True)
    full_pc, im = process_clouds2(full_pc, im)
    pcd = o3d.geometry.PointCloud()
    colors = [COLOR_MAP[clr] for clr in im] 
    pcd.points = o3d.utility.Vector3dVector(full_pc)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])
    sys.exit()
