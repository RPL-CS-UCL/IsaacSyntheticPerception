#
# import pptk
# from os.path import join
# import numpy as np
# import os, argparse, pickle
# import open3d as o3d
# import yaml
# from os.path import exists, join, isfile, dirname, abspath
#
# from PIL import Image
# import numpy as np
# import matplotlib.pyplot as plt
# import sys
# np.set_printoptions(threshold=sys.maxsize)
# # from main_SemanticKITTI import SemanticKITTI
# import sys
# def rgba2rgb( rgba, background=(255,255,255) ):
#     row, col, ch = rgba.shape
#
#     if ch == 3:
#         return rgba
#
#     assert ch == 4, 'RGBA image has 4 channels.'
#
#     rgb = np.zeros( (row, col, 3), dtype='float32' )
#     r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]
#
#     a = np.asarray( a, dtype='float32' ) / 255.0
#
#     R, G, B = background
#
#     rgb[:,:,0] = r * a + (1.0 - a) * R
#     rgb[:,:,1] = g * a + (1.0 - a) * G
#     rgb[:,:,2] = b * a + (1.0 - a) * B
#
#     return np.asarray( rgb, dtype='uint8' )
#
# def process_clouds(pc, cols):
#     print("begin proc")
#     pc_out = []
#     cols_out = []
#     for x in range(len(pc)):
#         for y in range(len(pc[x])):
#             pc_out.append(pc[x][y])
#             cols_out.append((cols[x][y][0]/255,cols[x][y][1]/255,cols[x][y][2]/255))
#     return np.array(pc_out), np.array(cols_out)
#
# def process_clouds2(pc, cols):
#     print("begin proc")
#     pc_out = []
#     cols_out = []
#     for x in range(len(pc)):
#         for y in range(len(pc[x])):
#             pc_out.append(pc[x][y])
#             cols_out.append(cols[x][y])
#     return np.array(pc_out), np.array(cols_out)
#
# def lin_col_to_norm(cols):
#     
#     print("linear: ", cols.shape[0])
#     new_cols = []
#     for i in range(0,cols.shape[0],4):
#         new_cols.append((cols[i]/255,cols[i+1]/255,cols[i+2]/255))
#     return np.array(new_cols)
#
# if __name__ == '__main__':
#
#     COLOR_MAP = {
#         0: (0, 0, 0),
#         1: (245, 150, 100),
#         2: (245, 230, 100),
#         3: (150, 60, 30),
#         4: (180, 30, 80),
#         5: (255, 0., 0),
#         6: (30, 30, 255),
#         7: (200, 40, 255),
#         8: (90, 30, 150),
#         9: (255, 0, 255),
#         10: (255, 150, 255),
#         11: (75, 0, 75),
#         12: (75, 0., 175),
#         13: (0, 200, 255),
#         14: (50, 120, 255),
#         15: (0, 175, 0),
#         16: (0, 60, 135),
#         17: (80, 240, 150),
#         18: (150, 240, 255),
#         19: (0, 0, 255),
#     }
#     for label in COLOR_MAP:
#         COLOR_MAP[label] = tuple(val/255 for val in COLOR_MAP[label])
#     seq_id = sys.argv[1:][0]
#     file_id = sys.argv[1:][1]
#     full_pc_path = f"{seq_id}/full_pointcloud/{file_id}.npy"
#     colour_path = f"{seq_id}/rgb/{file_id}.png"
#     full_pc_path = "_pc.npy"
#     colour_path = "_sem.npy"
#     im = np.load(colour_path, allow_pickle=True)
#
#     full_pc = np.load(full_pc_path,allow_pickle=True)
#     full_pc, im = process_clouds2(full_pc, im)
#     pcd = o3d.geometry.PointCloud()
#     colors = [COLOR_MAP[clr] for clr in im] 
#     pcd.points = o3d.utility.Vector3dVector(full_pc)
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     o3d.visualization.draw_geometries([pcd])
#     sys.exit()
# import numpy as np
# import open3d as o3d
# import matplotlib.cm as cm
#
# # Load the Velodyne points and labels from numpy files
# points = np.load('/home/jon/Documents/test/sequences/00/labels/60.npy')
# print(points)
# labels = np.load('/home/jon/Documents/test/sequences/00/velodyne/60.npy')
# print(labels)
#
# # Create a point cloud from the points
# point_cloud = o3d.geometry.PointCloud()
# point_cloud.points = o3d.utility.Vector3dVector(points)
#
# # Automatically generate colors for labels using a color map
# num_labels = np.max(labels) + 1
# color_map = cm.get_cmap('hsv', num_labels)  # You can choose any colormap you like
# colors = color_map(labels / num_labels)[:, :3]  # Normalize labels and get RGB values
#
# # Assign colors to each point based on its label
# point_cloud.colors = o3d.utility.Vector3dVector(colors)
#
# # Visualize the point cloud
# o3d.visualization.draw_geometries([point_cloud])

import os
import numpy as np
npy_sequences_folder = '/home/jon/Documents/test/sequences_raw'
output_sequences_folder = '/home/jon/Documents/test/sequences'

# Create output folder if it doesn't exist
os.makedirs(output_sequences_folder, exist_ok=True)

# Iterate over npy sequences
for sequence_folder in os.listdir(npy_sequences_folder):
    sequence_path = os.path.join(npy_sequences_folder, sequence_folder)
    
    # Check if the folder represents a sequence
    if os.path.isdir(sequence_path):
        velodyne_folder = os.path.join(sequence_path, 'velodyne')
        labels_folder = os.path.join(sequence_path, 'labels')
        
        # Create corresponding output sequence folder
        sequence_output_path = os.path.join(output_sequences_folder, sequence_folder)
        os.makedirs(sequence_output_path, exist_ok=True)
        
        # Create output folders for velodyne and labels
        velodyne_folder_output = os.path.join(sequence_output_path, 'velodyne')
        labels_folder_output = os.path.join(sequence_output_path, 'labels')
        os.makedirs(velodyne_folder_output, exist_ok=True)
        os.makedirs(labels_folder_output, exist_ok=True)
        
        # Iterate over files and save as .bin and .label
        for file_name in os.listdir(velodyne_folder):
            if file_name.endswith('.npy'):
                # Load points from numpy file
                points = np.load(os.path.join(velodyne_folder, file_name))
                
                # Save points as .bin
                points_file_name = file_name.replace('.npy', '.bin')
                points_file_output_path = os.path.join(velodyne_folder_output, points_file_name)
                points.astype('float32').tofile(points_file_output_path)
        
        for file_name in os.listdir(labels_folder):
            if file_name.endswith('.npy'):
                # Load labels from numpy file
                labels = np.load(os.path.join(labels_folder, file_name))
                
                # Save labels as .label
                labels_file_name = file_name.replace('.npy', '.label')
                labels_file_output_path = os.path.join(labels_folder_output, labels_file_name)
                labels.astype('uint32').tofile(labels_file_output_path)
