import numpy as np
import open3d as o3d
import numpy as np
import glob
import random
from tqdm import tqdm
id = 100
from pathlib import Path
import sys
def vis_pc(pc):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pcd])

distance = 200
data_path = "/home/jon/Desktop/SparseNewDS"
folder_path = f"{data_path}/velodyne/"
core_path="/home/jon/Documents/Datasets/SparseFinal/sequences/"
def fix_orb_around_pc(data , distance):
    # print(" ================= ")
    start = np.array([0.0,0.0,0.0])

    points =np.concatenate(data,axis=0)

    # print(f"Points before: {len(points)}")
    new_arr = []
    indexes_to_remove = []
    for i,point in enumerate(points):

        dist = np.linalg.norm(start - point)
        # print(dist)
        if dist<distance:
            new_arr.append(point)
        else:
            indexes_to_remove.append(i)

    # print(f"Points after: {len(new_arr)}")
    limit = 4096*5
    if len(new_arr) < limit:
        print("array too small")
    return np.array(new_arr), indexes_to_remove
# load mappings
# map_dict = {}
# print(map_dict)
# sys.exit()
#
# {1: 'Asphalt', 3: 'Carpet_Beige', 2: 'Carpet_Pattern_Squares_Multi', 4: 'ladder', 5: 'sofa', 6: 'table', 7: 'tree'}
class_names = {
        0:"unlabled",
        1:"ground",
        2:"tree",
        3:"vegetation",
        4:"ladder",
        5:"sofa",
        6:"table",
        7:"bicycle",
        8:"pole",
        9:"fence",
        }
class_to_id_remap= {
        "Grass_Countryside": 1,
        "Leaves" : 2,
        "Carpet_Pattern_Squares_Multi":1,
        "tree":2,
        "vegetation":3,
        "Asphalt": 1,
        "Carpet_Beige":1,
        "ladder":4,
        "sofa":5,
        "table":6,
        "bicycle":7,
        "fence":9,
        "pole":8,
        "sign":8,

        }
computed_remap = {}

mappings = np.load(f"{data_path}/mapping.npy", allow_pickle=True)
print(mappings)
# print(np.unique(mappings,axis=2))
unique_dict = {}
for row in mappings:
    unique_dict[row[3]] = row[2]
print(unique_dict)
# sys.exit()
for tup in mappings:
    current_val = tup[2]
    class_name = tup[3]
    real_class_val = class_to_id_remap[class_name]
    computed_remap[current_val] = real_class_val
print("Computed remap")
print(computed_remap)
mapping = computed_remap
# {3: 'Grass_Countryside', 1: 'Leaves', 2: 'Carpet_Pattern_Squares_Multi', 4: 'tree', 5: 'vegetation'}

Path(core_path+"00/velodyne").mkdir(parents=True, exist_ok=True)
Path(core_path+"00/labels").mkdir(parents=True, exist_ok=True)

txtfiles = glob.glob(f"{folder_path}/*.npy")
txtfiles = sorted(txtfiles)
num_files = len(txtfiles)
num_seq = 8
num_files_per_seq = int(num_files/num_seq)
seq_id = 0
seq_id_addresses = []
count = 0
pcs_removed = 0
for seq_id in range(num_seq):
    Path(core_path+f"{seq_id:02d}/velodyne/").mkdir(parents=True, exist_ok=True)
    Path(core_path+f"{seq_id:02d}/labels/").mkdir(parents=True, exist_ok=True)
    seq_id_addresses.append(0)

for file in tqdm(txtfiles):
    id_name = file.split("/")[-1]
    data = np.load(file)

    # print(data, len(data))
    if len(data) == 0:
        # print("data too small")
        pcs_removed +=1
        continue

    # now handle the labels
    labels = np.load(f"{data_path}/velodyneLabels/{id_name}")

    labels = np.concatenate(labels,axis=0)

    if len(labels) == 0:
        continue
    k = np.array(list(mapping.keys()))
    v = np.array(list(mapping.values()))

    out= np.zeros_like(labels)
    for key,val in zip(k,v):
        out[labels==key] = val
    labels = out
    original_pc, inds_to_remove = fix_orb_around_pc(data, distance) 
    print(original_pc)
    print(original_pc.shape)
    # vis_pc(original_pc)
    labels = np.delete(labels, inds_to_remove)

    mu, sigma = 0, 0.1 
    noise = np.random.normal(mu, sigma, [original_pc.shape[0],original_pc.shape[1]]) 
    noisified_pc = original_pc + noise
    # vis_pc(noisified_pc)
    limit = 4096*5
    if noisified_pc.shape[0] <= limit:
        pcs_removed += 1
        continue
    # print(noisified_pc)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(noisified_pc)
    # o3d.visualization.draw_geometries([pcd])
    seq_id +=1
    if seq_id >= num_seq:
    	seq_id = 0
    # sys.exit()
    """
    if count >= num_files_per_seq:
        seq_id+=1
        count = 0

        Path(core_path+f"{seq_id:02d}/velodyne/").mkdir(parents=True, exist_ok=True)
        Path(core_path+f"{seq_id:02d}/labels/").mkdir(parents=True, exist_ok=True)
    """
    id_name = count
    id_name = seq_id_addresses[seq_id]
    seq_id_addresses[seq_id]+=1
    np.save(f"{core_path}{seq_id:02d}/velodyne/{id_name}",noisified_pc)
    np.save(f"{core_path}{seq_id:02d}/labels/{id_name}",labels)

    count +=1
print(f"removed {pcs_removed} files") 
print(f"kept {count}")
"""

re_map_natural = {0:0,
        1:2, # leaves to tree
        2:1, #carpet to ground
        3:1, #grass ground to ground
        4:2, #tree to tree
        5:3 # veg to veg

        }
re_map_manmade = {
        1:1, # asphalt to ground
        2:1, # carpet to ground
        3:1,#carpet to ground
        4:4,# ladder to ladder
        5:5,# sofa to sofa
        6:6,#Table to table
        7:2,# tree to tree

        }
mapping= re_map_natural
mapping = re_map_manmade
"""
