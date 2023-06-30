"""
This module handles area and point generation.
"""
import json
import os 
import numpy as np
import numpy.typing as npt
# from . import PerlinNoise
from . import PoissonDisk
import matplotlib.colors
from . import PerlinNoise
import matplotlib.pyplot as plt
from typing import Tuple

# import open3d as o3d
# import numpy as np
#
# from perlin_numpy import generate_perlin_noise_2d, generate_fractal_noise_2d
# from sklearn.preprocessing import normalize
# from perlin_noise import PerlinNoise
# import matplotlib.pyplot as plt
#
# # Step 1: Install Open3D if needed
# # !pip install open3d
#
# l = 256
# shape = (l, l)
# noise1 = PerlinNoise(octaves=3)
# noise2 = PerlinNoise(octaves=6)
# noise3 = PerlinNoise(octaves=12)
# noise4 = PerlinNoise(octaves=24)
#
# xpix, ypix = l, l
# pic = []
# for i in range(xpix):
#     row = []
#     for j in range(ypix):
#         noise_val = noise1([i / xpix, j / ypix])
#         noise_val += 0.5 * noise2([i / xpix, j / ypix])
#         noise_val += 0.25 * noise3([i / xpix, j / ypix])
#         noise_val += 0.125 * noise4([i / xpix, j / ypix])
#
#         row.append(noise_val)
#     pic.append(row)
# a = np.array(pic)
# arr = abs(a)
# points = []
# all_verts = []
# for x in range(len(arr)):
#     for y in range(len(arr)):
#         # vertex 0
#         mypoint = [x, y, arr[x][y]*20]
#
#         all_verts.append(mypoint)
#         points.append(mypoint)
# print(a)
#
# faces = []
# for i in range(1, len(all_verts) - 1):
#     face = [i - 1, i, i + 1]  # Create a face using three consecutive vertices
#     faces.append(face)
# # print('here 2 222 l, faces', faces)
# faces = []
# subdivisions= l - 1
# for j in range(subdivisions):
#     for i in range(subdivisions):
#         index = j * (subdivisions + 1) + i
#         face1 = [index, index + 1, index + subdivisions + 2]
#         face2 = [index, index + subdivisions + 2, index + subdivisions + 1]
#         faces.append(face1)
#         faces.append(face2)
# # Step 3: Create an array representing the mesh vertices
# vertices = np.array(points)  # Vertex 4
# print(vertices.shape)
# print(type(vertices))
#
# # Step 4: Create a TriangleMesh object and assign the vertices
# mesh = o3d.geometry.TriangleMesh()
# mesh.vertices = o3d.utility.Vector3dVector(vertices)
# print('here4')
#
# # Step 5: Define the triangle faces
# faces = np.array(faces)  # Triangle 2 (vertices 1, 3, 4)
# print('faces')
# print(faces)
#
# # Step 6: Assign the faces to the mesh
# mesh.triangles = o3d.utility.Vector3iVector(faces)
# mesh.paint_uniform_color([1, 0.706, 0])
#
# # Step 7: Compute the normals of the mesh
# mesh.compute_vertex_normals()
# mesh =mesh.filter_smooth_laplacian(number_of_iterations=10)
#
# mesh.compute_vertex_normals()
# # Step 8: Visualize the mesh
# o3d.visualization.draw_geometries([mesh])
#
# # Step 1: Define the list of face indices for splitting
# split_indices = [i for i in range(int(len(mesh.triangles)/2))]  # List of face indices for splitting
#
# # Step 3: Create empty TriangleMesh objects for split meshes
# mesh1 = o3d.geometry.TriangleMesh()
# mesh2 = o3d.geometry.TriangleMesh()
#
# # Step 4: Iterate through the face indices and assign each face to the corresponding mesh
# for index in range(np.asarray(mesh.triangles).shape[0]):
#     face = mesh.triangles[index]
#     if index in split_indices:
#         mesh1.triangles.append(face)
#     else:
#         mesh2.triangles.append(face)
#
# # Step 5: Assign the original vertices and vertex normals to split meshes
# mesh1.vertices = mesh.vertices
# mesh1.vertex_normals = mesh.vertex_normals
# mesh2.vertices = mesh.vertices
# mesh2.vertex_normals = mesh.vertex_normals
# mesh1 = mesh1.remove_unreferenced_vertices()
#
# mesh2 = mesh2.remove_unreferenced_vertices()
#
# mesh1.paint_uniform_color([1, 0, 2])
#
# mesh2.paint_uniform_color([1, 0.706, 1])
# # Step 6: Compute the normals for split meshes
# mesh1.compute_vertex_normals()
# mesh2.compute_vertex_normals()
#
# # Step 7: Visualize the split meshes
# o3d.visualization.draw_geometries([mesh1,mesh2])
# print('done')
# import bpy, math
# import numpy as np
def append_inside_area(
    area: npt.NDArray[np.float64],
    area_to_add: npt.NDArray[np.float64],
    area_value: float,
) -> npt.NDArray[np.float64]:
    """
    Function returns a new mask that is only within the first mask

    """
    mask_indices = np.where((area_to_add >= area_value) & (area != 0))
    area2 = np.copy(area)
    area2[mask_indices] = area_value  # area_value

    return area2


def append_to_area(
    area: npt.NDArray[np.float64],
    area_to_add: npt.NDArray[np.float64],
    area_value: float,
) -> npt.NDArray[np.float64]:
    """
    Function returns a mask appended to another one


    """
    mask_indices = np.where(area_to_add >= area_value)

    area[mask_indices] = area_value

    return area


def show_plot(area):
    cvals = [0, 1, 2, 3, 4]
    colors = ["lightgreen", "green", "yellow", "brown", "red"]

    norm = plt.Normalize(min(cvals), max(cvals))
    tuples = list(zip(map(norm, cvals), colors))
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("", tuples)
    plt.imshow(area, cmap=cmap, norm=norm)
    plt.colorbar()
    plt.show()


def fill_area(
    area: npt.NDArray[np.float64], size: int, region_value: int, object_value: int
) -> Tuple[npt.NDArray[np.float64],list]:
    # Generate points and fill the area with objects using Poisson
    points = PoissonDisk.Bridson_sampling(
        width=area.shape[0], height=area.shape[1], radius=size, k=30
    )
    new_points = []
    for p in points:
        x_int = int(p[0])
        y_int = int(p[1])
        if area[y_int][x_int] == region_value:
            area[y_int][x_int] = object_value
            new_points.append(p)

    return area, new_points


class ObjectPrim:
    def __init__(self, scale, scale_delta, y_rot, u_id, usd_path, class_name, poisson_size) -> None:
        self.object_scale = scale
        self.object_scale_delta = scale_delta
        self.allow_y_rot = y_rot
        self.unique_id = u_id
        self.usd_path = usd_path
        self.class_name = class_name
        self.poisson_size = poisson_size

    def __str__(self) -> str:
        return f"""
    {self.unique_id} 
            scale: {self.object_scale} +/- {self.object_scale_delta}  
            allow y rot: {self.allow_y_rot}  
            poisson size: {self.poisson_size} 
            class name: {self.class_name}  
            usd path: {self.usd_path}

    """
        pass

class WorldHandler:
    def __init__(self, world_path, object_path) -> None:
        # self.objects = []
        self.objects_dict = {}
        self._object_path = object_path
        self._world_path = world_path
        self.objects_to_spawn = {}
        self._WORLD_TO_POISSON_SCALE = 1.6

    def _read_objects(self):
        with open(self._object_path, 'r+') as infile:
            data = json.load(infile)
            print(data)
            for key in data:
                scale = data[key]["object_scale"]
                scale_delta =data[key]["object_scale_delta"]
                y_rot = data[key]["allow_y_rot"]
                u_id = key
                usd_path = data[key]["usd_path"]
                class_name = data[key]["class_name"]
                poisson_size = data[key]["poisson_size"]
                tmp = ObjectPrim(scale, scale_delta, y_rot, u_id, usd_path, class_name, poisson_size)
                # self.objects.append(tmp)
                self.objects_dict[u_id] = tmp

        print("Loaded the following objects")
        # for i in self.objects:
        #     print(i)

    def _read_world(self):
        print("here")
        self.objects_to_spawn = {}
        data = None
        with open(self._world_path, 'r+') as infile:
            data = json.load(infile)
        if data != None:
            n = data["size"] 
            arr = np.zeros((n, n))
            regions = data["regions"]
            for region_id in regions:
                region_id = str(region_id)

                new_arr = PerlinNoise.generate_region2(
                        seed=int(region_id),
                    shape=(n, n),
                    threshold=float(regions[region_id]["threshold"]),
                    show_plot=False,
                    region_value=int(region_id),
                )

                arr = append_to_area(
                    arr, new_arr, int(region_id)
                )
                # handle objects in the zone
                objs = regions[region_id]["objects"]
                if len(objs) > 0:
                    for obj_uid in objs:
                        # get corresponding object from objects
                        object_prim = self.objects_dict[str(obj_uid)]

                        area, coords = fill_area(arr, object_prim.poisson_size / self._WORLD_TO_POISSON_SCALE , int(region_id), 999)#object_prim.unique_id
                        self.objects_to_spawn[object_prim.unique_id] = coords 

                # now we need to deal with sub zones
                zones = regions[region_id]["zones"]
                for zone_id in zones:

                    new_arr = PerlinNoise.generate_region2(
                            seed=int(zone_id),
                        shape=(n, n),
                        threshold=float(zones[zone_id]["threshold"]),
                        show_plot=False,
                        region_value=int(zone_id),
                    )

                    zone_to_save =append_inside_area(
                        arr, new_arr, int(zone_id)
                    )
                    objs = zones[zone_id]["objects"]
                    if len(objs) > 0:
                        for obj_uid in objs:
                            # get corresponding object from objects
                            object_prim = self.objects_dict[obj_uid]

                            area, coords = fill_area(zone_to_save, object_prim.poisson_size / self._WORLD_TO_POISSON_SCALE  , int(zone_id), 999)
                            # print(coords)
                            self.objects_to_spawn[object_prim.unique_id] = coords 
        # print(objects_to_spawn)
                            
        pass


def generate_world_from_file(world_path, object_path):
    print("creating world obj")
    world = WorldHandler(world_path, object_path)
    print("reading objs")
    world._read_objects()
    print("reading world")
    world._read_world()
    return world.objects_to_spawn, world.objects_dict


def test_world():

    n = 256
    forrest_region = PerlinNoise.generate_region(shape=(n,n), threshold=0.5, show_plot=False)
    treeone_region = PerlinNoise.generate_region(
        shape=(n,n), threshold=0.5, show_plot=False, region_value=2
    )

    treeone_region2 = PerlinNoise.generate_region(
        shape=(n,n), threshold=0.5, show_plot=False, region_value=3
    )
    forrest_region_treeone = append_inside_area(np.array(forrest_region), np.array(treeone_region), 2.0)

    area = append_inside_area(np.array(forrest_region_treeone), np.array(treeone_region2), 3.0)

    sand_region = PerlinNoise.generate_region(shape=(n,n), threshold=0.3, show_plot=False, region_value=3)
    
    sand_region_two = PerlinNoise.generate_region(
        shape=(n,n), threshold=0.5, show_plot=False, region_value=4)

    sand_region_zones = append_inside_area(np.array(sand_region), np.array(sand_region_two), 4.0)
    #fill trees
    area, trees1 = fill_area(area, 3, 1, 10)

    area, trees2 = fill_area(area, 6, 2, 11)
    
    area, rocks = fill_area(area, 2,1, 12)
    area, rocks2 = fill_area(area, 2,2, 13)

    return trees1, trees2, rocks, rocks2


def test_func():
    print("running now")
    n = 256
    reg1 = PerlinNoise.generate_region(shape=(n,n), threshold=0.5, show_plot=False)
    reg2 = PerlinNoise.generate_region(
        shape=(n,n), threshold=0.5, show_plot=False, region_value=2
    )
    # area = append_to_area(np.array(reg1), np.array(reg2), 1.0)
    area = append_inside_area(np.array(reg1), np.array(reg2), 2.0)
    # print(np.unique(area))
    # plt.imshow(area)
    # plt.colorbar()
    # plt.show()
    # fill_area(reg1)
    area, n1 = fill_area(area, 3, 1, 3)
    area, n2 = fill_area(area, 15, 2, 4)
    return n1, n2


# if __name__ == "__main__":
    # reg1 = generate_region(shape=(2048, 2048), threshold=0.5, show_plot=False)
    # # print(reg1)
    # reg2 = generate_region(
    #     shape=(2048, 2048), threshold=0.5, show_plot=False, region_value=2
    # )
    # # area = append_to_area(np.array(reg1), np.array(reg2), 1.0)
    # area = append_inside_area(np.array(reg1), np.array(reg2), 2.0)
    # # print(np.unique(area))
    # # plt.imshow(area)
    # # plt.colorbar()
    # # plt.show()
    # # fill_area(reg1)
    # area = fill_area(area, 3, 1, 3)
    # ara = fill_area(area, 15, 2, 4)
    # show_plot(area)
