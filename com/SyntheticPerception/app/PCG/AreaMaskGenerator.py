"""
This module handles area and point generation.
"""
from .MeshGenerator import MeshGen
import omni.kit.commands
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

from pxr import Usd, Sdf, Gf
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
            # area[y_int][x_int] = object_value
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
class TerrainPrim:
    def __init__(self, mesh_path, mat_path, scale = 0.001) -> None:
        self.mesh_path = mesh_path
        self.material_path = mat_path
        self.scale = scale

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
            # print(data)
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
        # print("here")
        self.objects_to_spawn = {}
        data = None
        objs_per_region = {}
        with open(self._world_path, 'r+') as infile:
            data = json.load(infile)
        if data != None:
            n = data["size"] 
            arr = np.zeros((n, n))
            total_arr = np.zeros((n, n))
            regions = data["regions"]
            terrain_info = {}
            # print( " == ", np.unique(total_arr))
            for region_id in regions:
                region_id = str(region_id)

                terrain_info[region_id] = TerrainPrim("",regions[region_id]["material_path"], regions[region_id]["material_scale"])
                # print("terrrain info key type ", type(region_id))
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
                total_arr = arr
                # handle objects in the zone
                objs = regions[region_id]["objects"]
                objs_per_region[region_id] = []
                if len(objs) > 0:
                    for obj_uid in objs:
                        # get corresponding object from objects
                        object_prim = self.objects_dict[str(obj_uid)]
                        objs_per_region[region_id].append(object_prim)

                # now we need to deal with sub zones
                zones = regions[region_id]["zones"]
                for zone_id in zones:

                    terrain_info[str(zone_id)] = TerrainPrim("",zones[zone_id]["material_path"],zones[zone_id]["material_scale"])
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

                    # print("zone == ", zone_id, "  ", zone_id)
                    total_arr = zone_to_save
                    objs = zones[zone_id]["objects"]


                    objs_per_region[zone_id] = []
                    if len(objs) > 0:
                        for obj_uid in objs:
                            # get corresponding object from objects
                            object_prim = self.objects_dict[obj_uid]

                            objs_per_region[zone_id].append(object_prim)

                            
            for key in objs_per_region:
                obs = objs_per_region[key]
                if len(obs) > 0:
                    for obj in obs: 
                        area, coords = fill_area(total_arr, obj.poisson_size / self._WORLD_TO_POISSON_SCALE  , int(key), 999)
                        self.objects_to_spawn[obj.unique_id] = coords 
            return total_arr, n, terrain_info


def generate_world_from_file(world_path, object_path):
    print("creating world obj")
    world = WorldHandler(world_path, object_path)
    print("reading objs")
    world._read_objects()
    print("reading world")
    res = world._read_world()
     
    terrain_mesh_paths = []
    if res:
        region_map, map_size, terrain_info= res
        # print(" ------- ")
        # print(map_size, 10, region_map.shape)
        # print(set(region_map.flatten()))
        # unique, counts = np.unique(region_map, return_counts=True)
        # print(dict(zip(unique, counts)))
        # return None

        m_path = 'C:/Users/jonem/Documents/Kit/apps/Isaac-Sim/exts/IsaacSyntheticPerception/com/SyntheticPerception/app/PCG'
        meshGen = MeshGen(map_size, 10, region_map, m_path)
        meshGen.generate_terrain_mesh()

        regs = list(np.unique(region_map))
        for key in terrain_info: 
            print(key)
            if float(key) in regs:
                terrain_info[str(key)].mesh_path = meshGen.final_mesh_paths_dict[int(key)]
        print(f"[AreaMaskGenerator] All terrain infos updated. Passing data back to main sample to genereate objects and load the terrain in.")


        return world.objects_to_spawn, world.objects_dict, terrain_info, meshGen#._points2#_noise_map_xy 
    return world.objects_to_spawn, world.objects_dict, None, None


#
# def test_world():
#
#     n = 256
#     forrest_region = PerlinNoise.generate_region(shape=(n,n), threshold=0.5, show_plot=False)
#     treeone_region = PerlinNoise.generate_region(
#         shape=(n,n), threshold=0.5, show_plot=False, region_value=2
#     )
#
#     treeone_region2 = PerlinNoise.generate_region(
#         shape=(n,n), threshold=0.5, show_plot=False, region_value=3
#     )
#     forrest_region_treeone = append_inside_area(np.array(forrest_region), np.array(treeone_region), 2.0)
#
#     area = append_inside_area(np.array(forrest_region_treeone), np.array(treeone_region2), 3.0)
#
#     sand_region = PerlinNoise.generate_region(shape=(n,n), threshold=0.3, show_plot=False, region_value=3)
#     
#     sand_region_two = PerlinNoise.generate_region(
#         shape=(n,n), threshold=0.5, show_plot=False, region_value=4)
#
#     sand_region_zones = append_inside_area(np.array(sand_region), np.array(sand_region_two), 4.0)
#     #fill trees
#     area, trees1 = fill_area(area, 3, 1, 10)
#
#     area, trees2 = fill_area(area, 6, 2, 11)
#     
#     area, rocks = fill_area(area, 2,1, 12)
#     area, rocks2 = fill_area(area, 2,2, 13)
#
#     return trees1, trees2, rocks, rocks2
#
#
# def test_func():
#     print("running now")
#     n = 256
#     reg1 = PerlinNoise.generate_region(shape=(n,n), threshold=0.5, show_plot=False)
#     reg2 = PerlinNoise.generate_region(
#         shape=(n,n), threshold=0.5, show_plot=False, region_value=2
#     )
#     # area = append_to_area(np.array(reg1), np.array(reg2), 1.0)
#     area = append_inside_area(np.array(reg1), np.array(reg2), 2.0)
#     # print(np.unique(area))
#     # plt.imshow(area)
#     # plt.colorbar()
#     # plt.show()
#     # fill_area(reg1)
#     area, n1 = fill_area(area, 3, 1, 3)
#     area, n2 = fill_area(area, 15, 2, 4)
#     return n1, n2

