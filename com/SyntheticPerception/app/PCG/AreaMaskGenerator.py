"""
This module handles area and point generation.
"""
from .MeshGenerator import MeshGen

# import omni.kit.commands
import json
import numpy as np
import numpy.typing as npt
import tempfile
from . import PoissonDisk
import matplotlib.colors
from . import PerlinNoise
import matplotlib.pyplot as plt
from typing import Tuple

import time
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
    area: npt.NDArray[np.float64],
    size: int,
    region_value: int,
    object_value: int,
) -> Tuple[npt.NDArray[np.float64], list]:
    # Generate points and fill the area with objects using Poisson
    # print(
    #     "beggining bridson sampling with area ",
    #     area.shape,
    #     " and poisson size of ",
    #     size,
    # )

    points = PoissonDisk.Bridson_sampling(
        width=area.shape[0] - 1, height=area.shape[1] - 1, radius=size, k=15
    )  # 30
    # print("sampling done now we need to extract what points can be used")
    new_points = []
    for i, p in enumerate(points):
        x_int = int(p[0])
        y_int = int(p[1])
        if area[y_int][x_int] == region_value:
            new_points.append(p)

    return area, new_points


class ObjectPrim:
    def __init__(
        self,
        scale,
        scale_delta,
        y_rot,
        u_id,
        usd_path,
        class_name,
        poisson_size,
    ) -> None:
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


class TerrainPrim:
    def __init__(self, mesh_path, mat_path, scale=0.001) -> None:
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
        self._o = "[WorldHandler]"

    def _log(self, msg):
        print(f"[{time.ctime()}]{self._o}{msg}")


    def read_configs(self):

        self._read_objects()
        res = self._read_world()
        return res
    def _read_objects(self):
        with open(self._object_path, "r+") as infile:
            data = json.load(infile)
            # print(data)
            for key in data:
                scale = data[key]["object_scale"]
                scale_delta = data[key]["object_scale_delta"]
                y_rot = data[key]["allow_y_rot"]
                u_id = key
                usd_path = data[key]["usd_path"]
                class_name = data[key]["class_name"]
                poisson_size = data[key]["poisson_size"]
                ignore_ground_normals = False
                if "ignore_ground_normals" in data[key]:
                    ignore_ground_normals = True
                tmp = ObjectPrim(
                    scale,
                    scale_delta,
                    y_rot,
                    u_id,
                    usd_path,
                    class_name,
                    poisson_size,
                )
                tmp.ignore_ground_normals = ignore_ground_normals
                # self.objects.append(tmp)
                self.objects_dict[u_id] = tmp

    def pad_to_square(self, arr):
        """
        Pad a 2D numpy array with zeros to make it square.

        :param arr: 2D numpy array
        :return: Square 2D numpy array
        """
        rows, cols = arr.shape
        if rows == cols:
            return arr  # Already square, no padding needed

        # Determine the size to pad to (the larger of rows or cols)
        size = max(rows, cols)

        # Calculate padding sizes
        pad_rows = size - rows
        pad_cols = size - cols

        # Pad the array
        return np.pad(arr, ((0, pad_rows), (0, pad_cols)), mode="constant")

    def pad_to_multiple_of_eight(self, arr):
        """
        Pad a 2D numpy array so that its dimensions are multiples of 8.

        :param arr: 2D numpy array
        :return: Padded 2D numpy array
        """
        height, width = arr.shape
        pad_height = (-height) % 8  # Padding needed to make height a multiple of 8
        pad_width = (-width) % 8  # Padding needed to make width a multiple of 8

        # Calculate padding for top/bottom and left/right
        pad_top = pad_height // 2
        pad_bottom = pad_height - pad_top
        pad_left = pad_width // 2
        pad_right = pad_width - pad_left

        # Apply padding and return the result
        return np.pad(
            arr,
            ((pad_top, pad_bottom), (pad_left, pad_right)),
            mode="constant",
            constant_values=0,
        )
    def _read_world(self):
        self.objects_to_spawn = {}
        with open(self._world_path, "r+") as infile:
            data = json.load(infile)

        if data is None:
            return None, None, None, None

        n = data["size"]
        preload_hm = preload_mask = None
        ignore_gen = False

        if "heightmap" in data:
            preload_hm = self.pad_to_square(np.load(data["heightmap"]))
            ignore_gen = True

        if "mask" in data:
            preload_mask = np.load(data["mask"])
            preload_mask = self.pad_to_square(preload_mask)
            preload_mask = self.pad_to_multiple_of_eight(preload_mask)
            preload_mask = preload_mask[:4000, :4000] if preload_hm is None else preload_mask
            n = len(preload_mask)
            ignore_gen = True

        arr = np.zeros((n, n)) if not ignore_gen else preload_mask
        terrain_info, objs_per_region = {}, {}

        for region_id, region_data in data["regions"].items():
            terrain_info[str(region_id)] = TerrainPrim("", region_data["material_path"], region_data["material_scale"])
            objs_per_region[str(region_id)] = [self.objects_dict[str(obj_uid)] for obj_uid in region_data["objects"]]

            if not ignore_gen:
                arr = self._generate_terrain(arr, region_id, region_data, n)

            for zone_id, zone_data in region_data["zones"].items():
                terrain_info[str(zone_id)] = TerrainPrim("", zone_data["material_path"], zone_data["material_scale"])
                objs_per_region[str(zone_id)] = [self.objects_dict[obj_uid] for obj_uid in zone_data["objects"]]

                if not ignore_gen:
                    arr = self._generate_terrain(arr, zone_id, zone_data, n)

        self._log("Performing poisson disc sampling for object placement.")
        self._log(f"The size of the array is, {arr.shape}" )
        for key, obs in objs_per_region.items():
            if len(obs) > 0 and np.isin(int(key), arr):
                self._log(f"Sampling for {key}.")
                for obj in obs:
                    area, coords = fill_area(arr, obj.poisson_size * 10 , int(key), 999)
                    # area, coords = fill_area(arr, obj.poisson_size / self._WORLD_TO_POISSON_SCALE, int(key), 999)
                    self.objects_to_spawn.setdefault(obj.unique_id, []).extend(coords)

        return arr, n, terrain_info, preload_hm

    def _generate_terrain(self, arr, region_id, region_data, n):
        new_arr = PerlinNoise.generate_region2(
            seed=int(region_id),
            shape=(n, n),
            threshold=float(region_data["threshold"]),
            show_plot=False,
            region_value=int(region_id),
        )
        return append_to_area(arr, new_arr, int(region_id))


def generate_world_from_file(world_path, object_path):
    world = WorldHandler(world_path, object_path)

    print(f"[{time.ctime()}][AreaMaskGenerator] Reading objects and world from file.")

    # world._read_objects()
    # res = world._read_world()
    res= world.read_configs()

    mesh_scale = 1  # 10

    # terrain_mesh_paths = []
    if res:
        region_map, map_size, terrain_info, preload_hm = res
        m_path = (
            tempfile.gettempdir()
        )  #'C:/Users/jonem/Documents/Kit/apps/Isaac-Sim/exts/IsaacSyntheticPerception/com/SyntheticPerception/app/PCG'
        meshGen = MeshGen(
            map_size, mesh_scale, region_map, m_path, heightmap=preload_hm
        )

        meshGen.generate_terrain_mesh()

        print(f"[{time.ctime()}][AreaMaskGenerator] Saving mesh paths.")
        regs = list(np.unique(region_map))
        for key in terrain_info:
            # print(key)
            if float(key) in regs:
                terrain_info[str(key)].mesh_path = meshGen.final_mesh_paths_dict[
                    int(key)
                ]
        print(
            f"[{time.time()}][AreaMaskGenerator] All terrain infos updated. Passing data back to main sample to genereate objects and load the terrain in."
        )

        return (
            world.objects_to_spawn,
            world.objects_dict,
            terrain_info,
            meshGen,
        )  # ._points2#_noise_map_xy
    return world.objects_to_spawn, world.objects_dict, None, None
