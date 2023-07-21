
import open3d as o3d
import numpy as np
import os
from perlin_numpy import generate_perlin_noise_2d, generate_fractal_noise_2d
from sklearn.preprocessing import normalize
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt
import cv2
import colorsys
import json
import asyncio

import numpy.typing as npt
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
        pass


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

    def _read_objects(self):
        with open(self._object_path, 'r+') as infile:
            data = json.load(infile)
            # print(data)
            for key in data:
                scale = data[key]['object_scale']
                scale_delta = data[key]['object_scale_delta']
                y_rot = data[key]['allow_y_rot']
                u_id = key
                usd_path = data[key]['usd_path']
                class_name = data[key]['class_name']
                poisson_size = data[key]['poisson_size']
                tmp = ObjectPrim(
                    scale,
                    scale_delta,
                    y_rot,
                    u_id,
                    usd_path,
                    class_name,
                    poisson_size,
                )
                # self.objects.append(tmp)
                self.objects_dict[u_id] = tmp

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
            n = data['size']
            arr = np.zeros((n, n))
            total_arr = np.zeros((n, n))
            regions = data['regions']
            terrain_info = {}
            # print( " == ", np.unique(total_arr))
            for region_id in regions:
                region_id = str(region_id)

                terrain_info[region_id] = TerrainPrim(
                    '',
                    regions[region_id]['material_path'],
                    regions[region_id]['material_scale'],
                )
                # print("terrrain info key type ", type(region_id))
                new_arr = PerlinNoise.generate_region2(
                    seed=int(region_id),
                    shape=(n, n),
                    threshold=float(regions[region_id]['threshold']),
                    show_plot=False,
                    region_value=int(region_id),
                )

                arr = append_to_area(arr, new_arr, int(region_id))
                total_arr = arr
                # handle objects in the zone
                objs = regions[region_id]['objects']
                objs_per_region[region_id] = []
                if len(objs) > 0:
                    for obj_uid in objs:
                        # get corresponding object from objects
                        object_prim = self.objects_dict[str(obj_uid)]
                        objs_per_region[region_id].append(object_prim)

                # now we need to deal with sub zones
                zones = regions[region_id]['zones']
                for zone_id in zones:

                    terrain_info[str(zone_id)] = TerrainPrim(
                        '',
                        zones[zone_id]['material_path'],
                        zones[zone_id]['material_scale'],
                    )
                    new_arr = PerlinNoise.generate_region2(
                        seed=int(zone_id),
                        shape=(n, n),
                        threshold=float(zones[zone_id]['threshold']),
                        show_plot=False,
                        region_value=int(zone_id),
                    )

                    zone_to_save = append_inside_area(
                        arr, new_arr, int(zone_id)
                    )

                    # print("zone == ", zone_id, "  ", zone_id)
                    total_arr = zone_to_save
                    objs = zones[zone_id]['objects']

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
                        area, coords = fill_area(
                            total_arr,
                            obj.poisson_size / self._WORLD_TO_POISSON_SCALE,
                            int(key),
                            999,
                        )
                        self.objects_to_spawn[obj.unique_id] = coords
            return total_arr, n, terrain_info
