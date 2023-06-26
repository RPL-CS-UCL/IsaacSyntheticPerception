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
        self.objects = []
        self._object_path = object_path
        self._world_path = world_path

    def _read_objects(self):
        with open(self._object_path, 'r+') as infile:
            data = json.load(infile)
            for key in data:
                scale = data[key]["object_scale"]
                scale_delta =data[key]["object_scale_delta"]
                y_rot = data[key]["allow_y_rot"]
                u_id = key
                usd_path = data[key]["usd_path"]
                class_name = data[key]["class_name"]
                poisson_size = data[key]["poisson_size"]
                tmp = ObjectPrim(scale, scale_delta, y_rot, u_id, usd_path, class_name, poisson_size)
                self.objects.append(tmp)

        print("Loaded the following objects")
        for i in self.objects:
            print(i)


def generate_world_from_file(world_path, object_path):
    world = WorldHandler(world_path, object_path)
    world._read_objects()


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
