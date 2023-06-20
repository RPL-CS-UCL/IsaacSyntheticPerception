"""
This module handles area and point generation.
"""
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

    area[mask_indices] = area_value  # area_value

    return area


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
