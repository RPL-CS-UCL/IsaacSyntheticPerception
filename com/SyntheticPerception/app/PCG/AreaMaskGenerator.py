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


def test_func():
    print("running now")
    n = 256
    reg1 = PerlinNoise.generate_region(shape=(n,n), threshold=0.5, show_plot=False)
    # print(reg1)
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
