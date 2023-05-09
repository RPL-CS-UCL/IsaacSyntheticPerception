"""
This module handles area and point generation.
"""
import numpy as np
import numpy.typing as npt
from PerlinNoise import *


def append_inside_area(
    area: npt.NDArray[np.float64],
    area_to_add: npt.NDArray[np.float64],
    area_value: float,
) -> npt.NDArray[np.float64]:
    """
    Function returns a new mask that is only within the first mask

    """
    mask_indices = np.where((area_to_add >= area_value) & (area != 0))

    area[mask_indices] = 3  # area_value

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


def fill_area(area: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    # Generate points and fill the area with objects using Poisson

    return area


if __name__ == "__main__":
    reg1 = generate_region(threshold=0.5, show_plot=False)
    # print(reg1)
    reg2 = generate_region(threshold=0.5, show_plot=False, region_value=2)
    # area = append_to_area(np.array(reg1), np.array(reg2), 1.0)
    area = append_inside_area(np.array(reg1), np.array(reg2), 2.0)
    print(np.unique(area))
    plt.imshow(area)
    plt.colorbar()
    plt.show()
