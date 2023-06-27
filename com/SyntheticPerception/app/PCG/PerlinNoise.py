import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
from perlin_numpy import (
    generate_perlin_noise_2d, generate_fractal_noise_2d
)


def generate_region(threshold=0.5, shape=(256,256), region_value=1, show_plot=False) -> npt.NDArray[np.float64]:

    # np.random.seed(0)
    data = generate_perlin_noise_2d(shape, (8, 8))
    data = (data-np.min(data))/(np.max(data)-np.min(data))
    data[data < threshold] = 0
    data[data >= threshold] = region_value

    if show_plot:
        plt.imshow(data, cmap='gray', interpolation='lanczos')
        plt.colorbar()
        plt.show()
    return data

def generate_region2(seed = 1, threshold=0.5, shape=(256,256), region_value=1, show_plot=False) -> npt.NDArray[np.float64]:

    np.random.seed(seed)
    data = generate_perlin_noise_2d(shape, (8, 8))
    data = (data-np.min(data))/(np.max(data)-np.min(data))
    data[data < threshold] = 0
    data[data >= threshold] = region_value

    if show_plot:
        plt.imshow(data, cmap='gray', interpolation='lanczos')
        plt.colorbar()
        plt.show()
    return data
if __name__ == "__main__":

    np.random.seed(0)
    generate_region(show_plot=True)

# noise = generate_perlin_noise_2d((256, 256), (8, 8))
# plt.imshow(noise, cmap='gray', interpolation='lanczos')
# plt.colorbar()
#
# np.random.seed(0)
# noise = generate_fractal_noise_2d((256, 256), (8, 8), 5)
# plt.figure()
# plt.imshow(noise, cmap='gray', interpolation='lanczos')
# plt.colorbar()
# plt.show()
#
#
