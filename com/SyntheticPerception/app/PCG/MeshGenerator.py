import open3d as o3d
import numpy as np
import os
from perlin_numpy import generate_perlin_noise_2d, generate_fractal_noise_2d
from sklearn.preprocessing import normalize
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt
import cv2
import colorsys
import asyncio
import omni.kit.asset_converter
import carb
import time
import concurrent.futures
import sys
import glob
sys.path.append('/home/jon/Documents/IsaacSyntheticPerception/com/SyntheticPerception/app/build/')
import mesh_module
from PIL import Image
mesh_module.test_open3d()


class MeshGen:
    def __init__(self, map_size, map_scale, regions_map, save_path, heightmap=None) -> None:
        pass
        self._size = map_size
        self._scale = map_scale
        # self._scale = 1
        # REMOVE THIS NEXT LINE``
        l = self._size * self._scale
        self._map_shape = (self._size * self._scale, self._size * self._scale)
        self._points = np.zeros(shape=(l * l, 3))
        self._points2 = np.zeros(shape=(l , l))
        self._noise_map_xy = None
        self._faces = []
        self._mesh = None
        self._regions_map = cv2.resize(
            regions_map,
            dsize=(self._size * self._scale, self._size * self._scale),
            interpolation=cv2.INTER_NEAREST,
        )
        self._save_path = save_path
        self.meshes = []
        self._o = '[MeshGenerator] '
        self._files_to_clean = []
        self.final_mesh_paths = []
        self.final_mesh_paths_dict = {}
        self.region_to_path = {}

        self.preload_hm = heightmap

    def _log(self,msg):
        print(f"[{time.ctime()}]{self._o}{msg}")


    async def convert(self, in_file, out_file, load_materials=False):
        def progress_callback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        converter_context.ignore_materials = not load_materials
        converter_context.ignore_animation = True
        converter_context.ignore_cameras = True
        converter_context.single_mesh = True
        converter_context.smooth_normals = True#False #True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        converter_context.use_meter_as_world_unit = True
        # converter_context.create_world_as_default_root_prim = False
        instance = omni.kit.asset_converter.get_instance()
        task = instance.create_converter_task(
            in_file, out_file, progress_callback, converter_context
        )
        success = True
        while True:
            success = await task.wait_until_finished()
            if not success:
                await asyncio.sleep(0.1)
            else:
                break
        return success

    def cnv(self):
        prefix = "mesh_"
        pattern = os.path.join(self._save_path, prefix + '*')

        # Use glob to find files matching the pattern
        files = glob.glob(pattern)
        for file_path in files:

            new_path = file_path.replace('.obj', '.usd')
            status = asyncio.get_event_loop().run_until_complete(
                self.convert(file_path, new_path)
            )



    def generate_terrain_mesh(self):
        self._log(f"Generating terrain mesh.")

        self._log(f"Creating noise map.")
        self._create_noise_map()

        self._log(f"Computing base mesh and seperating to different regions and zone meshes.")
        self._compute_base_mesh()

        # self._log(f"Saving meshes to temp.")
        # self._save_meshes()

        self._log(f"Converting meshes to usd.")
        self.cnv()

    def clean_up_files(self):
        def file_exists(file_path):
            return os.path.exists(file_path)

        for file_path in self._files_to_clean:
            if file_exists(file_path):
                os.remove(file_path)

    def _save_meshes(self):

        self._log(f'Saving meshes to folder {self._save_path}.')
        for i, key in enumerate(list(self.meshes_dict.keys())):
            self._files_to_clean.append(f'{self._save_path}/mesh_{i}.obj')
            self._files_to_clean.append(f'{self._save_path}/mesh_{i}.usd')
            self.final_mesh_paths_dict[key] = f'{self._save_path}/mesh_{i}.obj'
            o3d.io.write_triangle_mesh(
                filename=f'{self._save_path}/mesh_{i}.obj',
                mesh=self.meshes_dict[int(key)],
                compressed=False,
                write_vertex_normals=True,
                # write_vertex_colors=True,
                # write_triangle_uvs=True,
                print_progress=False,
            )
    def add_grain_noise(self,heightmap, intensity=0.02):
        """
        Add grain noise to a 2D numpy heightmap.

        :param heightmap: 2D numpy array representing the heightmap.
        :param intensity: Intensity of the noise; default is 0.1.
        :return: Heightmap with added noise.
        """
        noise = np.random.normal(0, intensity, heightmap.shape)
        noisy_heightmap = heightmap + noise

        # Optionally, clip the values to maintain the original data range
        # For example, for a heightmap with values ranging from 0 to 1:
        # noisy_heightmap = np.clip(noisy_heightmap, 0, 1)

        return noisy_heightmap

    def _create_noise_map(self):

        scale = 1#250.0
        if self.preload_hm is not None:
            self._noise_map_xy = self.preload_hm
            self._log(f"Using a preloaded noisemap")
        else:

            self._log(f'Creating Noise Map for terrain heights.')
            self._map_shape = (self._size * self._scale, self._size * self._scale)
            self._noise_map_xy = generate_perlin_noise_2d(
                self._map_shape, (8, 8)
            )

            # self._noise_map_xy = generate_fractal_noise_2d(
            #     self._map_shape, (8, 8), 5
            # )
        self._noise_map_xy = self.add_grain_noise(self._noise_map_xy)
        x = np.linspace(
            0,
            self._size,# * self._scale,
            self._size * self._scale,
            dtype=np.float32,#int32,
        )
        y = np.linspace(
            0,
            self._size,# * self._scale,
            self._size * self._scale,
            dtype=np.float32,#np.int32,
        )
        self._log(f"Map of size {x.shape}, {y.shape} created.")

        scale = 3
        self._noise_map_xy *= scale 
        noise_flat = self._noise_map_xy.flatten()
        X, Y = np.meshgrid(x, y)

        self._points = np.column_stack(
                (X.ravel(),Y.ravel(), noise_flat) # was abs::with
        )
    def assign_face(self, i, face, subdivisions):
        j = i // 2 // subdivisions
        res_ind = int(self._regions_map[j, i // 2 % subdivisions])
        self.meshes_dict[res_ind].triangles.append(face)

    def _compute_base_mesh(self):
        self._log(f"The shape of points is {self._points.shape}")
        materials = list(np.unique(self._regions_map))
        self.meshes_dict = {}
        for key in materials:
            self.meshes_dict[int(key)] = o3d.geometry.TriangleMesh()

            self.final_mesh_paths_dict[int(key)] = f'{self._save_path}/mesh_{key}.obj'

            self._files_to_clean.append(f'{self._save_path}/mesh_{key}.obj')
            self._files_to_clean.append(f'{self._save_path}/mesh_{key}.usd')
            # print("num mats")
        self._faces = []

        # load all the terrain textures
        # img = Image.open("/home/jon/Desktop/bricks.jpg")
        # new_width = int(img.width / 10)
        # new_height = int(img.height / 10)
        # img= img.resize((new_width, new_height), Image.ANTIALIAS)
        # img_gray = img.convert('L')
        # img_array = np.array(img_gray)
        img_array = np.array([[2,2]])

        self._points2 = mesh_module.build_meshes(self._size, self._scale, self._regions_map, self._points,self._save_path, img_array)
        self._points2 = np.array(self._points2)

        self.normals = 0
        self._log(f'Computing the base mesh (assigning tris and verts).')
        return
