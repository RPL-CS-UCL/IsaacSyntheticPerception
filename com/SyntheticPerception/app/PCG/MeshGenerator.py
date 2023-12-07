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

# 0.001
# enable project uvw coordinates


class MeshGen:
    def __init__(self, map_size, map_scale, regions_map, save_path, heightmap=None) -> None:
        pass
        self._size = map_size
        self._scale = map_scale
        self._scale = 1
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


    async def convert(self, in_file, out_file, load_materials=False):
        # This import causes conflicts when global

        def progress_callback(progress, total_steps):
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        converter_context.ignore_materials = not load_materials
        converter_context.ignore_animation = True
        converter_context.ignore_cameras = True
        converter_context.single_mesh = True
        converter_context.smooth_normals = False #True
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

        print(f'{self._o} Converting .obj files to .usd')
        for file_path in self._files_to_clean:
            new_path = file_path.replace('.obj', '.usd')
            self.final_mesh_paths.append(new_path)
            print(f'{self._o} Trying to convert {file_path} to {new_path}')

            status = asyncio.get_event_loop().run_until_complete(
                self.convert(file_path, new_path)
            )


    def generate_terrain_mesh(self):
        self._create_noise_map()
        self._compute_base_mesh()
        self._save_meshes()
        self.cnv()

    def clean_up_files(self):
        def file_exists(file_path):
            return os.path.exists(file_path)

        for file_path in self._files_to_clean:
            if file_exists(file_path):
                os.remove(file_path)

    def _save_meshes(self):

        print(f'{self._o} Saving meshes to folder {self._save_path}.')
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
    def add_grain_noise(self,heightmap, intensity=0.1):
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
        print(f'{self._o} Creating Noise Map for terrain heights.')
        # self._noise_map_xy = generate_fractal_noise_2d(
        #     self._map_shape, (8, 8), 5
        # )
        if self.preload_hm is not None:
            self._noise_map_xy = self.preload_hm
        else:
            self._noise_map_xy = generate_perlin_noise_2d(
                self._map_shape, (8, 8) 
            )
        self._noise_map_xy = self.add_grain_noise(self._noise_map_xy)
        x = np.linspace(
            0,
            self._size * self._scale,
            self._size * self._scale,
            dtype=np.int32,
        )
        y = np.linspace(
            0,
            self._size * self._scale,
            self._size * self._scale,
            dtype=np.int32,
        )

        self._noise_map_xy *= scale 
        noise_flat = self._noise_map_xy.flatten()
        X, Y = np.meshgrid(x, y)

        self._points = np.column_stack(
                (X.ravel(),Y.ravel(), noise_flat) # was abs::with
        )

    def _compute_base_mesh(self):

        subdivisions = (self._size * self._scale) - 1
        materials = list(np.unique(self._regions_map))
        print(f"There are {len(materials)},   {materials}")

        self.meshes_dict = {}
        for key in materials:
            self.meshes_dict[int(key)] = o3d.geometry.TriangleMesh()
        print(f'{self._o} Computing the base mesh.')
        self._faces = []
        
        for j in range(subdivisions):
            for i in range(subdivisions):
                index = j * (subdivisions + 1) + i
                face1 = [index, index + 1, index + subdivisions + 2]
                face2 = [
                    index,
                    index + subdivisions + 2,
                    index + subdivisions + 1,
                ]
                self._faces.append(face1)
                self._faces.append(face2)
                res_ind = int(self._regions_map[j,i])
                self.meshes_dict[res_ind].triangles.append(face1)
                self.meshes_dict[res_ind].triangles.append(face2)

        self._mesh = o3d.geometry.TriangleMesh()
        self._mesh.vertices = o3d.utility.Vector3dVector(self._points)

        self._mesh.triangles = o3d.utility.Vector3iVector(
            np.array(self._faces)
        )
        print(np.asarray(self._mesh.triangles)[0])
        print("vert pos")
        print(np.asarray(self._mesh.vertices)[int(np.asarray(self._mesh.triangles)[0][0])])

        self._mesh.paint_uniform_color([1, 0.706, 0])

        self._mesh.compute_vertex_normals()

        self._mesh = self._mesh.compute_vertex_normals()
        self._mesh = self._mesh.remove_unreferenced_vertices()
        self._mesh = self._mesh.remove_duplicated_vertices()
        self.normals = self._mesh.triangle_normals

        l = self._scale * self._size
        for i in range(len(self._mesh.vertices)):

            ind = np.unravel_index(i, (l, l))
            self._points2[ind] = self._mesh.vertices[i][2]
        self.np_tris =np.asarray(self._mesh.triangles) 
        self.np_verts =np.asarray(self._mesh.vertices) 


        N = len(materials)
        HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
        RGB_tuples = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))

        for i, key in enumerate(list(self.meshes_dict.keys())):
            self.meshes_dict[key].vertices = self._mesh.vertices

            self.meshes_dict[key].vertex_normals = self._mesh.vertex_normals
            self.meshes_dict[key] = self.meshes_dict[
                key
            ].remove_unreferenced_vertices()
            self.meshes_dict[key].paint_uniform_color(RGB_tuples[i])
            self.meshes_dict[key] = self.meshes_dict[
                key
            ].compute_vertex_normals()
            self.meshes_dict[key] = self.meshes_dict[
                key
            ].compute_triangle_normals()
            print(np.array(self.meshes_dict[key].triangle_normals))

