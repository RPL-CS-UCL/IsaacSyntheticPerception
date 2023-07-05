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
    def __init__(self, map_size, map_scale, regions_map, save_path) -> None:
        pass
        self._size = map_size
        self._scale = map_scale
        l = self._size * self._scale
        self._map_shape = (self._size * self._scale, self._size * self._scale)
        self._points = np.zeros(shape=(l * l, 3))
        self._noise_map_xy = None
        self._faces = []
        self._mesh = None
        print('materials')
        print(np.unique(regions_map))
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

    async def _convert_asset_to_usd(self, input_obj: str, output_usd: str):
        def progress_callback(progress, total_steps):
            print(
                f'{self._o} current progress: ', progress, ' / ', total_steps
            )
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        converter_context.ignore_material = True
        converter_context.ignore_animation = True
        converter_context.ignore_cameras = True
        converter_context.single_mesh = True
        converter_context.smooth_normals = True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        # converter_context.use_meter_as_world_unit = True
        # converter_context.create_world_as_default_root_prim = False
        instance = omni.kit.asset_converter.get_instance()
        task = instance.create_converter_task(
            input_obj, output_usd, progress_callback, converter_context
        )
        success = await task.wait_until_finished()
        if not success:
            carb.log_error(task.get_status(), task.get_detailed_error())
        print(f'{self._o} Converting done')

    def _convert_asset_to_usd_nonasync(self, input_obj: str, output_usd: str):
        def progress_callback(progress, total_steps):
            print(
                f'{self._o} current progress: ', progress, ' / ', total_steps
            )
            pass

        converter_context = omni.kit.asset_converter.AssetConverterContext()
        # setup converter and flags
        # converter_context.ignore_material = False
        # converter_context.ignore_animation = False
        # converter_context.ignore_cameras = True
        # converter_context.single_mesh = True
        # converter_context.smooth_normals = True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        # converter_context.use_meter_as_world_unit = True
        # converter_context.create_world_as_default_root_prim = False
        instance = omni.kit.asset_converter.get_instance()
        task = instance.create_converter_task(
            input_obj, output_usd, progress_callback, converter_context
        )
        # success = await task.wait_until_finished()
        # if not success:
        #     carb.log_error(task.get_status(), task.get_detailed_error())
        print(f'{self._o} Converting done')

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
        converter_context.smooth_normals = True
        # converter_context.preview_surface = False
        # converter_context.support_point_instancer = False
        # converter_context.embed_mdl_in_usd = False
        # converter_context.use_meter_as_world_unit = True
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
        tasks = []
        for file_path in self._files_to_clean:
            new_path = file_path.replace('.obj', '.usd')
            self.final_mesh_paths.append(new_path)
            print(f'{self._o} Trying to convert {file_path} to {new_path}')

            status = asyncio.get_event_loop().run_until_complete(
                self.convert(file_path, new_path)
            )

    async def _convert_all_async(self):

        print(f'{self._o} Converting .obj files to .usd')
        tasks = []
        for file_path in self._files_to_clean:
            new_path = file_path.replace('.obj', '.usd')
            self.final_mesh_paths.append(new_path)
            print(f'{self._o} Trying to convert {file_path} to {new_path}')
            tasks.append(
                asyncio.create_task(
                    self._convert_asset_to_usd(file_path, new_path)
                )
            )
            # wait for all tasks to complete
        done, pending = await asyncio.wait(tasks)
        print('done')

    def _convert_all(self):

        print(f'{self._o} Converting .obj files to .usd')
        for file_path in self._files_to_clean:
            new_path = file_path.replace('.obj', '.usd')
            self.final_mesh_paths.append(new_path)
            print(f'{self._o} Trying to convert {file_path} to {new_path}')

            x = asyncio.get_event_loop().run_until_complete(
                self.convert(file_path, new_path)
            )
            # self._convert_asset_to_usd_nonasync(file_path, new_path)
            # asyncio.ensure_future(
            #     self.convert(file_path, new_path)
            # )

    def generate_terrain_mesh(self):
        self._create_noise_map()
        self._compute_base_mesh()
        # self._create_mesh_by_region()
        self._save_meshes()
        # self._convert_all()
        # asyncio.run(self._convert_all_async())
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
            # print('final mesh paths dict key type ', type(key))
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
        # for i, m in enumerate(self.meshes):
        #     self._files_to_clean.append(f'{self._save_path}/mesh_{i}.obj')
        #     o3d.io.write_triangle_mesh(
        #         filename=f'{self._save_path}/mesh_{i}.obj',
        #         mesh=m,
        #         compressed=False,
        #         write_vertex_normals=True,
        #         # write_vertex_colors=True,
        #         # write_triangle_uvs=True,
        #         print_progress=False,
        #     )

    def _create_noise_map(self):
        print(f'{self._o} Creating Noise Map for terrain heights.')

        self._noise_map_xy = generate_fractal_noise_2d(
            self._map_shape, (8, 8), 3
        )

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
        noise_flat = self._noise_map_xy.flatten()
        X, Y = np.meshgrid(x, y)

        self._points = np.column_stack(
            (X.ravel(), abs(noise_flat) * 150, Y.ravel())
        )

    def _compute_base_mesh(self):

        subdivisions = (self._size * self._scale) - 1
        total_size_needed = subdivisions*subdivisions*2
        materials = list(np.unique(self._regions_map))

        self.meshes_dict = {}
        faces_holder = {}
        faces_counter = {}
        for key in materials:
            self.meshes_dict[int(key)] = o3d.geometry.TriangleMesh()
            faces_holder[int(key)] =np.zeros((total_size_needed,3)) 
            faces_counter[int(key)] = 0
        print(f'{self._o} Computing the base mesh.')
        self._faces = []
        # faces_holder = [np.zeros((total_size_needed,3)) for i in range(len(materials))]
        # faces_counter = [0 for i in range(len(materials))]
        # print("debug")
        # print(len(faces_holder))
        # print(len(faces_counter))
        # print(len(materials))

        mesh_face_holder = np.zeros((total_size_needed,3))
        mesh_face_counter = 0
        for j in range(subdivisions):
            for i in range(subdivisions):
                index = j * (subdivisions + 1) + i
                face1 = [index, index + 1, index + subdivisions + 2]
                face2 = [
                    index,
                    index + subdivisions + 2,
                    index + subdivisions + 1,
                ]
                # self._faces.append(face1)
                # self._faces.append(face2)
                mesh_face_holder[mesh_face_counter] = face1
                mesh_face_holder[mesh_face_counter+1] = face2
                mesh_face_counter += 2

                # print(faces_ctotal_size_needed)
                res_ind = int(self._regions_map[j,i])
                # print(faces_counter)
                # print(res_ind)
                # print(self._regions_map[j,i])
                faces_holder[res_ind][faces_counter[res_ind]] = face1
                faces_holder[res_ind][faces_counter[res_ind]+1] = face2
                faces_counter[res_ind] += 2
                # self.meshes_dict[res_ind].triangles.append(face1)
                # self.meshes_dict[res_ind].triangles.append(face2)
        
        #clean faces
        self._faces= mesh_face_holder[:mesh_face_counter]
        for key in self.meshes_dict:
            k = int(key)
            faces_holder[k] = mesh_face_holder[k][:mesh_face_counter]


        self._mesh = o3d.geometry.TriangleMesh()
        self._mesh.vertices = o3d.utility.Vector3dVector(self._points)
        self._mesh.triangles = o3d.utility.Vector3iVector(
            np.array(self._faces)
        )
        self._mesh.paint_uniform_color([1, 0.706, 0])

        self._mesh.compute_vertex_normals()
        self._mesh = self._mesh.filter_smooth_laplacian(
            number_of_iterations=10
        )

        self._mesh = self._mesh.compute_vertex_normals()

        N = len(materials)
        HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
        RGB_tuples = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))

        for i, key in enumerate(list(self.meshes_dict.keys())):
            self.meshes_dict[key].vertices = self._mesh.vertices

            self.meshes_dict[key].vertex_normals = self._mesh.vertex_normals
            # v_uv = np.random.rand(len(mesh.triangles) * 3, 2)
            # mesh2.triangle_uvs = v_uv
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

    def _create_mesh_by_region(self):

        print(f'{self._o} Creating each submesh by region.')
        materials = list(np.unique(self._regions_map))
        # for i,val in enumerate(materials):
        #     if val > 100:
        #         materials.remove(val)
        # print(materials)
        self.meshes = [
            o3d.geometry.TriangleMesh() for i in range(len(materials))
        ]
        self.meshes_dict = {}
        for key in materials:
            self.meshes_dict[int(key)] = o3d.geometry.TriangleMesh()

        index_to_try = 0
        other_id = 0
        while index_to_try < len(self._mesh.triangles):
            face = self._mesh.triangles[index_to_try]
            face2 = self._mesh.triangles[index_to_try + 1]
            l = self._scale * self._size
            ind = np.unravel_index(other_id, (l, l))
            # print(ind)
            res_ind = self._regions_map[ind]
            # self.meshes[int(res_ind)].triangles.append(face)
            # self.meshes[int(res_ind)].triangles.append(face2)
            self.meshes_dict[int(res_ind)].triangles.append(face)
            self.meshes_dict[int(res_ind)].triangles.append(face2)
            index_to_try += 2
            other_id += 1

        N = len(materials)
        HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
        RGB_tuples = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))

        for i, key in enumerate(list(self.meshes_dict.keys())):
            self.meshes_dict[key].vertices = self._mesh.vertices

            self.meshes_dict[key].vertex_normals = self._mesh.vertex_normals
            # v_uv = np.random.rand(len(mesh.triangles) * 3, 2)
            # mesh2.triangle_uvs = v_uv
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
        # for i, mesh2 in enumerate(self.meshes):
        #     mesh2.vertices = self._mesh.vertices
        #
        #     mesh2.vertex_normals = self._mesh.vertex_normals
        #     # v_uv = np.random.rand(len(mesh.triangles) * 3, 2)
        #     # mesh2.triangle_uvs = v_uv
        #     mesh2 = mesh2.remove_unreferenced_vertices()
        #     mesh2.paint_uniform_color(RGB_tuples[i])
        #     mesh2 = mesh2.compute_vertex_normals()
        #     mesh2 = mesh2.compute_triangle_normals()


# print('starting class')
#
# print('generating initial mesh noise')
# points = []
# all_verts = []
#
# shape = (256, 256)
# threshold = 0.5
# region_value = 1
# # Convert to pymeshlab mesh
# l = shape[0] * 10   # 2560
# data = generate_perlin_noise_2d(shape, (8, 8))
# data = (data - np.min(data)) / (np.max(data) - np.min(data))
# data[data < threshold] = 0
# data[data >= threshold] = region_value
# mGen = MeshGen(
#     256,
#     10,
#     data,
#     'C:/Users/jonem/Documents/Kit/apps/Isaac-Sim/exts/IsaacSyntheticPerception/com/SyntheticPerception/app/PCG',
# )
# mGen.generate_terrain_mesh()
# asdasd
#
# # points_np = np.zeros(shape=(l * l, 3))
#
# # def vec_fill(x):
# #     x,y = x
# #     noise_val = noise1([x / xpix, y / ypix])
# #     return [x, y, abs(noise_val) * 200]
# # fill_array = np.vectorize(vec_fill)
# # array = fill_array([x,y] for y in range(l) for x in range(l))
# # print(array)
#
# points_np = np.zeros(shape=(l * l, 3))
# shape2 = (l, l)
# res = generate_perlin_noise_2d(shape2, (8, 8))
#
#
#
#
# x = np.linspace(0, l, l, dtype=np.int32)
# print(x.shape)
#
# y = np.linspace(0, l, l, dtype=np.int32)
# print(y.shape)
# res2 = res.flatten()
# print(res2.shape)
# X, Y = np.meshgrid(x, y)
#
# # Stack the X and Y grids into a 2-column array
# points_np= np.column_stack((X.ravel(), Y.ravel(),res2*500))
# # result = np.stack((x, y, res.flatten()), axis=1)
#
# # print(result.shape)
# # vectorize_calculate_value = np.vectorize(calculate_value)
# # points_np = vectorize_calculate_value(res.flatten(), np.zeros((len(res.flatten()))))
# print('initial gen done')
# a = np.array(pic)
# arr = abs(a)
# # faces = []
# # for i in range(1, len(all_verts) - 1):
# #     face = [i - 1, i, i + 1]  # Create a face using three consecutive vertices
# #     faces.append(face)
# # print('here 2 222 l, faces', faces)
# print('generating triangle face index')
# faces = []
# subdivisions = l - 1
# for j in range(subdivisions):
#     for i in range(subdivisions):
#         index = j * (subdivisions + 1) + i
#         face1 = [index, index + 1, index + subdivisions + 2]
#         face2 = [index, index + subdivisions + 2, index + subdivisions + 1]
#         faces.append(face1)
#         faces.append(face2)
# # Step 3: Create an array representing the mesh vertices
# vertices = points_np  # np.array(points)  # Vertex 4
# print(vertices.shape)
# print(type(vertices))
#
# # Step 4: Create a TriangleMesh object and assign the vertices
# self._mesh = o3d.geometry.TriangleMesh()
# self._mesh.vertices = o3d.utility.Vector3dVector(vertices)
# print('here4')
#
# # Step 5: Define the triangle faces
# faces = np.array(faces)  # Triangle 2 (vertices 1, 3, 4)
# print('faces')
# print(faces)
#
# # Step 6: Assign the faces to the mesh
# self._mesh.triangles = o3d.utility.Vector3iVector(faces)
# self._mesh.paint_uniform_color([1, 0.706, 0])
#
# # Step 7: Compute the normals of the mesh
# self._mesh.compute_vertex_normals()
# self._mesh = self._mesh.filter_smooth_laplacian(number_of_iterations=10)
#
# self._mesh.compute_vertex_normals()
# # Step 8: Visualize the mesh
# o3d.visualization.draw_geometries([self._mesh])
#
# pymesh = ml.Mesh(self._mesh.vertices, self._mesh.triangles)
# # pymesh.compute_normals()
#
# # Create pymeshlab instance and add the mesh
# ms = ml.MeshSet()
# ms.add_mesh(pymesh)
#
# # Save as USD file
# usd_path = 'FINAL2.stl'
# ms.save_current_mesh(usd_path)
#
# # Step 1: Define the list of face indices for splitting
# # split_indices = [i for i in range(int(len(mesh.trian-1gles)/2))]  # List of face indices for splitting
# materials = np.unique(res)
# print('The amount of materials to make, ', len(materials))
# new_meshes = [o3d.geometry.TriangleMesh() for i in range(len(materials))]
# print('The amount of meshes to make  ', len(new_meshes))
#
# print(
#     'The amount of triangles in the original mesh ',
#     len(self._mesh.triangles),
#     '   ',
#     np.asarray(self._mesh.triangles).shape[0],
# )
# print('The length of the res ', len(res))
# # for index in range(0, np.asarray(mesh.triangles).shape[0], 2):
# #     if index >= len(mesh.triangles):
# #         print("This index is too high", index)
# #     if index < len(mesh.triangles) and index < l*l:
# #         face = mesh.triangles[index]
# #         face2 = mesh.triangles[index+1]
# #         # get the value in the array
# #         ind = np.unravel_index(index, (l, l))
# #         res_ind = res[ind]
# #         new_meshes[int(res_ind)].triangles.append(face)
# #         new_meshes[int(res_ind)].triangles.append(face2)
# #     else:
# #         print(f"index {index}")
# index_to_try = 0
# other_id = 0
# while index_to_try < len(self._mesh.triangles):
#     face = self._mesh.triangles[index_to_try]
#     face2 = self._mesh.triangles[index_to_try + 1]
#     ind = np.unravel_index(other_id, (l, l))
#     res_ind = res[ind]
#     new_meshes[int(res_ind)].triangles.append(face)
#     new_meshes[int(res_ind)].triangles.append(face2)
#     index_to_try += 2
#     other_id += 1
# print(
#     'the amount of triangles in the meshes added together ',
#     (len(new_meshes[0].triangles) + len(new_meshes[1].triangles)),
# )
# # # Step 3: Create empty TriangleMesh objects for split meshes
# # mesh1 = o3d.geometry.TriangleMesh()
# # mesh2 = o3d.geometry.TriangleMesh()
# #
# # # Step 4: Iterate through the face indices and assign each face to the corresponding mesh
# # for index in range(np.asarray(mesh.triangles).shape[0]):
# #     face = mesh.triangles[index]
# #     if index in split_indices:
# #         mesh1.triangles.append(face)
# #     else:
# #         mesh2.triangles.append(face)
# #
# # Step 5: Assign the original vertices and vertex normals to split meshes
#
# N = len(materials)
# HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
# RGB_tuples = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))
#
# for i, mesh2 in enumerate(new_meshes):
#     mesh2.vertices = self._mesh.vertices
#
#     mesh2.vertex_normals = self._mesh.vertex_normals
#     # v_uv = np.random.rand(len(mesh.triangles) * 3, 2)
#     # mesh2.triangle_uvs = v_uv
#     mesh2 = mesh2.remove_unreferenced_vertices()
#     mesh2.paint_uniform_color(RGB_tuples[i])
#     mesh2 = mesh2.compute_vertex_normals()
#     mesh2 = mesh2.compute_triangle_normals()
# for i, m in enumerate(new_meshes):
#     o3d.io.write_triangle_mesh(
#         filename=f'mesh3_{i}.obj',
#         mesh=m,
#         compressed=False,
#         write_vertex_normals=True,
#         # write_vertex_colors=True,
#         # write_triangle_uvs=True,
#         print_progress=False,
#     )
# print('done')
