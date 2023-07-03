import open3d as o3d
import numpy as np

from perlin_numpy import generate_perlin_noise_2d, generate_fractal_noise_2d
from sklearn.preprocessing import normalize
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt
import cv2

import colorsys

# Step 1: Install Open3D if needed
# !pip install open3d
shape = (256, 256)
threshold = 0.5
region_value = 1

l = 2560
data = generate_perlin_noise_2d(shape, (8, 8))
data = (data - np.min(data)) / (np.max(data) - np.min(data))
data[data < threshold] = 0
data[data >= threshold] = region_value
res = cv2.resize(data, dsize=(l, l), interpolation=cv2.INTER_NEAREST)
print("the amount of materials to make should be", np.unique(res))
print(res)
l = 2560
shape = (l, l)
noise1 = PerlinNoise(octaves=3)
noise2 = PerlinNoise(octaves=6)
noise3 = PerlinNoise(octaves=12)
noise4 = PerlinNoise(octaves=24)

xpix, ypix = l, l
pic = []
# for i in range(xpix):
#     row = []
#     for j in range(ypix):
#         noise_val = noise1([i / xpix, j / ypix])
#         # noise_val += 0.5 * noise2([i / xpix, j / ypix])
#         # noise_val += 0.25 * noise3([i / xpix, j / ypix])
#         # noise_val += 0.125 * noise4([i / xpix, j / ypix])
#
#         row.append(noise_val)
#     pic.append(row)
# a = np.array(pic)
# arr = abs(a)
print("generating initial mesh noise")
points = []
all_verts = []
points_np = np.zeros(shape=(l*l,3))

# def vec_fill(x):
#     x,y = x
#     noise_val = noise1([x / xpix, y / ypix])
#     return [x, y, abs(noise_val) * 200]
# fill_array = np.vectorize(vec_fill)
# array = fill_array([x,y] for y in range(l) for x in range(l))
# print(array)
for x in range(l):
    row = []
    for y in range(l):
        # vertex 0

        noise_val = noise1([x / xpix, y / ypix])
        i = x
        j = y
        # noise_val += 0.5 * noise2([i / xpix, j / ypix])
        # noise_val += 0.25 * noise3([i / xpix, j / ypix])
        # noise_val += 0.125 * noise4([i / xpix, j / ypix])
        mypoint = [x, y, abs(noise_val) * 500]
        points_np[ x* l + y] = mypoint

        # all_verts.append(mypoint)
        # points.append(mypoint)
        # row.append(noise_val)
    # pic.append(row)

a = np.array(pic)
arr = abs(a)
# faces = []
# for i in range(1, len(all_verts) - 1):
#     face = [i - 1, i, i + 1]  # Create a face using three consecutive vertices
#     faces.append(face)
# print('here 2 222 l, faces', faces)
print("generating triangle face index")
faces = []
subdivisions = l - 1
for j in range(subdivisions):
    for i in range(subdivisions):
        index = j * (subdivisions + 1) + i
        face1 = [index, index + 1, index + subdivisions + 2]
        face2 = [index, index + subdivisions + 2, index + subdivisions + 1]
        faces.append(face1)
        faces.append(face2)
# Step 3: Create an array representing the mesh vertices
vertices = points_np#np.array(points)  # Vertex 4
print(vertices.shape)
print(type(vertices))

# Step 4: Create a TriangleMesh object and assign the vertices
mesh = o3d.geometry.TriangleMesh()
mesh.vertices = o3d.utility.Vector3dVector(vertices)
print('here4')

# Step 5: Define the triangle faces
faces = np.array(faces)  # Triangle 2 (vertices 1, 3, 4)
print('faces')
print(faces)

# Step 6: Assign the faces to the mesh
mesh.triangles = o3d.utility.Vector3iVector(faces)
mesh.paint_uniform_color([1, 0.706, 0])

# Step 7: Compute the normals of the mesh
mesh.compute_vertex_normals()
mesh = mesh.filter_smooth_laplacian(number_of_iterations=10)

mesh.compute_vertex_normals()
# Step 8: Visualize the mesh
o3d.visualization.draw_geometries([mesh])



# Step 1: Define the list of face indices for splitting
# split_indices = [i for i in range(int(len(mesh.trian-1gles)/2))]  # List of face indices for splitting
materials = np.unique(res)
print("The amount of materials to make, ", len(materials))
new_meshes = [o3d.geometry.TriangleMesh() for i in range(len(materials))]
print("The amount of meshes to make  ", len(new_meshes))

print("The amount of triangles in the original mesh ", len(mesh.triangles), "   ", np.asarray(mesh.triangles).shape[0])
print("The length of the res ", len(res))
# for index in range(0, np.asarray(mesh.triangles).shape[0], 2):
#     if index >= len(mesh.triangles):
#         print("This index is too high", index)
#     if index < len(mesh.triangles) and index < l*l:
#         face = mesh.triangles[index]
#         face2 = mesh.triangles[index+1]
#         # get the value in the array
#         ind = np.unravel_index(index, (l, l))
#         res_ind = res[ind]
#         new_meshes[int(res_ind)].triangles.append(face)
#         new_meshes[int(res_ind)].triangles.append(face2)
#     else:
#         print(f"index {index}")
index_to_try = 0
other_id = 0
while index_to_try < len(mesh.triangles):
    face = mesh.triangles[index_to_try]
    face2 = mesh.triangles[index_to_try+1]
    ind = np.unravel_index(other_id, (l, l))
    res_ind = res[ind]
    new_meshes[int(res_ind)].triangles.append(face)
    new_meshes[int(res_ind)].triangles.append(face2)
    index_to_try += 2
    other_id += 1
print("the amount of triangles in the meshes added together ", (len(new_meshes[0].triangles) + len(new_meshes[1].triangles)))
# # Step 3: Create empty TriangleMesh objects for split meshes
# mesh1 = o3d.geometry.TriangleMesh()
# mesh2 = o3d.geometry.TriangleMesh()
#
# # Step 4: Iterate through the face indices and assign each face to the corresponding mesh
# for index in range(np.asarray(mesh.triangles).shape[0]):
#     face = mesh.triangles[index]
#     if index in split_indices:
#         mesh1.triangles.append(face)
#     else:
#         mesh2.triangles.append(face)
#
# Step 5: Assign the original vertices and vertex normals to split meshes

N = len(materials)
HSV_tuples = [(x * 1.0 / N, 0.5, 0.5) for x in range(N)]
RGB_tuples = list(map(lambda x: colorsys.hsv_to_rgb(*x), HSV_tuples))

for i, mesh2 in enumerate(new_meshes):
    mesh2.vertices = mesh.vertices

    mesh2.vertex_normals = mesh.vertex_normals
    mesh2 = mesh2.remove_unreferenced_vertices()
    mesh2.paint_uniform_color(RGB_tuples[i])
    mesh2.compute_vertex_normals()
# mesh1.vertices = mesh.vertices
# mesh1.vertex_normals = mesh.vertex_normals
# mesh2.vertices = mesh.vertices
# mesh2.vertex_normals = mesh.vertex_normals
# mesh1 = mesh1.remove_unreferenced_vertices()
#
# mesh2 = mesh2.remove_unreferenced_vertices()
#
# mesh1.paint_uniform_color([1, 0, 2])
#
# mesh2.paint_uniform_color([1, 0.706, 1])
# # Step 6: Compute the normals for split meshes
# mesh1.compute_vertex_normals()
# mesh2.compute_vertex_normals()

# Step 7: Visualize the split meshes
o3d.visualization.draw_geometries(new_meshes)
o3d.visualization.draw_geometries([mesh])
print('done')
