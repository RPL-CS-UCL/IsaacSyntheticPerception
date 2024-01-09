#include <iostream>
#include <mutex>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <thread>
#include <unordered_map>
#include <vector>
// #include <open3d/Open3D.h>

namespace py = pybind11;

// std::unordered_map<int, std::vector<std::vector<int>>> compute_base_mesh(
//     int size,
//     double scale,
//     const std::vector<std::vector<int>>& regions_map) {
//
//     int subdivisions = size * scale - 1;
//     std::unordered_map<int, std::vector<std::vector<int>>> meshes_dict;
//
//     // Assuming materials are unique integers in regions_map
//     for (const auto& row : regions_map) {
//         for (int mat : row) {
//             meshes_dict[mat]; // This will default initialize the vector if
//             the key doesn't exist
//         }
//     }
//
//     std::vector<std::vector<int>> faces;
//
//     for (int j = 0; j < subdivisions; ++j) {
//         for (int i = 0; i < subdivisions; ++i) {
//             int index = j * (subdivisions + 1) + i;
//
//             std::vector<int> face1 = {index, index + 1, index + subdivisions
//             + 2}; std::vector<int> face2 = {index, index + subdivisions + 2,
//             index + subdivisions + 1};
//
//             faces.push_back(face1);
//             faces.push_back(face2);
//
//             int res_ind = regions_map[j][i];
//             meshes_dict[res_ind].push_back(face1);
//             meshes_dict[res_ind].push_back(face2);
//         }
//     }
//
//     return meshes_dict;
// }
void compute_mesh_chunk(
    int start, int end, int subdivisions,
    const std::vector<std::vector<int>> &regions_map,
    std::unordered_map<int, std::vector<std::vector<int>>> &meshes_dict,
    std::mutex &dict_mutex) {

  for (int j = start; j < end; ++j) {
    for (int i = 0; i < subdivisions; ++i) {
      int index = j * (subdivisions + 1) + i;

      std::vector<int> face1 = {index, index + 1, index + subdivisions + 2};
      std::vector<int> face2 = {index, index + subdivisions + 2,
                                index + subdivisions + 1};

      int res_ind = regions_map[j][i];

      // Lock the mutex to safely update the shared data structure
      std::lock_guard<std::mutex> guard(dict_mutex);
      meshes_dict[res_ind].push_back(face1);
      meshes_dict[res_ind].push_back(face2);
    }
  }
}

std::unordered_map<int, std::vector<std::vector<int>>>
compute_base_mesh(int size, double scale,
                  const std::vector<std::vector<int>> &regions_map) {

  std::cout << "**** running multithreahd****" << std::endl;
  int subdivisions = size * scale - 1;
  std::unordered_map<int, std::vector<std::vector<int>>> meshes_dict;
  std::mutex dict_mutex;

  // Determine the number of threads
  unsigned int num_threads = std::thread::hardware_concurrency();
  std::vector<std::thread> threads(num_threads);

  // Calculate the chunk size for each thread
  int chunk_size = subdivisions / num_threads;

  // Launch threads
  for (unsigned int i = 0; i < num_threads; ++i) {
    int start = i * chunk_size;
    int end = (i == num_threads - 1) ? subdivisions : start + chunk_size;
    threads[i] = std::thread(compute_mesh_chunk, start, end, subdivisions,
                             std::ref(regions_map), std::ref(meshes_dict),
                             std::ref(dict_mutex));
  }

  // Join threads
  for (auto &t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  // std::unordered_map<int, std::shared_ptr<open3d::geometry::TriangleMesh>>
  //     open3d_meshes;
  //
  // for (const auto &pair : meshes_dict) {
  //   int material_id = pair.first;
  //   const auto &faces = pair.second;
  //
  //   auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  //
  //   // Assuming you have a way to convert face indices to actual vertices and
  //   // triangles This is a placeholder for actual mesh creation logic
  //   for (const auto &face : faces) {
  //     // Add vertices and triangles to your mesh
  //     // mesh->vertices.push_back(...);
  //     // mesh->triangles.push_back(...);
  //   }
  //
  //   open3d_meshes[material_id] = mesh;
  // }

  return meshes_dict;
}

PYBIND11_MODULE(mesh_module, m) {
  m.def("compute_base_mesh", &compute_base_mesh, "Compute the base mesh");
}
