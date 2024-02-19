#include <iostream>
#include <mutex>
#include <open3d/Open3D.h>
#include <ostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <vector>
#include <string>
namespace py = pybind11;
std::vector<uint8_t> convert_to_bytes(
    const std::vector<std::array<float, 3>>& points,
    const std::vector<float>& intensities,
    const std::vector<uint64_t>& timestamps,
    const std::vector<uint16_t>& rings,
    const std::vector<uint32_t>& ranges) {
    
    std::vector<uint8_t> buffer;
    
    for (size_t i = 0; i < points.size(); ++i) {
        // Pack each point
        for (const auto& val : points[i]) {
            auto bytes = reinterpret_cast<const uint8_t*>(&val);
            buffer.insert(buffer.end(), bytes, bytes + sizeof(float));
        }

        buffer.insert(buffer.end(), 4, 0); // 4 bytes of padding

        auto intensity_bytes = reinterpret_cast<const uint8_t*>(&intensities[i]);
        buffer.insert(buffer.end(), intensity_bytes, intensity_bytes + sizeof(float));
        
        uint32_t timestamp = static_cast<uint32_t>(timestamps[i] / 10000);
        auto timestamp_bytes = reinterpret_cast<const uint8_t*>(&timestamp);
        buffer.insert(buffer.end(), timestamp_bytes, timestamp_bytes + sizeof(uint32_t));

        buffer.insert(buffer.end(), {0, 0}); // Reflectivity (2 bytes)
        
        auto ring_bytes = reinterpret_cast<const uint8_t*>(&rings[i]);
        buffer.insert(buffer.end(), ring_bytes, ring_bytes + sizeof(uint16_t));

        buffer.insert(buffer.end(), {0, 0}); // Ambient (2 bytes)

        buffer.insert(buffer.end(), 2, 0); // 2 bytes of padding

        auto range_bytes = reinterpret_cast<const uint8_t*>(&ranges[i]);
        buffer.insert(buffer.end(), range_bytes, range_bytes + sizeof(uint32_t));

        buffer.insert(buffer.end(), 4, 0); // 4 bytes of padding
    }
    
    return buffer;
}
// std::vector<std::vector<float>>
// apply_texture(std::vector<std::vector<float>> &terrainHeightmap,
//               std::vector<std::vector<float>> &brickHeightMap, int size) {
//   for (int y = 0; y < size; ++y) {
//     for (int x = 0; x < size; ++x) {
//       // Calculate the 1D index for the terrain heightmap
//       int index = y * size + x;
//
//       // Find the corresponding point in the brick heightmap
//       int brickX = x % brickHeightMap.size();
//       int brickY = y % brickHeightMap.size();
//
//       // Apply the brick heightmap onto the terrain heightmap
//       terrainHeightmap[index][2] += brickHeightMap[brickY][brickX]/200;
//     }
//   }
//   return terrainHeightmap;
// }
//
// std::vector<std::vector<float>>
// apply_brick(std::vector<std::vector<float>> &points, int size) {
//   std::cout << " begining brick modification" << std::endl;
//   const int b_width = 5;                  // Brick width (integral)
//   const int b_height = 2;                 // Brick height (integral)
//   const int mortar_width = 1;             // Mortar width (integral)
//   const int mortar_height = 1;            // Mortar height (integral)
//   const float brick_height_value = 2.0f;  // Height of bricks (floating-point)
//   const float mortar_height_value = 0.5f; // Height of mortar (floating-point)
//   const int l = size;
//   for (int i = 0; i < points.size(); ++i) {
//     // Convert 1D index to 2D coordinates
//     int x = i % l;
//     int y = i / l;
//     // Determine if the point is on a brick or in mortar
//     bool isBrick = (x % (b_width + mortar_width) < b_width) &&
//                    (y % (b_height + mortar_height) < b_height);
//
//     // Set height accordingly
//     points[i][2] += isBrick ? brick_height_value : mortar_height_value;
//   }
//
//   return points;
// }
std::vector<std::vector<float>>
build_meshes(int size, double scale,
             const std::vector<std::vector<int>> &regions_map,
             std::vector<std::vector<float>> &points, std::string file_path,
             std::vector<std::vector<float>> &terrainHeightmap,

                const int chunkSize
             ) {

  // std::cout << "starting building meshpoints array" << std::endl;

  int subdivisions = size * scale - 1;
  // apply_texture(points,terrainHeightmap, subdivisions);
  // apply_brick(points, subdivisions);

  // Convert the height map points vector to the eigen vec representation
  std::vector<Eigen::Vector3d> points_eigen;
  for (const auto &pts : points) {

    Eigen::Vector3d v(pts[0], pts[1], pts[2]);
    points_eigen.push_back(v);
  }

  // std::cout << "started building of the chunk mat dict " << std::endl;
  std::unordered_map<
      int,
      std::unordered_map<int, std::shared_ptr<open3d::geometry::TriangleMesh>>>
      chunk_mat_dict;


  int length = size * scale;
  std::vector<std::vector<float>> return_points(
      length, std::vector<float>(length, 0.0f));
  int l = subdivisions + 1;
  // std::cout << "begin fixing hte points array" << std::endl;
  // std::cout << points.size() << std::endl;
  // std::cout << return_points.size() << std::endl;
  for (int i = 0; i < points.size(); ++i) {
    int row = i / l;
    int col = i % l;
    // std::cout << points[i][2] << std::endl;
    // std::cout << ":: " << row << " , " << col << std::endl;
    return_points[row][col] = points[i][2]; // Accessing the z-coordinate
  }
  // free the vec
  points.clear();
  int numberOfChunkCreated = 0;
  for (int j = 0; j < subdivisions; ++j) {
    for (int i = 0; i < subdivisions; ++i) {
      int index = j * (subdivisions + 1) + i;
      int chunkRow = i / chunkSize;
      int chunkCol = j / chunkSize;

      // Calculate number of chunks per row
      int chunksPerRow = size / chunkSize;

      // Calculate unique chunk number
      int chunkNumber = chunkRow * chunksPerRow + chunkCol;

      int key = chunkNumber;
      // std::cout << "trying to see if the key ecists" << std::endl;
      auto it = chunk_mat_dict.find(key);
      if (it != chunk_mat_dict.end()) {

        // std::cout << "exists" << std::endl;
      } else {

        // std::cout << "need to create" << std::endl;
        // Key does not exist
        chunk_mat_dict[key] = std::unordered_map<
            int, std::shared_ptr<open3d::geometry::TriangleMesh>>();
          numberOfChunkCreated += 1;
      }

      Eigen::Vector3i vec1(index, index + 1,
                           index + subdivisions +
                               2); // face1[0], face1[1], face1[2]
      Eigen::Vector3i vec2(index, index + subdivisions + 2,
                           index + subdivisions +
                               1); // face2[0], face2[1], face2[2]

      int res_ind = regions_map[j][i];

      // by this point the chunk map for this chunk should contain another map
      // we now need to see if the individual material exists here

      int material_key = res_ind;

      auto it2 = chunk_mat_dict[key].find(material_key);
      if (it2 != chunk_mat_dict[key].end()) {

      } else {
        auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
        chunk_mat_dict[chunkNumber][material_key] = mesh;
      }

      chunk_mat_dict[chunkNumber][material_key]->triangles_.push_back(vec1);
      chunk_mat_dict[chunkNumber][material_key]->triangles_.push_back(vec2);
    }
  }
  // std::cout << "we have created " << numberOfChunkCreated << " chunks" << std::endl;


  for (auto &key_value : chunk_mat_dict) {

    int chunkID = key_value.first;
    auto &chunkMeshes = key_value.second;

    for (auto &key_mesh :chunkMeshes) {

      int matID = key_mesh.first;
      auto mesh = key_mesh.second;
      // std::cout << "Chunk " << chunkID << " mat id " << matID << std::endl;

      mesh->vertices_ = points_eigen;

      // std::cout << "vertices asigned"<< std::endl;
      mesh->RemoveUnreferencedVertices();

      mesh->ComputeVertexNormals();
      mesh->ComputeTriangleNormals();

      // std::cout << "mesh values all asigned fine"<< std::endl;
      std::string name = file_path; //"/home/jon/Desktop/mesh_";
      name += "/mesh_";
      name += std::to_string(chunkID);
      name += "_";

      name += std::to_string(matID);
      name += ".obj";

      // std::cout << name << std::endl;
      open3d::io::WriteTriangleMesh(
          name, *mesh,
          /* write_ascii = */ false, // If you want to write in binary format,
                                     // set this to false
          /* compressed = */ false,
          /* write_vertex_normals = */ true,
          /* write_vertex_colors = */ false,
          /* write_triangle_uvs = */ false,
          /* print_progress = */ true);

    mesh.reset();
     }
  }

return return_points;
}

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

  return meshes_dict;
}


void test_open3d() {

  auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();

  std::cout << "runs fine" << mesh->HasTriangles() << std::endl;
}
int main() {

  auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
  std::cout << "runs fine" << mesh->HasTriangles() << std::endl;
  return 0;
}
PYBIND11_MODULE(mesh_module, m) {
  m.def("compute_base_mesh", &compute_base_mesh, "Compute the base mesh");
  m.def("build_meshes", &build_meshes, "build meshes");
  m.def("test_open3d", &test_open3d, "build meshes");
  m.def("convert_to_bytes", &convert_to_bytes, "convert to bytes");
}
