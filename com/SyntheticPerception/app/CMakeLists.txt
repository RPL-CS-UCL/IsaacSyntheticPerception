
#cmake_minimum_required(VERSION 3.16)
#project(mesh_module)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED True)
# Set this to the directory containing Open3DConfig.cmake

#set(Open3D_DIR "/usr/local/Open3D/")
#find_package(Open3D REQUIRED)

#add_subdirectory(pybind11)
#pybind11_add_module(mesh_module mesh_module.cpp)
#add_executable(mesh_module mesh_module.cpp)
#target_link_libraries(mesh_module PRIVATE Open3D::Open3D)
#target_link_libraries(mesh_module PRIVATE ${MY_LIBRARIES})
#target_include_directories(mesh_module PRIVATE ${Open3D_INCLUDE_DIRS})
#target_link_libraries(mesh_module PRIVATE ${Open3D_LIBRARIES})

cmake_minimum_required(VERSION 3.4...3.18)
project(mesh_module)

set(Open3D_DIR "/home/jon/open3d_install/lib/cmake/Open3D/")

find_package(Open3D REQUIRED)
# find_package(Open3D REQUIRED)
add_subdirectory(pybind11)
pybind11_add_module(mesh_module mesh_module.cpp)
target_link_libraries(mesh_module PRIVATE ${Open3D_LIBRARIES})
# install(TARGETS mesh_module DESTINATION /usr/local/bin)
