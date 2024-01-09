cmake_minimum_required(VERSION 3.4...3.18)
project(mesh_module)

add_subdirectory(pybind11)
pybind11_add_module(mesh_module mesh_module.cpp)

