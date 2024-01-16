#from setuptools import setup, find_packages
#setup(
#    name = 'your_package_name',
#    packages = find_packages(),
#)
from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext
#

eigen_include_dir = '/usr/include/eigen3'
open3d_include_dir = '/home/jon/open3d_install/include/open3d/'
open3d_lib_dir = "/home/jon/open3d_install/lib/cmake/Open3D/"
# eigen_include_dir = '/usr/include/eigen3'
# #
# open3d_include_dir = '/usr/local/include/Open3D/'
#
# fmt_include_dir = '/usr/include/fmt'
#
#
ext_modules = [
    Pybind11Extension(
        "mesh_module",
        ["mesh_module.cpp"],
      	include_dirs=[open3d_include_dir,eigen_include_dir],
        library_dirs=[open3d_lib_dir],
        #libraries=["Open3D"],  # Add the Open3D library name
    ),
]
#
setup(
    name="mesh_module",
    version="0.11",
    author="Your Name",
    description="A custom mesh processing package",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)

# from setuptools import setup, Extension
# from setuptools.command.build_ext import build_ext
# import sys
# import setuptools
#
# class get_pybind_include(object):
#     """Helper class to determine the pybind11 include path"""
#
#     def __str__(self):
#         import pybind11
#         return pybind11.get_include()
#
# eigen_include_dir = '/usr/include/eigen3'
# open3d_include_dir = '/home/jon/open3d_install/include/'
# open3d_lib_dir = "/home/jon/open3d_install/lib/"
# ext_modules = [
#     Extension(
#         'mesh_module',
#         ['mesh_module.cpp'],
#         include_dirs=[
#             eigen_include_dir,open3d_include_dir,
#             # Path to Open3D includes if not automatically found
#         ],
#         library_dirs=[open3d_lib_dir],
#         language='c++'
#     ),
# ]
#
# setup(
#     name='mesh_module',
#     version='0.1',
#     author='Your Name',
#     description='Python package with Open3D and Pybind11',
#     long_description='',
#     ext_modules=ext_modules,
#     install_requires=['pybind11', 'open3d'],
#     cmdclass={'build_ext': build_ext},
#     zip_safe=False,
# )
