#from setuptools import setup, find_packages
#setup(
#    name = 'your_package_name',
#    packages = find_packages(),
#)
from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension, build_ext

ext_modules = [
    Pybind11Extension(
        "mesh_module",
        ["mesh_module.cpp"],
    ),
]

setup(
    name="mesh_module",
    version="1.0",
    author="Your Name",
    description="A custom mesh processing package",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
