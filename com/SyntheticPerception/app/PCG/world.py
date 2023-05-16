"""
This module contains the required data types and parameter loading
for PCG world ge

"""
# from __future__ import annotations
# import json
# from dataclasses import dataclass
#
# @dataclass
# class XObject:
#     """
#     Class that represents types of an object that can be placed in a world
#     """
#
#     position: list[float]
#     orientation: list[float]
#     poisson_size: float
#     prim_path: str = ""
#
#
# @dataclass
# class Zone:
#     """
#     Class that represents the smallest unit of area in a world - a zone
#     """
#
#     zone_map: list[list[float]] = []
#     xObjects: list[XObject] = []
#
#
# @dataclass
# class Region:
#     """
#     Class that represents a Region containing multiple zones in a world
#     """
#
#     zones: list[Zone] = []
#
#
# @dataclass
# class Environment:
#     """
#     Class that represents an environment containing multiple regions
#     """
#
#     regions: list[Region] = []
#
#
class EnvParamAndData:
     """
     DOCstring
     """

     def __init__(self):
         self._file_path = ""
         self.environments = []

     # Finish this function
     def json_to_obj(self) -> None:
         """
         doc
         """
         with open(self._file_path, "r", encoding="utf-8") as file:
             data = json.load(file)

             for environment in data["Enironment"]:
                 for region in environment["Regions"]:
                     for zone in region["Zones"]:
                         for x_object in zone["XObjects"]:
                             print(x_object)

     def get_environemnt_params(self) -> None:
         """
         doc
         """
         return None

#
# """
#
# {
#   "Environment": {
#     "Regions": {
#       "Zones": {
#         "XObject": {
#           "name": "name",
#           "position": "pos",
#           "orientation": "ori",
#           "poisson_size": "size"
#         }
#       }
#     }
#   }
# }
#
# """
