import pathlib
from omni.syntheticdata.scripts.sensors import enable_sensors
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from omni.isaac.range_sensor import _range_sensor
import omni
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
import omni.replicator.core as rep
import numpy as np
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control

from pxr import Sdf


class Lidar:
    def __init__(
        self,
        path='/Lidar1',
        parent='/World',
        min_range=0.4,
        max_range=100.0,
        draw_points=False,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=60.0,
        horizontal_resolution=0.4,
        vertical_resolution=0.4,
        rotation_rate=0,
        high_lod=True,
        yaw_offset=0.0,
        enable_semantics=False,
        origin_pos=(2.0, 0.0, 4.0),
    ):
        self.__path = '/' + path
        self.__min_range = min_range
        self.__max_range = max_range
        self.__draw_points = draw_points
        self.__draw_lines = draw_lines
        self.__horizontal_fov = horizontal_fov
        self.__vertical_fov = vertical_fov
        self.__horizontal_resolution = horizontal_resolution
        self.__vertical_resolution = vertical_resolution
        self.__rotation_rate = rotation_rate
        self.__high_lod = high_lod
        self.__yaw_offset = yaw_offset
        self.__enable_semantics = enable_semantics
        self.__origin_pos = origin_pos
        self.__rotation = [0.0,0.0,0.0]
        self.sample_count = 0
        self.save_path = None

    def init_output_folder(self, path):
        self.save_path = path

        pathlib.Path(path +"/velodyne").mkdir(parents=True, exist_ok=True)
        
        pathlib.Path(path +"/velodyneLabels").mkdir(parents=True, exist_ok=True)
    def read_from_json(self, data):
        # We have been given data["LIDAR"]
        # for instance_ids in data:
        lidar_settings = data
        print(lidar_settings["name"])
        self.__path = '/' + lidar_settings['name']
        self.__min_range = lidar_settings['min_range']
        self.__max_range = lidar_settings['max_range']
        self.__draw_points = lidar_settings['draw_points']
        self.__draw_lines = lidar_settings['draw_lines']
        self.__horizontal_fov = lidar_settings['horizontal_fov']
        self.__vertical_fov = lidar_settings['vertical_fov']
        self.__horizontal_resolution = lidar_settings[
            'horizontal_resolution'
        ]
        self.__vertical_resolution = lidar_settings['vertical_resolution']
        self.__rotation_rate = lidar_settings['rotation_rate']
        self.__high_lod = lidar_settings['high_lod']
        self.__yaw_offset = lidar_settings['yaw_offset']
        self.__enable_semantics = lidar_settings['enable_semantics']
        self.__origin_pos = lidar_settings['origin_pos']
        self.__rotation = lidar_settings['rotation']

    def init_sensor(self, parent):
        print(f'init the lidar {parent}')
        # self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
        _, self.__lidar_prim = omni.kit.commands.execute(
            'RangeSensorCreateLidar',
            path=self.__path,
            parent=parent,
            min_range=self.__min_range,
            max_range=self.__max_range,
            draw_points=self.__draw_points,
            draw_lines=self.__draw_lines,
            horizontal_fov=self.__horizontal_fov,
            vertical_fov=self.__vertical_fov,
            horizontal_resolution=self.__horizontal_resolution,
            vertical_resolution=self.__vertical_resolution,
            rotation_rate=self.__rotation_rate,
            high_lod=self.__high_lod,
            yaw_offset=self.__yaw_offset,
            enable_semantics=self.__enable_semantics,
        )
        UsdGeom.XformCommonAPI(self.__lidar_prim).SetTranslate(
            self.__origin_pos
        )
        self.__lidar_path = parent +  self.__path
        print(f'lidar path should be {self.__lidar_path}')
        self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

    # def sample_sensor(self):
    #     self.get_pc_and_semantic()
    def sample_sensor(self):
        # return
        self.get_pc_and_semantic()

        self.sample_count += 1

    def get_pc_and_semantic(self, save_path='/home/jon/Documents/temp/a'):
        pointcloud = self.__lidarInterface.get_point_cloud_data(
            self.__lidar_path
        )
        semantics = self.__lidarInterface.get_semantic_data(self.__lidar_path)

        np.save(f"{self.save_path}velodyne/{self.sample_count}.npy",pointcloud)
        np.save(f"{self.save_path}velodyneLabels/{self.sample_count}.npy",semantics)
        # lidar_position = self.__get_position()
        # pointcloud, semantics = self.__clear_max_lidar_points(
        #     pointcloud, semantics, lidar_position, self.__max_range
        # )

        # if save_path is not None:
        #     np.save(
        #         f'{save_path}_pc.npy',
        #         np.array(pointcloud),
        #     )
        #     np.save(
        #         f'{save_path}_sem.npy',
        #         np.array(semantics),
        #     )
        return pointcloud, semantics

    def __get_position(self):
        transform = Gf.Transform()
        transform.SetMatrix(
            UsdGeom.Xformable(self.__lidar_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
        )
        return transform.GetTranslation()

    def __clear_max_lidar_points(self, pc, sem, lidar_pos, max_dist):
        new_points = []
        new_sems = []
        for seq_id in range(len(pc)):
            for point_id in range(len(pc[seq_id])):
                point = pc[seq_id][point_id]
                dist = np.linalg.norm(point - lidar_pos)
                if dist < max_dist - 10:
                    new_points.append(pc[seq_id][point_id])
                    new_sems.append(sem[seq_id][point_id])

        return np.array(new_points), np.array(new_sems)
