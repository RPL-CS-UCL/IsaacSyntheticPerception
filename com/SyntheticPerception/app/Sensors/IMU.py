import pathlib
from pxr import (

    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
import numpy as np
import omni.replicator.core as rep
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator
import omni

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control

from pxr import Usd, Gf, UsdGeom
from omni.isaac.sensor import _sensor


class IMUSensor:
    def __init__(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        orientation=(1, 1, 1, 1),
        parent='/World',
        name='/DepthCamera',
    ) -> None:
        self.__pos = position
        self.__ori = orientation
        self.__rot = rotation
        self.__name = name
        # self.__imu_prim
        self._is = _sensor.acquire_imu_sensor_interface()
        self.__path = ''
        self._save_path = ""
        #     self.__attach_annotoators()

    def init_output_folder(self, path):
        self._save_path =path +"/posesIMU" 

        pathlib.Path(self._save_path).mkdir(parents=True, exist_ok=True)
        

    def init_sensor(self, parent):
        x,y,z = self.__pos
        qx,qw,qy,qz = self.__rot
        self.__path = parent + "/" + self.__name
        result, self.__imu_prim = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor',
            path='/' + self.__name,
            parent=parent,
            sensor_period=-1.0,
            translation=Gf.Vec3d(x,y,z),
            orientation=Gf.Quatd(qx,qw,qy,qz),
            visualize=True,
        )

    def read_from_json(self, data):
        print("Trying to read IMU settings")
        # We have been given data["LIDAR"]
        self.__name = data['name']
        self.__pos = data['position']
        self.__rot =data['rotation']

    def sample_sensor(self):
        # print(self.__path)
        # return

        # await rep.orchestrator.step_async()
        reading = self._is.get_sensor_readings(self.__path)
        # print(reading)
        # print(reading)
        # self.sliders[0].model.set_value(float(reading[-1]["lin_acc_x"]) * self.meters_per_unit)  # readings
        #                 self.sliders[1].model.set_value(float(reading[-1]["lin_acc_y"]) * self.meters_per_unit)  # readings
        #                 self.sliders[2].model.set_value(float(reading[-1]["lin_acc_z"]) * self.meters_per_unit)  # readings
        #                 self.sliders[3].model.set_value(float(reading[-1]["ang_vel_x"]))  # readings
        #                 self.sliders[4].model.set_value(float(reading[-1]["ang_vel_y"]))  # readings
        #                 self.sliders[5].model.set_value(float(reading[-1]["ang_vel_z"]))  # readings
        #                 self.sliders[6].model.set_value(float(reading[-1]["orientation"][0]))  # readings
        #                 self.sliders[7].model.set_value(float(reading[-1]["orientation"][1]))  # readings
        #                 self.sliders[8].model.set_value(float(reading[-1]["orientation"][2]))  # readings
        #                 self.sliders[9].model.set_value(float(reading[-1]["orientation"][3]))  # readings

