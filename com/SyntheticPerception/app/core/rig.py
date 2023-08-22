from .objects import Object
import omni
from omni.physx import get_physx_scene_query_interface
import numpy as np

from omni.isaac.core.prims import XFormPrim, RigidPrim
import json

from pxr import Sdf
from omni.isaac.core.utils.stage import get_stage_units
from ..Sensors.LIDAR import Lidar
from ..Sensors.IMU import IMUSensor
from ..Sensors.Camera import DepthCamera

class Rig(Object):
    def __init__(
        self,

        position,
        rotation,
        scale,
        prim_name,
        parent_path,
        stage,

        usd_path=None,
        semantic_class="None",

        visibility = True,
        gravity = True,
        instanceable=False,
        rig_file_path = "",

    ) -> None:
        super().__init__(
            position,
            rotation,
            scale,
            prim_name,
            parent_path,
            stage,
            usd_path=usd_path,
            semantic_class=semantic_class,
            instanceable=instanceable,

            visibility = visibility,
            gravity =gravity 
        )

        # init rig related stuff
        # add rigid body with certain collider
        self._velocity = 0
        self._sample_rate = 0
        self._sensors = []
        self.create_rig_from_file(rig_file_path)

    def ray_cast(self, origin, direction, distance):
        """
        Returns the hit information from a raycast
        Args: origin: List[float], direction: list[float], distance: float
        Returns: hit info

        """
        hit = get_physx_scene_query_interface().raycast_closest(
            origin, direction, distance
        )

        if hit["hit"]:
            return hit

        return None

    def create_rig_from_file(self, path):
        pos, ori = self.load_sensors_from_file(path)
        position = np.array([pos[0], pos[1], pos[2]])
        orientation = np.array([ori[0], ori[1], ori[2], ori[3]])

    def load_sensors_from_file(self, file_path):
        with open(file_path, "r+") as infile:
            data = json.load(infile)
            # print(data)
            pos = data["POSITION"]
            ori = data["ORIENTATION"]
            self._velocity = data["VELOCITY"]
            self._sample_rate = data["SAMPLE_RATE"]

            # self.create_rig(np.array(pos), np.asarray(ori), stage)
            sensors = data["SENSORS"]
            print(sensors)
            for key in sensors:
                if key == "LIDAR":
                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        lidar = Lidar()
                        lidar.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(lidar)
                elif key == "CAMERA":
                    print("creating camera")

                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        cam = DepthCamera()
                        cam.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(cam)
                elif key == "IMU":
                    for sensor_id in sensors[key]["instances"]:
                        sensor_settings = sensors[key]["instances"][sensor_id]
                        imu = IMUSensor()
                        imu.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(imu)
                else:
                    print(" ERROR, tried adding sensor with type ", key)
            return pos, ori

    def add_sensor_to_rig(self, sensor):
        self._sensors.append(sensor)
        self._sensors[-1].init_sensor(self._prim_path)
