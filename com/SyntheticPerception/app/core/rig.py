from .objects import Object
import omni
from omni.physx import get_physx_scene_query_interface
import numpy as np

from omni.isaac.core.prims import XFormPrim, RigidPrim
import json

from pxr import Sdf
from omni.isaac.core.utils.stage import get_stage_units
from Sensors.LIDAR import Lidar
from Sensors.IMU import IMUSensor
from Sensors.Camera import DepthCamera


class Agent(Object):
    """
    Class that inherits from and object and has rig and equiptment features.
    """

    def __init__(
        self,
        rig_file_path='',
        *args,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)

        # lock axis of rotation
        self._prim.GetAttribute('physxRigidBody:lockedRotAxis').Set(3)

        self._velocity = 0
        self._sample_rate = 0
        self._sensors = []
        (
            self._initial_translate,
            self._initial_orientation,
        ) = self.create_rig_from_file(rig_file_path)
        self._translate = self._initial_translate
        self._orientation = self._initial_orientation
        print(type(self._orientation))
        self.set_translate(self._translate)
        self.set_orient(self._orientation)

        # Used for death checks
        self._steps_since_moved = 0
        self._max_steps_death_count = 100
        self._death_delta_translate = 0.1
        self._death_delta_orientation = 0.1
        self._last_translate = self._initial_translate
        self._last_orientation = self._initial_orientation

    def ray_cast(self, origin, direction, distance):
        """
        Returns the hit information from a raycast
        Args: origin: List[float], direction: list[float], distance: float
        Returns: hit info

        """
        hit = get_physx_scene_query_interface().raycast_closest(
            origin, direction, distance
        )

        if hit['hit']:
            return hit

        return None

    def create_rig_from_file(self, path):
        """
        Creates all the rig and equiptment from a rig param file
        Args: path: str
        Returns: Gf.Vec3d positin, orientation
        """
        pos, ori = self._load_sensors_from_file(path)
        position = np.array([pos[0], pos[1], pos[2]])
        orientation = np.array([ori[0], ori[1], ori[2], ori[3]])
        return position, orientation

    def step(self, step_size, linear_veloc, angular_veloc):
        """
        Steps the agent by applying commands from the model
        Args: None
        Returns: None

        """

        self._last_translate = self.get_translate()
        self._last_orientation = self.get_orientation()
        self.apply_velocity(linear_veloc, angular_veloc)

        # death check
        is_dead = self._death_check()

        if is_dead:
            self.reset()
            return

    def _death_check(self):
        """
        Checks if the agent has died or not moved in a while
        Args: None
        Returns: boolean
        """
        current_translate = self.get_translate()
        current_orientation = self.get_orientation()
        diff_translate = abs(current_translate - self._last_translate)
        diff_orientation = abs(current_orientation - self._last_orientation)
        if (
            diff_translate < self._death_delta_translate
            and diff_orientation < self._death_delta_orientation
        ):
            self._steps_since_moved += 1

        if self._steps_since_moved >= self._max_steps_death_count:
            return True
        return False

    def _load_sensors_from_file(self, file_path):
        """
        Loads sensors from a sensor file
        Args: file_path: str
        Returns: Gf.Vec3d translation, orientation
        """
        with open(file_path, 'r+') as infile:
            data = json.load(infile)
            # print(data)
            pos = data['POSITION']
            ori = data['ORIENTATION']
            self._velocity = data['VELOCITY']
            self._sample_rate = data['SAMPLE_RATE']

            sensors = data['SENSORS']
            # print(sensors)
            for key in sensors:
                if key == 'LIDAR':
                    for sensor_id in sensors[key]['instances']:
                        sensor_settings = sensors[key]['instances'][sensor_id]
                        lidar = Lidar()
                        lidar.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(lidar)
                elif key == 'CAMERA':
                    print('creating camera')

                    for sensor_id in sensors[key]['instances']:
                        sensor_settings = sensors[key]['instances'][sensor_id]
                        cam = DepthCamera()
                        cam.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(cam)
                elif key == 'IMU':
                    for sensor_id in sensors[key]['instances']:
                        sensor_settings = sensors[key]['instances'][sensor_id]
                        imu = IMUSensor()
                        imu.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(imu)
                else:
                    print(' ERROR, tried adding sensor with type ', key)
            return pos, ori

    def add_sensor_to_rig(self, sensor):
        """
        Adds a sensor to the rig/agent
        Args: sensor: Object (lidar, imu, camera)
        Returns: None
        """
        self._sensors.append(sensor)
        self._sensors[-1].init_sensor(self._prim_path)

    def reset(self):
        """
        reset function that extends default object reset.
        Args: None
        Returns: None
        """
        super().reset()
        # reset other values relating to agent here
