from omni.syntheticdata.scripts.sensors import enable_sensors
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
import omni
import asyncio
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import omni.replicator.core as rep
import builtins
import math

import numpy as np
import scipy.spatial.transform as tf
from dataclasses import dataclass
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control
import carb
from pxr import Sdf
from .Sensors.LIDAR import Lidar
from .Sensors.Camera import DepthCamera
#.

def get_world_translation(prim):
    transform = Gf.Transform()
    transform.SetMatrix(
        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    )
    return transform.GetTranslation()


def get_world_pose(prim):
    transform = Gf.Transform()
    transform.SetMatrix(
        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    )
    return transform.GetRotation()


class SensorRig:
    def __init__(self, name, path) -> None:
        print("SensorRig Init function")
        self.__sensors = []
        self.__waypoints = []
        self.__curr_waypoint_id = 0

        self._prim_path = path
        self._prim_name = name
        self._full_prim_path = f"{self._prim_path}/{self._prim_name}"
        self._prim = None
        self._dc = None
        self._rb = None

    def create_rig(self, position, orientation, stage):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        print(f"Dynamic control aquired: {self._dc}")
        # spawn object
        print("Creating rig")
        print(position)
        print(position / get_stage_units())
        self._prim = XFormPrim(
            name=self._prim_name,
            prim_path=self._full_prim_path,
            position=position / get_stage_units(),
            orientation=orientation,
        )

        # collisionAPI = PhysicsRigidBodyAPI.Apply(self._prim)
        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=stage.GetPrimAtPath(self._full_prim_path),
            component="PhysicsRigidBodyAPI",
        )

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{self._full_prim_path}.physxRigidBody:disableGravity"),
            value=True,
            prev=None,
        )
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        print(self._rb)
        print(self.get_pos_rot())
        print(self._dc)

    def apply_veloc(self, veloc):
        print("applying ", veloc)
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        self._dc.set_rigid_body_linear_velocity(self._rb, veloc)

    def add_depth_camera_to_rig(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        image_size=(512, 512),
        attach=True,
        name="/DepthCamera",
    ):
        self.__sensors.append(
            DepthCamera(
                position, rotation, image_size, attach, self._full_prim_path, name
            )
        )

    def add_lidar_to_rig(self, name, origin_pos):
        self.__sensors.append(
            Lidar(path=name, parent=self._full_prim_path, origin_pos=origin_pos)
        )

    def add_sensor_to_rig(self, sensor):
        self.__sensors.append(sensor)
        self.__sensors[-1].init_sensor(self._full_prim_path)

    def sample_sensors(self):
        # Sample all sensors
        for sensor in self.__sensors:
            # sensor.sample_sensor()
            asyncio.ensure_future(sensor.sample_sensor())
        return
        # save position and rotation of sensor rig as a whole, and for each sensor.
        pos, rot = self.get_pos_rot()

    def get_pos_rot(self):
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        object_pose = self._dc.get_rigid_body_pose(self._rb)
        return object_pose.p, object_pose.r

    def initialize_waypoints(self, waypoint_parent_tag, stage):
        # Reset the waypoints
        self.__waypoints = []
        # Get the current sensor rig position and orientation
        current_pos, current_rot = self.get_pos_rot()

        # iter over the stage and get all the waypoints
        # go through each child and save its tranform details to the waypoints list.
        print("Waypoint initialization")
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            if "_waypoints_" in prim_ref_name:
                for i in range(len(prim_ref.GetChildren())):
                    prim_child = prim_ref.GetChildren()[i]
                    self.__waypoints.append(get_world_translation(prim_child))

        print("SensorRig waypoints initialization complete:")
        print(self.__waypoints)

    def _waypoint_update(self, pos):
        # Get the goal position and convert it into the correct type
        goal_pos = self.__waypoints[self.__curr_waypoint_id]
        goal_pos = Gf.Vec3d(goal_pos)

        # Calculate the diff vector
        move_vec = goal_pos - pos
        distance = np.linalg.norm(goal_pos - pos)
        move_vec = (move_vec / distance) * 5
        # convert it to a distance check
        # iter over the points till the next valid one found.

        if distance < 0.5:
            self.__curr_waypoint_id += 1
            if self.__curr_waypoint_id >= len(self.__waypoints):
                self.__curr_waypoint_id = 0
            return self._waypoint_update(pos)

        return move_vec

    def move(self, time_step):
        # Retrieve the current position and orientation of the sensor rig
        current_pos, current_rot = self.get_pos_rot()
        current_pos = Gf.Vec3d(current_pos[0], current_pos[1], current_pos[2])

        # Load the correct waypoint, check if we should change to next one ..
        # and then calculate the required move vector.
        move_vec = self._waypoint_update(current_pos)

        # Apply the required veloc
        self.apply_veloc(move_vec)
