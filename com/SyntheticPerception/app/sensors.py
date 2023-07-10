from omni.syntheticdata.scripts.sensors import enable_sensors
import os

import pathlib
import json
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
from .Sensors.IMU import IMUSensor
from .Sensors.Camera import DepthCamera
from omni.isaac.core.utils.rotations import (
    lookat_to_quatf,
    quat_to_euler_angles,
    gf_quat_to_np_array,
)

# .


def get_world_translation(prim):
    transform = Gf.Transform()
    transform.SetMatrix(
        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
    )
    return transform.GetTranslation()


def get_world_pose(prim):
    transform = Gf.Transform()
    transform.SetMatrix(
        UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
    )
    return transform.GetRotation()


class SensorRig:
    def __init__(self, name, path) -> None:
        print('SensorRig Init function')
        self.__sensors = []
        self.__waypoints = []
        self.__curr_waypoint_id = 0

        self._prim_path = path
        self._prim_name = name
        self._full_prim_path = f'{self._prim_path}/{self._prim_name}'
        self._prim = None
        self._dc = None
        self._rb = None
        self.start_time = 0 

    def create_rig_from_file(self, path, stage):
        pos, ori = self.load_sensors_from_file(path, stage)
        print(pos, ori)
        position = np.array([pos[0], pos[1], pos[2]])
        orientation = np.array([ori[0], ori[1], ori[2], ori[3]])
        print('returned pos or')
        print(position, orientation)
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        print(f'Dynamic control aquired: {self._dc}')
        # spawn object
        print('Creating rig')
        # print(position)
        # print(position / get_stage_units())
        self._prim = XFormPrim(
            name=self._prim_name,
            prim_path=self._full_prim_path,
            position=position / get_stage_units(),
            orientation=orientation,
        )

        # collisionAPI = PhysicsRigidBodyAPI.Apply(self._prim)
        omni.kit.commands.execute(
            'AddPhysicsComponent',
            usd_prim=stage.GetPrimAtPath(self._full_prim_path),
            component='PhysicsRigidBodyAPI',
        )

        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{self._full_prim_path}.physxRigidBody:disableGravity'
            ),
            value=True,
            prev=None,
        )
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        print(self._rb)
        print(self.get_pos_rot())
        print(self._dc)
        print('Rig created')

    def create_rig(self, position, orientation, stage):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        print(f'Dynamic control aquired: {self._dc}')
        # spawn object
        print('Creating rig')
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
            'AddPhysicsComponent',
            usd_prim=stage.GetPrimAtPath(self._full_prim_path),
            component='PhysicsRigidBodyAPI',
        )

        omni.kit.commands.execute(
            'ChangeProperty',
            prop_path=Sdf.Path(
                f'{self._full_prim_path}.physxRigidBody:disableGravity'
            ),
            value=True,
            prev=None,
        )
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        print(self._rb)
        print(self.get_pos_rot())
        print(self._dc)

    def apply_veloc(self, veloc, ang_veloc):
        print('applying ', veloc)
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        self._dc.set_rigid_body_linear_velocity(self._rb, veloc)

        object_pose = self._dc.get_rigid_body_pose(self._rb)
        object_pose.r = ang_veloc
        self._dc.set_rigid_body_pos(self._rb, ang_veloc)

        # omni.kit.commands.execute(
        #     'TransformPrimSRTCommand',
        #     path=prim_path,  # f"/World/{p_name}",
        #     old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
        #     new_scale=Gf.Vec3f(scale, scale, scale),
        #     old_translation=Gf.Vec3f(x, y, z),
        #     new_translation=Gf.Vec3f(x, y, z),
        #     old_rotation_euler=Gf.Vec3f(0, 0, 0),
        #     old_rotation_order=Gf.Vec3i(0, 1, 2),
        #     new_rotation_euler=Gf.Vec3f(0, 0, random_rotation),
        #     new_rotation_order=Gf.Vec3i(0, 1, 2),
        #     time_code=Usd.TimeCode(),
        #     had_transform_at_key=False,
        # )
        # omni.kit.commands.execute(
        #     'TransformPrimSRTCommand',
        #     path=prim_path,  # f"/World/{p_name}",
        #     old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
        #     new_scale=Gf.Vec3f(scale, scale, scale),
        #     old_translation=Gf.Vec3f(x, y, z),
        #     new_translation=Gf.Vec3f(x, y, z),
        #     old_rotation_euler=Gf.Vec3f(0, 0, 0),
        #     old_rotation_order=Gf.Vec3i(0, 1, 2),
        #     new_rotation_euler=Gf.Vec3f(0, 0, random_rotation),
        #     new_rotation_order=Gf.Vec3i(0, 1, 2),
        #     time_code=Usd.TimeCode(),
        #     had_transform_at_key=False,
        # )
        # self._dc.set_rigid_body_angular_velocity(self._rb, ang_veloc)

    def add_depth_camera_to_rig(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        image_size=(512, 512),
        attach=True,
        name='/DepthCamera',
    ):
        self.__sensors.append(
            DepthCamera(
                position,
                rotation,
                image_size,
                attach,
                self._full_prim_path,
                name,
            )
        )

    def add_lidar_to_rig(self, name, origin_pos):
        self.__sensors.append(
            Lidar(
                path=name, parent=self._full_prim_path, origin_pos=origin_pos
            )
        )

    def add_sensor_to_rig(self, sensor):
        self.__sensors.append(sensor)
        self.__sensors[-1].init_sensor(self._full_prim_path)

    def sample_sensors(self):
        print("sampling sensors")
        # log timestep
        # Sample all sensors
        for sensor in self.__sensors:
            # sensor.sample_sensor()
            asyncio.ensure_future(sensor.sample_sensor())
        return

    def get_pos_rot(self):
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        object_pose = self._dc.get_rigid_body_pose(self._rb)
        return object_pose.p, object_pose.r

    def initialize_waypoints(self, waypoint_parent_tag, stage):
        # Reset the waypoints
        self.__waypoints = []
        # Get the current sensor rig position and orientation
        # current_pos, current_rot = self.get_pos_rot()

        # iter over the stage and get all the waypoints
        # go through each child and save its tranform details to the waypoints list.
        print('Waypoint initialization')
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            if '_waypoints_' in prim_ref_name:
                for i in range(len(prim_ref.GetChildren())):
                    prim_child = prim_ref.GetChildren()[i]
                    self.__waypoints.append(get_world_translation(prim_child))

        print('SensorRig waypoints initialization complete:')
        print(self.__waypoints)
    def initialize_waypoints_preloaded(self, waypoints):
        self.__waypoints = []
        self.__waypoints = waypoints

    def _waypoint_update(self, pos):
        
        # Get the goal position and convert it into the correct type
        goal_pos = self.__waypoints[self.__curr_waypoint_id]
        goal_pos = Gf.Vec3d(goal_pos)
        ori_ = lookat_to_quatf(pos, goal_pos, Gf.Vec3d(0, 0, 1))
        rot_vec = ori_
        # ori_np = gf_quat_to_np_array(ori_)
        # rot_vec = quat_to_euler_angles(ori_np)
        print(' =============== ', ori_)

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

        return move_vec, rot_vec

    def move(self, time_step):

        timeline = omni.timeline.get_timeline_interface()

        # timecode = (
        #     timeline.get_current_time() * timeline.get_time_codes_per_seconds()
        # )
        self.start_time += time_step
        # print(self.start_time)

        self.sample_sensors()
        # print(timeline.get_current_time())
        if len(self.__waypoints) == 0:
            return
        # Retrieve the current position and orientation of the sensor rig
        current_pos, current_rot = self.get_pos_rot()
        current_pos = Gf.Vec3d(current_pos[0], current_pos[1], current_pos[2])
        print('The current orientation of the SR is ', current_rot)

        # Load the correct waypoint, check if we should change to next one ..
        # and then calculate the required move vector.
        move_vec, rot_vec = self._waypoint_update(current_pos)

        # Apply the required veloc
        self.apply_veloc(move_vec, rot_vec)

    def load_sensors_from_file(self, file_path, stage):
        with open(file_path, 'r+') as infile:
            data = json.load(infile)
            # print(data)
            pos = data['POSITION']
            ori = data['ORIENTATION']

            self.create_rig(np.array(pos), np.asarray(ori), stage)
            sensors = data['SENSORS']
            print(sensors)
            for key in sensors:
                print('Trying to add sensor of type ', key)
                if key == 'LIDAR':
                    for sensor_id in sensors[key]['instances']:
                        sensor_settings = sensors[key]['instances'][sensor_id]
                        lidar = Lidar()
                        lidar.read_from_json(sensor_settings)
                        self.add_sensor_to_rig(lidar)
                elif key == 'CAMERA':

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

    def init_output_folder(self, path):
        # create any paths needed
        pathlib.Path(path).mkdir(parents=True, exist_ok=True)
        # create any needed directories for the sensors
        for sensor in self.__sensors:
            sensor.init_output_folder(path)


"""
{"POSITION" : [0,0,0],
  "ORIENTATION" : [0,0,0,0],
  "SENSORS":{
  
  "IMU":{
  "instances":
    {"1" : 
      {
        "name" : "imu",
        "position": [0.0, 0.0, 0.0],
        "rotation" : [0.0,0.0,0.0]
     }
    }
},
"CAMERA" :
  {"instances" :
    {"1" :
    {
      "name" : "camera",
      "focal_length": 24.0,
      "focus_distance" : 400.0,
      "f_stop": 0.0,
      "horizontal_aperture": 20.955,
      "horizontal_aperture_offset": 0,
      "vertical_aperture_offset": 0,
      "clipping_range": [1.0, 1000000.0],
      "resolution": [1024,1024],
      "position" : [0.0,0.0,0.0],
      "rotation" : [0.0,0.0,0.0]
      
    }
    }
  },
"LIDAR":
  {"instances" : 
    {"1" : 
      {"name": 1,
        "min_range": 0.4,
        "max_range": 100.0,
        "draw_points": false,
        "draw_lines" : false,
        "horizontal_fov": 360,
        "vertical_fov": 60.0,
        "rotation_rate": 0.0,
        "horizontal_resolution": 0.4,
        "vertical_resolution" : 0.4,
        "high_lod":true,
        "yaw_offset": 0.0,
        "enable_semantics":true,
        "origin_pos": [0,0,0],
        "rotation" : [0.0,0.0,0.0]
        
      }
    }
  }
}
}
"""
