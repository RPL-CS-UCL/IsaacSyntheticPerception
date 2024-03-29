"""
This class represents the SensorRig and supporting functions. 
The goal of this class is to provide an easy control method to sample an array of sensors 
and control its movement within and environment. 

The SensorRig primarily has a collection of sensors that is read in from a json file. 
These sensors are created and stored depending on the parameters and are contructed within 
their own classes. See the Sensors folder for all available implemented sensors. 
The rig also handles sampling rates and timestamps. 



"""
from omni.syntheticdata.scripts.sensors import enable_sensors

from omni.syntheticdata import helpers
from omni.isaac.core.utils.prims import define_prim, delete_prim
import pathlib
import json
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
import omni
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import omni.replicator.core as rep
import math


import numpy as np
import scipy.spatial.transform as tf
from dataclasses import dataclass
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator
from omni.physx import get_physx_scene_query_interface
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

from numpy.linalg import norm
import copy
import traceback
from scipy.spatial.transform import Rotation


def quat_to_euler_angles(q):
    q_img = q.GetImaginary()
    q_real = q.GetReal()
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q_real * q_img[0] + q_img[1] * q_img[2])
    cosr_cosp = 1 - 2 * (q_img[0] * q_img[0] + q_img[1] * q_img[1])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q_real * q_img[1] - q_img[2] * q_img[0])
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q_real * q_img[2] + q_img[0] * q_img[1])
    cosy_cosp = 1 - 2 * (q_img[1] * q_img[1] + q_img[2] * q_img[2])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize(v):
    if norm(v) == 0:
        traceback.print_stack()
    v /= norm(v)
    return v


def normalized(v):
    if v is None:
        return None
    return normalize(copy.deepcopy(v))


def proj_orth(v1, v2, normalize_res=False, eps=1e-5):
    v2_norm = norm(v2)
    if v2_norm < eps:
        return v1

    v2n = v2 / v2_norm
    v1 = v1 - np.dot(v1, v2n) * v2n
    if normalize_res:
        return normalized(v1)
    else:
        return v1


def axes_to_mat(axis_x, axis_z, dominant_axis="z"):
    if dominant_axis == "z":
        axis_x = proj_orth(axis_x, axis_z)
    elif dominant_axis == "x":
        axis_z = proj_orth(axis_z, axis_x)
    elif dominant_axis is None:
        pass
    else:
        raise RuntimeError("Unrecognized dominant_axis: %s" % dominant_axis)

    axis_x = axis_x / norm(axis_x)
    axis_z = axis_z / norm(axis_z)
    axis_y = np.cross(axis_z, axis_x)

    R = np.zeros((3, 3))
    R[0:3, 0] = axis_x
    R[0:3, 1] = axis_y
    R[0:3, 2] = axis_z

    return R


# Projects T to align with the provided direction vector v.
def proj_to_align(R, v):
    max_entry = max(
        enumerate([np.abs(np.dot(R[0:3, i], v)) for i in range(3)]),
        key=lambda entry: entry[1],
    )
    return axes_to_mat(R[0:3, (max_entry[0] + 1) % 3], v)


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
        self.__sensors = []
        self.__waypoints = []
        self.__curr_waypoint_id = 0

        self._prim_path = path
        self._prim_name = name
        self._full_prim_path = f"{self._prim_path}/{self._prim_name}"
        self._prim = None
        self._dc = None
        self._rb = None
        self.start_time = 0

        self.velocity = 10
        self.sample_rate = 10
        self._waypoints_parent = None
        self.time = 0
        self.sample_time_counter = 0
        self._o = "[SensorRig] "

    def reset(self):
        self.time = 0

    def ray_cast(self, origin):
        # pos, _ = self.get_pos_rot()

        # print(pos)
        hit = get_physx_scene_query_interface().raycast_closest(
            origin, [0, 0, -1], 100000.0
        )

        if hit["hit"]:
            distance = hit["distance"]

            print(hit["position"][2])
            return hit["position"][2]
        return 0

    def create_rig_from_file(self, path, stage, world):
        self._world = world
        pos, ori = self.load_sensors_from_file(path, stage)
        print(
            f"{self._o} Creating sensor righ with initial position of: {pos} and rot of {ori}"
        )
        position = np.array([pos[0], pos[1], pos[2]])
        orientation = np.array([ori[0], ori[1], ori[2], ori[3]])
        self._prim = XFormPrim(
            name=self._prim_name,
            prim_path=self._full_prim_path,
            position=position / get_stage_units(),
            orientation=orientation,
        )
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

    def hide_waypoints_an_rig(self):
        pass

    def create_rig(self, position, orientation, stage):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
        self._prim = XFormPrim(
            name=self._prim_name,
            prim_path=self._full_prim_path,
            position=position / get_stage_units(),
            orientation=orientation,
        )
        self.actual_prim = stage.GetPrimAtPath(self._full_prim_path)

        self.orient_val = self.actual_prim.GetAttribute("xformOp:orient")
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

    def apply_veloc(self, veloc, ang_veloc):
        # print('applying ', veloc)
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        self._dc.set_rigid_body_linear_velocity(self._rb, veloc)

        x = ang_veloc
        self._dc.set_rigid_body_angular_velocity(self._rb, x)

    def setup_sensor_output_path(self, path):
        print(path)

        instance_mapping = helpers.get_instance_mappings()
        print(" ================== initiating mapping")
        print(instance_mapping)
        np.save(f"{path}/mapping.npy", instance_mapping, allow_pickle=True)
        pathlib.Path(path).mkdir(parents=True, exist_ok=True)
        pathlib.Path(path + "/timestamps.csv")
        self._time_stamp_file = open(path + "/timestamps.csv", "a")
        for sensor in self.__sensors:
            sensor.init_output_folder(path)

    def add_sensor_to_rig(self, sensor):
        self.__sensors.append(sensor)
        self.__sensors[-1].init_sensor(self._full_prim_path)

    def sample_sensors(self, n):
        # print("sampling sensors")
        self.time += n
        self.sample_time_counter += n
        # print(self.time)
        # log timestep
        # Sample all sensors
        if self.sample_time_counter >= (1 / self.sample_rate):
            # print("sampling at ", self.time)
            for sensor in self.__sensors:
                # print(sensor)
                sensor.sample_sensor()
            self._time_stamp_file.write(f"{str(self.time)}\n")
            self.sample_time_counter = 0

    def sample_sensors_return(self):
        sensor_output = []
        for sensor in self.__sensors:
            sensor_output.append(sensor.sample_sensor())

        return sensor_output

    def sample_all_sensors(self):
        for sensor in self.__sensors:
            sensor.sample_sensor()

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
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            if "_waypoints_" in prim_ref_name:
                self._waypoints_parent = prim_ref
                for i in range(len(prim_ref.GetChildren())):
                    prim_child = prim_ref.GetChildren()[i]
                    self.__waypoints.append(get_world_translation(prim_child))

        print(f"{self._o} SensorRig waypoints initialization complete:")
        print(self.__waypoints)

    def initialize_waypoints_preloaded(self, waypoints, parent_prim):
        self.__waypoints = []
        self.__waypoints = waypoints
        self._waypoints_parent = parent_prim
        print(f"{self._o} loaded waypoints from file ")
        for i in range(len(self.__waypoints)):
            origin = self.__waypoints[i]
            z = self.ray_cast(origin)
            z += 0.7
            self.__waypoints[i][2] = z
        print(f"{self._o} Synced waypoints to ground")

    def _waypoint_update(self, pos):
        print(f"{self._o} Waypoint {self.__curr_waypoint_id}/{len(self.__waypoints)}")
        # Get the goal position and convert it into the correct type
        # print("moving")
        goal_pos = self.__waypoints[self.__curr_waypoint_id]
        # goal_pos[2] = z_val
        goal_pos = Gf.Vec3d(goal_pos)
        ori_ = lookat_to_quatf(pos, goal_pos, Gf.Vec3d(0, 0, 1))

        rot_vec = quat_to_euler_angles(ori_)
        rot_float = 0.0

        # Calculate the diff vector
        move_vec = goal_pos - pos
        distance = np.linalg.norm(goal_pos - pos)
        move_vec = (move_vec / distance) * self.velocity
        goal_pos_arr = np.array([[goal_pos[0], goal_pos[1], 0]])
        pos_arr = np.array([[pos[0], pos[1], 0]])
        ori_now = self.orient_val.Get()
        rvg = rot_vec
        rvc = quat_to_euler_angles(ori_now)
        rot_ang = Gf.Vec3d(0, 0, rvg[2] - rvc[2])
        calc = rvg[2] - rvc[2]
        calc *= 57.2
        x_ = rvg[0] - rvc[0]
        y_ = rvg[1] - rvc[1]

        rot_float = Gf.Vec3d(0, 0, calc / 5.73)

        if distance < 0.5:
            self.__curr_waypoint_id += 1

            if self.__curr_waypoint_id >= len(self.__waypoints):
                self.__curr_waypoint_id = 0
                timeline = omni.timeline.get_timeline_interface()
                timeline.pause()

            return self._waypoint_update(pos)

        return move_vec, rot_vec, rot_float

    def move(self, time_step):
        # timeline = omni.timeline.get_timeline_interface()

        # timecode = (
        #     timeline.get_current_time() * timeline.get_time_codes_per_seconds()
        # )
        self.start_time += time_step
        if len(self.__waypoints) == 0:
            return

        # Retrieve the current position and orientation of the sensor rig
        current_pos, current_rot = self.get_pos_rot()
        current_pos = Gf.Vec3d(current_pos[0], current_pos[1], current_pos[2])

        # Load the correct waypoint, check if we should change to next one ..
        # and then calculate the required move vector.
        move_vec, rot_vec, rot_float = self._waypoint_update(current_pos)

        # Apply the required veloc
        self.apply_veloc(move_vec, rot_float)

    def load_sensors_from_file(self, file_path, stage):
        with open(file_path, "r+") as infile:
            print(f"{self._o} Loading sensor rig from file at {file_path}.")
            data = json.load(infile)
            # print(data)
            pos = data["POSITION"]
            ori = data["ORIENTATION"]
            self.velocity = data["VELOCITY"]
            self.sample_rate = data["SAMPLE_RATE"]

            self.create_rig(np.array(pos), np.asarray(ori), stage)
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

    def init_output_folder(self, path):
        # create any paths needed
        pathlib.Path(path).mkdir(parents=True, exist_ok=True)
        pathlib.Path(path + "/timestamps.csv")

        print(instance_mapping)
        self._time_stamp_file = open(path + "/timestamps.csv", "a")
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
"""
