from omni.syntheticdata.scripts.sensors import enable_sensors
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from omni.isaac.range_sensor import _range_sensor
import omni
import asyncio
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from time import sleep
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
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

from pxr import Sdf


class SensorRig:
    def __init__(self, name, path) -> None:
        self.__sensors = []
        self.__num_sensors = []
        self.__waypoints = []
        self.__curr_waypoint_id = 0
        self.__current_transform = None

        self._prim_path = path
        self._prim_name = name
        self._full_prim_path = f"{self._prim_path}/{self._prim_name}"
        self._prim = None
        self._dc = None
        self._rb = None

    def create_rig(self, position, orientation, stage):
        self._dc = _dynamic_control.acquire_dynamic_control_interface()
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
    def apply_veloc(self):

        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        self._dc.set_rigid_body_linear_velocity(self._rb, np.array([5,0,0]))
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
        #save position and rotation of sensor rig as a whole, and for each sensor.
        pos, rot = self.get_pos_rot()

    def get_pos_rot(self):
        self._rb = self._dc.get_rigid_body(self._full_prim_path)
        # self._dc.wake_up_rigid_body(self._rb)
        object_pose = self._dc.get_rigid_body_pose(self._rb)
        return object_pose.p, object_pose.r

    def initialize_waypoints(self, waypoint_parent):
        # iter over the stage and get all the waypoints
        # go through each child and save its tranform details to the waypoints list.
        pass

    def __get_target_rot(self, waypoint_id):
        pass

    def __advance_waypoint(self, waypoint_id):
        pass

    def move(self):
        # updates curr waypoint
        self.__advance_waypoint(self.__curr_waypoint_id)

        self.__get_target_rot(self.__curr_waypoint_id)


class DepthCamera:
    def __init__(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        image_size=(512, 512),
        attach=True,
        parent="/World/DepthCamera",
        name="DepthCamera",
    ) -> None:
        self.__rgb_annot: Annotator
        self.__save_path = ""
        self.__pos = position
        self.__rot = rotation
        self.__image_size = image_size
        self.__attach = attach
        self.__name = name
        # self.__cam = rep.create.camera(position=position, parent=parent, name=name)
        # self.__rp: og.Node = rep.create.render_product(self.__cam, image_size)
        # if attach:
        #     self.__init_annotators()
        #     self.__attach_annotoators()

    def init_sensor(self,parent):

        self.__cam = rep.create.camera(position=self.__pos, parent=parent, name=self.__name)
        self.__rp: og.Node = rep.create.render_product(self.__cam, self.__image_size)
        if self.__attach:
            self.__init_annotators()
            self.__attach_annotoators()

    def construct_pc(self, rgb_image, depth_image):
        pass

    def __init_annotators(self):
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        # self.pc_annot = rep.AnnotatorRegistry.get_annotator("pointcloud")
        self.sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

    def __attach_annotoators(self):
        self.depth_annot.attach(self.__rp)
        self.rgb_annot.attach(self.__rp)
        self.sem_annot.attach(self.__rp)
        # self.pc_annot.attach(self.rp)

    def __detatch_annototators(self):
        self.depth_annot.detach(self.__rp)
        self.rgb_annot.detach(self.__rp)
        self.sem_annot.detach(self.__rp)
        # self.pc_annot.dettach(self.rp)

    async def sample_sensor(self):
        await rep.orchestrator.step_async()

        rgb_data = self.rgb_annot.get_data()
        np.save("/home/jon/Documents/temp/image.npy", rgb_data)

        depth_data = self.depth_annot.get_data()
        np.save("/home/jon/Documents/temp/depth.npy", depth_data)

        sem_data = self.sem_annot.get_data()
        np.save("/home/jon/Documents/temp/sem.npy", sem_data)
        return


class Lidar:
    def __init__(
        self,
        path="/Lidar1",
        parent="/World",
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
        self.__path = "/" +path
        self.__min_range=min_range
        self.__max_range=max_range
        self.__draw_points=draw_points
        self.__draw_lines=draw_lines
        self.__horizontal_fov=horizontal_fov
        self.__vertical_fov=vertical_fov
        self.__horizontal_resolution=horizontal_resolution
        self.__vertical_resolution=vertical_resolution
        self.__rotation_rate=rotation_rate
        self.__high_lod=high_lod
        self.__yaw_offset=yaw_offset
        self.__enable_semantics=enable_semantics
        self.__origin_pos=origin_pos
        # result, self.__lidar_prim = omni.kit.commands.execute(
        #     "RangeSensorCreateLidar",
        #     path=path,
        #     parent=parent,
        #     min_range=min_range,
        #     max_range=max_range,
        #     draw_points=draw_points,
        #     draw_lines=draw_lines,
        #     horizontal_fov=horizontal_fov,
        #     vertical_fov=vertical_fov,
        #     horizontal_resolution=horizontal_resolution,
        #     vertical_resolution=vertical_resolution,
        #     rotation_rate=rotation_rate,
        #     high_lod=high_lod,
        #     yaw_offset=yaw_offset,
        #     enable_semantics=enable_semantics,
        # )
        # UsdGeom.XformCommonAPI(self.__lidar_prim).SetTranslate(origin_pos)
        # self.__lidar_path = parent + "/" + path
        # print(f"lidar path should be {self.__lidar_path}")
    def init_sensor(self,parent):
        print(f"init the lidar {parent}")
        # self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
        result, self.__lidar_prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
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
        UsdGeom.XformCommonAPI(self.__lidar_prim).SetTranslate(self.__origin_pos)
        self.__lidar_path = parent + "/" + self.__path
        print(f"lidar path should be {self.__lidar_path}")
        self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

    # def sample_sensor(self):
    #     self.get_pc_and_semantic()
    async def sample_sensor(self):
        self.get_pc_and_semantic()

    def get_pc_and_semantic(self, save_path="/home/jon/Documents/temp/a"):
        pointcloud = self.__lidarInterface.get_point_cloud_data(self.__lidar_path)
        semantics = self.__lidarInterface.get_semantic_data(self.__lidar_path)
        lidar_position = self.__get_position()
        # pointcloud, semantics = self.__clear_max_lidar_points(
        #     pointcloud, semantics, lidar_position, self.__max_range
        # )

        if save_path is not None:
            np.save(
                f"{save_path}_pc.npy",
                np.array(pointcloud),
            )
            np.save(
                f"{save_path}_sem.npy",
                np.array(semantics),
            )
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
