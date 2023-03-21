# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
try:
    import omni.isaac.version as v

    VERSION = v.get_version()[0]
except:
    VERSION = "2021"
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.semantics import get_semantics
import omni
import asyncio
import omni.kit.commands
import omni.timeline
from omni.isaac.core.utils.prims import get_prim_at_path  # , get_prim_property
import omni.kit.viewport
from time import sleep
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
import omni.replicator.core as rep
from .sensors import Lidar, DepthCamera, SensorRig
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.synthetic_utils import SyntheticDataHelper
from .syntehtic_data_watch import SyntheticDataWatch, SyntheticDataWatch_V2
import omni.physx as _physx
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core import World


class SyntheticPerception(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.__created_objs = []
        self.save_count = 0
        self.pri = None
        self.obstacles = []
        self.__undefined_class_string = "undef"
        self.__sensor = None
        self._rp = None
        self._depth_camera = None
        self._rc = None

        self.sr = SensorRig("SensorRig", "/World")

    async def load_sample(self):
        """Function called when clicking load buttton
        """
        if World.instance() is None:
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        await self._world.reset_async()
        await self._world.pause_async()
        await self.setup_post_load()

    def setup_scene(self):
        self.world = self.get_world()

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        if world.physics_callback_exists("sim_timestep"):
            world.remove_physics_callback("sim_timestep")
        stage = omni.usd.get_context().get_stage()
        self.sr.initialize_waypoints("",stage)

    # async def setup_post_load(self):
    #     self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}

    def world_cleanup(self):
        self.remove_all_objects()
        return

    def add_semantic(self, p, prim_class):
        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if "Semantics" not in sem_dict:
            print(
                "adding semantics and collider to ",
                p.GetPrimPath(),
                " of class ",
                prim_class,
            )
            sem = Semantics.SemanticsAPI.Apply(p, "Semantics")
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set("class")
            sem.GetSemanticDataAttr().Set(prim_class)

    def __add_semantics_to_all(self, stage):
        prim_class = self.__undefined_class_string
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            for word in prim_ref_name.split("/"):
                if "class" in word:
                    prim_class = word
            if prim_ref.GetPrimTypeInfo().GetTypeName() == "Mesh":
                print("adding straight to mesh")
                self.add_semantic(prim_ref, prim_class)

            for i in range(len(prim_ref.GetChildren())):
                prim_child = prim_ref.GetChildren()[i]
                if prim_child.GetPrimTypeInfo().GetTypeName() == "Mesh":
                    p = stage.GetPrimAtPath(prim_child.GetPrimPath())
                    for word in str(prim_child.GetPrimPath()).split("/"):
                        if "class" in word:
                            prim_class = word
                    self.add_semantic(p, prim_class)

    def _get_translate(self, prim_path):
        # prim = stage.GetPrimAtPath(prim_path)
        dc = _dynamic_control.acquire_dynamic_control_interface()

        object = dc.get_rigid_body(prim_path)
        object_pose = dc.get_rigid_body_pose(object)

        print("position:", object_pose.p)
        print("rotation:", object_pose.r)

    def init_sensor_and_semantics(self):
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface()
        )  # Used to interact with simulation

        # self._depth_camera = DepthCamera()
        self._editor_event = _physx.get_physx_interface().subscribe_physics_step_events(
            self._controller_update
        )
        # prim = self.stage.GetPrimAtPath("/World/SensorOrigin")
        # xform = UsdGeom.Xformable(prim)
        # transform = prim.GetAttriute('xformOp:transform')
    def init_sensor_rig(self):
        print(" ============================================================== ")
        print("trying to load sensor rig")
        self.sr.create_rig(np.array([0, 5, 0]), np.asarray([1, 1, 1, 1]), self.stage)
        # self.sr.add_depth_camera_to_rig( (0, 0, 0), (0, 0, 0), (512, 512), True,"DepthCamera")
        self.sr.add_sensor_to_rig(DepthCamera(name="depthcam2"))
        self.sr.add_sensor_to_rig(Lidar(path="coolLidar"))
        # self.sr.add_lidar_to_rig("Lidar", (0,0,0))

        # self.init_camera()

        return

    def _controller_update(self, step):
        pass

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

    async def save_lidar_data(self):
        # lidar_att = pri.GetAttribute("enabled")
        # lidar_att.Set(1)
        # lidar_att = pri.GetAttribute("enableSemantics")
        # lidar_att.Set(1)
        # await asyncio.sleep(1.0)
        # self.timeline.pause()
        pc, sem = self.__sensor.get_pc_and_semantic(save_path="/home/jon/Desktop/")

    async def setup_post_load(self):
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

        self.init_sensor_and_semantics()
        self.init_sensor_rig()
        # stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        # omni.kit.commands.execute(
        #     "AddPhysicsSceneCommand",
        #     stage=stage,
        #     path="/World/PhysicsScene",
        #     context=omni.usd.get_context(),
        # )
        #
        # self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}

        # self.get_world().add_physics_callback("sim_step", self.rep_fn)

    def remove_all_objects(self):
        for i in reversed(range(len(self.__created_objs))):
            try:
                self._world.scene.remove_object(self.__created_objs[i])
            except:
                pass  # already deleted from world
            del self.__created_objs[i]

    async def final_fn(self):
        pos, rot = self.sr.get_pos_rot()
        print(pos, rot)

    import omni.replicator.core as rep

    def init_camera(self):
        self.cam = rep.create.camera(position=(0, 0, 0))
        self.rp = rep.create.render_product(self.cam, (512, 512))

        self.rgb_annot = self.rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach(self.rp)

        self.depth_annot = self.rep.AnnotatorRegistry.get_annotator(
            "distance_to_camera"
        )
        self.depth_annot.attach(self.rp)

        # self.pc_annot = self.rep.AnnotatorRegistry.get_annotator("pointcloud")
        # self.pc_annot.attach(self.rp)

        self.sem_annot = self.rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation"
        )
        self.sem_annot.attach(self.rp)
        #asdf

    def test(self, data):
        # asyncio.ensure_future(self._depth_camera.sample_sensor())
        print(data)
        # self.sr.sample_sensors()

        self.sr.apply_veloc()

    def sample_sensors(self):
        self.sr.sample_sensors()

    def tt(self, time_step):
        pass

    def temp_passthrough(self, srx):
        self.get_world().add_physics_callback("sim_step", callback_fn = srx.move)
