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
import omni.physx as _physx
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core import World

import omni.replicator.core as rep
import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb
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


        self._event_flag = False

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.5, 0.0, 0.0],
            "UP": [1.5, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.5, 0.0, 0.0],
            "DOWN": [-1.5, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.0, 0.0],
            "RIGHT": [0.0, -1.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.0, 0.0],
            "LEFT": [0.0, 1.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],}
        
    


    def _sub_keyboard_event(self, event, *args, **kwargs):
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                print(f"Here: {event.input.name}")
                print("calling apply veloc")
                self.sr.apply_veloc(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.sr.apply_veloc([0,0,0])
                # print(self._input_keyboard_mapping[event.input.name])
        return True

    async def load_sample(self):
        """Function called when clicking load buttton"""
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
        self.sr.initialize_waypoints("", stage)

    # async def setup_post_load(self):
    #     self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}

    def world_cleanup(self):
        self.remove_all_objects()
        return

    def add_semantic(self, p, prim_class):
        "Adds semantic to prim"
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
        "Add semantic information to all prims on stage based on parent xform"
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
        "Gettgs the tranformation of a prim at a path"
        # prim = stage.GetPrimAtPath(prim_path)
        dc = _dynamic_control.acquire_dynamic_control_interface()

        object = dc.get_rigid_body(prim_path)
        object_pose = dc.get_rigid_body_pose(object)

        print("position:", object_pose.p)
        print("rotation:", object_pose.r)

    def init_sensor_and_semantics(self):
        "Initializes sensors and the replicator package"
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface()
        )  # Used to interact with simulation

        # self._depth_camera = DepthCamera()
        # self._editor_event = _physx.get_physx_interface().subscribe_physics_step_events(
        #     self._controller_update
        # )

    def init_sensor_rig(self):
        "Initializes the sensor rig and adds individual sensors"
        print(" ============================================================== ")
        print("trying to load sensor rig")
        self.sr.create_rig(np.array([0, 5, 0]), np.asarray([1, 1, 1, 1]), self.stage)
        # self.sr.add_depth_camera_to_rig( (0, 0, 0), (0, 0, 0), (512, 512), True,"DepthCamera")
        self.sr.add_sensor_to_rig(DepthCamera(name="depthcam2"))
        self.sr.add_sensor_to_rig(Lidar(path="coolLidar"))


    def __clear_max_lidar_points(self, pc, sem, lidar_pos, max_dist):
        "Clears the lidar dome points. - max range points so they do not display or get saved."
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
        pc, sem = self.__sensor.get_pc_and_semantic(save_path="/home/jon/Desktop/")

    async def setup_post_load(self):
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

        self.init_sensor_and_semantics()
        self.init_sensor_rig()
        print("Aquiring keyboard interface")
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

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


    # def init_camera(self):
    #     self.cam = rep.create.camera(position=(0, 0, 0))
    #     self.rp = rep.create.render_product(self.cam, (512, 512))
    #
    #     self.rgb_annot = self.rep.AnnotatorRegistry.get_annotator("rgb")
    #     self.rgb_annot.attach(self.rp)
    #
    #     self.depth_annot = self.rep.AnnotatorRegistry.get_annotator(
    #         "distance_to_camera"
    #     )
    #     self.depth_annot.attach(self.rp)
    #
    #     # self.pc_annot = self.rep.AnnotatorRegistry.get_annotator("pointcloud")
    #     # self.pc_annot.attach(self.rp)
    #
    #     self.sem_annot = self.rep.AnnotatorRegistry.get_annotator(
    #         "semantic_segmentation"
    #     )
    #     self.sem_annot.attach(self.rp)
        # asdf

    # def test(self, data):
    #     # asyncio.ensure_future(self._depth_camera.sample_sensor())
    #     print(data)
    #
    #     self.sr.apply_veloc()

    def sample_sensors(self):
        self.sr.sample_sensors()

    def tt(self, time_step):
        pass

    def temp_passthrough(self, srx):
        # un comment to enalbe wAYPOINT
        # self.get_world().add_physics_callback("sim_step", callback_fn=srx.move)
        pass
