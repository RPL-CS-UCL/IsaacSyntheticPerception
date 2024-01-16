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

from pxr import Usd, Gf

# from .PCG import AreaMaskGenerator
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.semantics import get_semantics
import omni
import omni.kit.commands
import omni.timeline
from omni.isaac.core.utils.prims import get_prim_at_path  # , get_prim_property
import omni.kit.viewport
from pxr import Usd, Gf, UsdGeom
import numpy as np
from .sensors import Lidar, DepthCamera, SensorRig
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core import World

import omni.appwindow  # Contains handle to keyboard
import carb
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)

from pxr import UsdShade, Sdf

import omni.kit.commands

# from omni import usd._usd
from pxr import Sdf
import omni.physx
from omni.physx import get_physx_scene_query_interface


class SyntheticPerception(BaseSample):
    """

    Main class
    """

    def __init__(self) -> None:
        super().__init__()
        self.__created_objs = []
        self.save_count = 0
        self.obstacles = []
        self.__undefined_class_string = "undef"

        self.sr = SensorRig("SensorRig", "/World")

        self._event_flag = False
        self._o = "[SyntheticPerception] "

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
            "M": [0.0, 0.0, -1.0],
        }

    def _sub_keyboard_event(self, event, *args, **kwargs):
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self.sr.apply_veloc(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.sr.apply_veloc([0, 0, 0])
            # print(self._input_keyboard_mapping[event.input.name])
        return True

    def force_reload(self):
        self._world.initialize_physics()
        self.setup_scene()

    async def _on_load_world_async(self):
        await omni.kit.app.get_app().next_update_async()

        self._world = World(**self._world_settings)

        await self._world.initialize_simulation_context_async()
        # await self._world.reset_async()

    async def load_sample(self) -> None:
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

    async def init_world(self) -> None:
        if World.instance() is None:
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        await self._world.reset_async()
        await self._world.pause_async()
        self.world_cleanup()

        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = omni.timeline.get_timeline_interface()
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        print("The world is initialized.")

    async def setup_post_load(self):
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

    def remove_all_objects(self):
        for i in reversed(range(len(self.__created_objs))):
            try:
                self._world.scene.remove_object(self.__created_objs[i])
            except:
                pass  # already deleted from world
            del self.__created_objs[i]

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        if world.physics_callback_exists("sim_timestep"):
            world.remove_physics_callback("sim_timestep")
        stage = omni.usd.get_context().get_stage()
        print("Pre rest setup over")
        # self.sr.initialize_waypoints('', stage)

    def world_cleanup(self):
        self.remove_all_objects()

    def init_semantics_in_scene(self):
        self.stage = omni.usd.get_context().get_stage()
        print(
            f"{self._o} Adding semantics to scene. Please wait until complete... ... ... "
        )
        self.__add_semantics_to_all2(self.stage)

        print(f"{self._o} All semantics added to scene. Complete.")

    def init_sensor_and_semantics(self):
        """Initializes sensors and the replicator package"""
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = omni.timeline.get_timeline_interface()

    def add_semantic(self, p, prim_class):
        """Adds semantic to prim"""
        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if "Semantics" not in sem_dict:
            sem = Semantics.SemanticsAPI.Apply(p, "Semantics")
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set("class")
            sem.GetSemanticDataAttr().Set(prim_class)

    def __add_semantics_to_all2(self, stage):
        """Add semantic information to all prims on stage based on parent xform"""
        prim_class = self.__undefined_class_string
        completed_classes = []
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            len_of_prim = len(prim_ref_name.split("/"))
            for word in prim_ref_name.split("/"):
                if "class" in word and word not in completed_classes:
                    prim_class = word

                    # self.add_semantic(prim_ref, prim_class)
                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        len_of_child = len(str(prim_child.GetPrimPath()).split("/"))
                        # print(len_of_prim, ' : ', len_of_child)
                        if abs(len_of_prim - len_of_child) == 1:
                            # print(prim_child)
                            self.add_semantic(prim_child, prim_class)

                    completed_classes.append(prim_class)

    def init_sensor_rig(self):
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        """Initializes the sensor rig and adds individual sensors"""
        self.sr.create_rig(np.array([0, 5, 0]), np.asarray([1, 1, 1, 1]), self.stage)
        # self.sr.add_depth_camera_to_rig( (0, 0, 0), (0, 0, 0), (512, 512), True,"DepthCamera")
        self.sr.add_sensor_to_rig(DepthCamera(name="depthcam2"))
        self.sr.add_sensor_to_rig(Lidar(path="coolLidar"))

    def init_sensor_rig_from_file(self, path, out_path):
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.sr.create_rig_from_file(path, self.stage, self._world)
        self.sr.setup_sensor_output_path(out_path)

    def sample_sensors(self):
        self.sr.sample_sensors()

    def attach_sensor_waypoint_callback(self, srx):
        self._world.add_physics_callback("sim_step", callback_fn=srx.move)

    def attach_sensor_sample_callback(self):
        # un comment to enalbe wAYPOINT
        self.get_world().add_physics_callback(
            "sim_sample_step", callback_fn=self.sr.sample_sensors
        )
