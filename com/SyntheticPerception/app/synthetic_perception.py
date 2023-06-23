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
from .PCG import AreaMaskGenerator
from omni.isaac.core.objects import DynamicCuboid
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
from omni.isaac.core.utils.stage import add_reference_to_stage


class SyntheticPerception(BaseSample):
    """

    Main class
    """

    # pylint: disable=too-many-instance-attributes
    # Big class requires lots of attrs.
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
            "M": [0.0, 0.0, -1.0],
        }

    def _sub_keyboard_event(self, event, *args, **kwargs):
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                print(f"Here: {event.input.name}")
                print("calling apply veloc")
                self.sr.apply_veloc(
                    self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.sr.apply_veloc([0, 0, 0])
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


    def __add_semantics_to_all2(self, stage):
        "Add semantic information to all prims on stage based on parent xform"
        prim_class = self.__undefined_class_string
        completed_classes = []
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            len_of_prim = len(prim_ref_name.split("/"))
            for word in prim_ref_name.split("/"):

                if "class" in word and word not in completed_classes:

                    prim_class = word

                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        len_of_child = len(
                            str(prim_child.GetPrimPath()).split("/"))
                        print(len_of_prim, " : ", len_of_child)
                        if abs(len_of_prim-len_of_child) == 1:
                            print(prim_child)
                            self.add_semantic(prim_child, prim_class)

                    completed_classes.append(prim_class)

    def _get_translate(self, prim_path):
        "Gettgs the tranformation of a prim at a path"
        # prim = stage.GetPrimAtPath(prim_path)
        dc = _dynamic_control.acquire_dynamic_control_interface()

        object = dc.get_rigid_body(prim_path)
        object_pose = dc.get_rigid_body_pose(object)

        print("position:", object_pose.p)
        print("rotation:", object_pose.r)

    async def init_world(self):

        if World.instance() is None:
            self._world = World(**self._world_settings)
            await self._world.initialize_simulation_context_async()
            self.setup_scene()
        else:
            self._world = World.instance()
        await self._world.reset_async()
        await self._world.pause_async()
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()

        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface())
        self._world_settings = {
            "physics_dt": 1.0 / 60.0,
            "stage_units_in_meters": 1.0,
            "rendering_dt": 1.0 / 60.0,
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        # self.__add_semantics_to_all2(stage)

    def init_semantics_in_scene(self):
        self.__add_semantics_to_all2(self.stage)

    def init_sensor_and_semantics(self):
        "Initializes sensors and the replicator package"
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface()
        )

    def init_sensor_rig(self):
        "Initializes the sensor rig and adds individual sensors"
        print(" ============================================================== ")
        print("trying to load sensor rig")
        self.sr.create_rig(np.array([0, 5, 0]),
                           np.asarray([1, 1, 1, 1]), self.stage)
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
        pc, sem = self.__sensor.get_pc_and_semantic(
            save_path="/home/jon/Desktop/")

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
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )

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

    def sample_sensors(self):
        self.sr.sample_sensors()

    def temp_passthrough(self, srx):
        # un comment to enalbe wAYPOINT
        self.get_world().add_physics_callback("sim_step", callback_fn=srx.move)

    def spawn_asset(self, asset_path, class_name, prim_name, x, y, z):
        prim_path = "/World/"+"class_"+class_name + "/" + prim_name

        prim = self.add_asset_to_stage(asset_path, prim_name, prim_path, self._world.scene,

                                       # Using the current stage units which is in meters by default.
                                       position=np.array([x, y, 0]),
                                       # most arguments accept mainly numpy arrays.
                                       scale=np.array(
                                           [0.5015, 0.5015, 0.5015]),
                                       )

        omni.kit.commands.execute('TransformPrimSRTCommand',
                                  path=prim_path,  # f"/World/{p_name}",

                                  old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
                                  new_scale=Gf.Vec3f(.01, .01, .01),
                                  old_translation=Gf.Vec3f(x, y, z),
                                  new_translation=Gf.Vec3f(x, y, z),
                                  time_code=Usd.TimeCode(),
                                  had_transform_at_key=False)
        omni.kit.commands.execute('TransformPrimSRTCommand',
                                  path=prim_path,  # f"/World/{p_name}",

                                  old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
                                  new_scale=Gf.Vec3f(.01, .01, .01),
                                  old_translation=Gf.Vec3f(x, y, z),
                                  new_translation=Gf.Vec3f(x, y, z),
                                  time_code=Usd.TimeCode(),
                                  had_transform_at_key=False)

    def test_areagen(self):
        self.test_areagen2()
        return
        print("running test for area generations")
        n1, n2 = AreaMaskGenerator.test_func()
        # world = self.get_world()
        print(len(n1), len(n2), " this many cubes")
        for i, n in enumerate(n1):
            x, y = n
            x = float(x)
            y = float(y)
            z = float(0)
            p_name = f"tree_{i}"
            self.spawn_asset(
                "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Trees\\American_Beech.usd", "treeN", p_name,  x, y, z)
        for i, n in enumerate(n2):

            x, y = n
            x = float(x)
            y = float(y)
            z = float(0)
            p_name = f"tree_pine_{i}"
            self.spawn_asset(
                "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Trees\\White_pine.usd", "treeP", p_name,  x, y, z)

        #     fancy_cube = self._world.scene.add(
        #         DynamicCuboid(
        #             prim_path=f"/World/random_cube_{i}", # The prim path of the cube in the USD stage
        #             name=f"fancy_cube_{i}", # The unique name used to retrieve the object from the scene later on
        #             position=np.array([x, y, 0]), # Using the current stage units which is in meters by default.
        #             scale=np.array([.1,.1,.1]), # most arguments accept mainly numpy arrays.
        #             color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        #         ))

    def spawn_loop(self, path, class_name, p_name, coll):
        for i, n in enumerate(coll):
            x, y = n
            x = float(x)
            y = float(y)
            z = float(0)

            _p_name = f"{p_name}_{i}"
            self.spawn_asset(path, class_name, _p_name,  x, y, z)

    def test_areagen2(self):
        tree1, tree2, rocks, rocks2 = AreaMaskGenerator.test_world()
        tree1_path = "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Trees\\American_Beech.usd"
        tree2_path = "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Trees\\Black_Oak.usd"
        rocks_path = "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Shrub\\Fountain_Grass_Tall.usd"
        rocks2_path = "C:\\Users\\jonem\\Documents\\Isaac\\content\\ov-vegetation3dpack-01-100.1.0\\Plant_Tropical\\Jungle_Flame.usd"
        print("how many to generate")
        print(len(tree1), len(tree2), len(rocks), len(rocks2))
        self.spawn_loop(tree1_path, "tree1", "tree_1_", tree1)
        self.spawn_loop(tree2_path, "tree2", "tree_2_", tree2)
        self.spawn_loop(rocks_path, "shrub1", "shrub_1_", rocks)
        self.spawn_loop(rocks2_path, "shrub2", "shrub_2_", rocks2)

    def add_asset_to_stage(self, asset_path, prim_name, prim_path, scene, **kwargs):
        if scene.object_exists(prim_name):
            scene.remove_object(prim_name)

        if "scale" in kwargs.keys():
            scale_np = kwargs["scale"]
            scale = Gf.Vec3d(scale_np[0], scale_np[1], scale_np[2])
        else:
            scale = Gf.Vec3d(1., 1., 1.)
        if "orientation_euler" in kwargs.keys():
            ori_np = kwargs["orientation_euler"]
            ori = Gf.Vec3d(ori_np[0], ori_np[1], ori_np[2])
        else:
            ori = Gf.Vec3d(0., 0., 0.)
        pos = kwargs["position"]

        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        if "make_rigid" in kwargs.keys():
            if kwargs["make_rigid"] == False:
                return None
        if "semantic_class" in kwargs.keys():
            sc = kwargs["semantic_class"]
            del kwargs["semantic_class"]
        else:
            sc = None

    def add_asset_to_stage_archive(self, asset_path, prim_name, prim_path, scene, **kwargs):
        if scene.object_exists(prim_name):
            scene.remove_object(prim_name)
        if "2021" in VERSION:
            import omni.kit.commands
            from omni.usd import _usd, get_context
            from omni.isaac.core.simulation_context import SimulationContext
            from pxr import Usd, Gf

            if get_prim_at_path(prim_path) is not None:
                omni.kit.commands.execute('DeletePrims',
                                          paths=[prim_path])

            context = get_context()
            # prim_path = '/_4042_750_mL_Wine_Bottle_r_v1_L3'
            omni.kit.commands.execute('CreateReferenceCommand',
                                      usd_context=context,
                                      path_to=prim_path,
                                      asset_path=asset_path,
                                      instanceable=False)

            if "scale" in kwargs.keys():
                scale_np = kwargs["scale"]
                scale = Gf.Vec3d(scale_np[0], scale_np[1], scale_np[2])
            else:
                scale = Gf.Vec3d(1., 1., 1.)

            if "orientation_euler" in kwargs.keys():
                ori_np = kwargs["orientation_euler"]
                ori = Gf.Vec3d(ori_np[0], ori_np[1], ori_np[2])
            else:
                ori = Gf.Vec3d(0., 0., 0.)

            pos = kwargs["position"]
            transform(prim_path, pos, ori, scale)
            if "semantic_class" in kwargs.keys():
                sc = kwargs["semantic_class"]
            else:
                sc = None
            prim = ColliderPrim(prim_path=prim_path,
                                name=prim_name, semantic_class=sc)
            scene.add(prim)

        else:
            from pxr import Usd, Gf
            if "scale" in kwargs.keys():
                scale_np = kwargs["scale"]
                scale = Gf.Vec3d(scale_np[0], scale_np[1], scale_np[2])
            else:
                scale = Gf.Vec3d(1., 1., 1.)
            if "orientation_euler" in kwargs.keys():
                ori_np = kwargs["orientation_euler"]
                ori = Gf.Vec3d(ori_np[0], ori_np[1], ori_np[2])
            else:
                ori = Gf.Vec3d(0., 0., 0.)
            pos = kwargs["position"]

            add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
            if "make_rigid" in kwargs.keys():
                if kwargs["make_rigid"] == False:
                    return None
            if "semantic_class" in kwargs.keys():
                sc = kwargs["semantic_class"]
                del kwargs["semantic_class"]
            else:
                sc = None


"""


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
"""
