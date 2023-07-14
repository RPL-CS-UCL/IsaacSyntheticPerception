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
    VERSION = '2021'
import random
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube

from pxr import Usd, Gf
from .PCG import AreaMaskGenerator
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.semantics import get_semantics
import omni
import asyncio
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
import numpy as np
import carb
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)

from pxr import UsdShade, Sdf


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
        self.obstacles = []
        self.__undefined_class_string = 'undef'

        self.sr = SensorRig('SensorRig', '/World')

        self._event_flag = False

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            'NUMPAD_8': [1.5, 0.0, 0.0],
            'UP': [1.5, 0.0, 0.0],
            # back command
            'NUMPAD_2': [-1.5, 0.0, 0.0],
            'DOWN': [-1.5, 0.0, 0.0],
            # left command
            'NUMPAD_6': [0.0, -1.0, 0.0],
            'RIGHT': [0.0, -1.0, 0.0],
            # right command
            'NUMPAD_4': [0.0, 1.0, 0.0],
            'LEFT': [0.0, 1.0, 0.0],
            # yaw command (positive)
            'NUMPAD_7': [0.0, 0.0, 1.0],
            'N': [0.0, 0.0, 1.0],
            # yaw command (negative)
            'NUMPAD_9': [0.0, 0.0, -1.0],
            'M': [0.0, 0.0, -1.0],
        }

    def _sub_keyboard_event(self, event, *args, **kwargs):
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self.sr.apply_veloc(
                    self._input_keyboard_mapping[event.input.name]
                )
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.sr.apply_veloc([0, 0, 0])
            # print(self._input_keyboard_mapping[event.input.name])
        return True

    def force_reload(self):


        self._world.initialize_physics()
        self.setup_scene()

    async def _on_load_world_async(self):

        await omni.kit.app.get_app().next_update_async()

        # print('[company.hello.world] company hello world startup')

        self._world = World(**self._world_settings)

        await self._world.initialize_simulation_context_async()
        # await self._world.reset_async()
        print("=== FINISHES RESETTING")

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
        print('loading finished')

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

        self.stage = (
            omni.usd.get_context().get_stage()
        )  # Used to access Geometry
        self.timeline = omni.timeline.get_timeline_interface()
        self._world_settings = {
            'physics_dt': 1.0 / 60.0,
            'stage_units_in_meters': 1.0,
            'rendering_dt': 1.0 / 60.0,
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        print('The world is initialized.')

    async def setup_post_load(self):
        self._world_settings = {
            'physics_dt': 1.0 / 60.0,
            'stage_units_in_meters': 1.0,
            'rendering_dt': 1.0 / 60.0,
        }

        # self.init_sensor_and_semantics()
        # self.init_sensor_rig()
        # print('Aquiring keyboard interface')
        # self._appwindow = omni.appwindow.get_default_app_window()
        # self._input = carb.input.acquire_input_interface()
        # self._keyboard = self._appwindow.get_keyboard()
        # self._sub_keyboard = self._input.subscribe_to_keyboard_events(
        #     self._keyboard, self._sub_keyboard_event
        # )

    def remove_all_objects(self):
        for i in reversed(range(len(self.__created_objs))):
            try:
                self._world.scene.remove_object(self.__created_objs[i])
            except:
                pass  # already deleted from world
            del self.__created_objs[i]

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists('sim_step'):
            world.remove_physics_callback('sim_step')
        if world.physics_callback_exists('sim_timestep'):
            world.remove_physics_callback('sim_timestep')
        stage = omni.usd.get_context().get_stage()
        print('Pre rest setup over')
        # self.sr.initialize_waypoints('', stage)

    def world_cleanup(self):
        self.remove_all_objects()

    def init_semantics_in_scene(self):
        self.__add_semantics_to_all2(self.stage)

    def init_sensor_and_semantics(self):
        """Initializes sensors and the replicator package"""
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = (
            omni.usd.get_context().get_stage()
        )  # Used to access Geometry
        self.timeline = omni.timeline.get_timeline_interface()

    def add_semantic(self, p, prim_class):
        """Adds semantic to prim"""
        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if 'Semantics' not in sem_dict:
            print(
                'adding semantics and collider to ',
                p.GetPrimPath(),
                ' of class ',
                prim_class,
            )
            sem = Semantics.SemanticsAPI.Apply(p, 'Semantics')
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set('class')
            sem.GetSemanticDataAttr().Set(prim_class)

    def __add_semantics_to_all2(self, stage):
        """Add semantic information to all prims on stage based on parent xform"""
        prim_class = self.__undefined_class_string
        completed_classes = []
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            len_of_prim = len(prim_ref_name.split('/'))
            for word in prim_ref_name.split('/'):

                if 'class' in word and word not in completed_classes:

                    prim_class = word

                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        len_of_child = len(
                            str(prim_child.GetPrimPath()).split('/')
                        )
                        print(len_of_prim, ' : ', len_of_child)
                        if abs(len_of_prim - len_of_child) == 1:
                            print(prim_child)
                            self.add_semantic(prim_child, prim_class)

                    completed_classes.append(prim_class)

    def init_sensor_rig(self):

        self.stage = (
            omni.usd.get_context().get_stage()
        )  # Used to access Geometry
        """Initializes the sensor rig and adds individual sensors"""
        print(
            ' ============================================================== '
        )
        print('trying to load sensor rig')
        self.sr.create_rig(
            np.array([0, 5, 0]), np.asarray([1, 1, 1, 1]), self.stage
        )
        # self.sr.add_depth_camera_to_rig( (0, 0, 0), (0, 0, 0), (512, 512), True,"DepthCamera")
        self.sr.add_sensor_to_rig(DepthCamera(name='depthcam2'))
        self.sr.add_sensor_to_rig(Lidar(path='coolLidar'))

    def init_sensor_rig_from_file(self, path,out_path):

        self.stage = (
            omni.usd.get_context().get_stage()
        )  # Used to access Geometry
        self.sr.create_rig_from_file(path, self.stage)
        self.sr.setup_sensor_output_path(out_path)


    def sample_sensors(self):
        self.sr.sample_sensors()

    def attach_sensor_waypoint_callback(self, srx):
        # print(self.get_world())
        # print(self._world)
        #
        # self._world = World.instance()
        # self.get_world().initialize()
        # un comment to enalbe wAYPOINT
        # self.get_world().add_physics_callback('sim_step', callback_fn=srx.move)
        self._world.add_physics_callback('sim_step', callback_fn=srx.move)

    def attach_sensor_sample_callback(self):
        # un comment to enalbe wAYPOINT
        self.get_world().add_physics_callback('sim_sample_step', callback_fn=self.sr.sample_sensors)

    def spawn_asset(
        self,
        asset_path,
        class_name,
        prim_name,
        x,
        y,
        z,
        scale,
        object_scale_delta,
        allow_rot,
    ):
        prim_path = '/World/' + 'class_' + class_name + '/' + prim_name

        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        # here we want to modify the scale
        low_lim = scale - object_scale_delta
        high_lim = scale + object_scale_delta
        scale = random.uniform(low_lim, high_lim) / 100

        random_rotation = 0
        if allow_rot:
            random_rotation = random.uniform(0, 360)
        # x = x *100
        # y = y *100

        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=prim_path,  # f"/World/{p_name}",
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            new_rotation_euler=Gf.Vec3f(0, 0, random_rotation),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=prim_path,  # f"/World/{p_name}",
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            new_rotation_euler=Gf.Vec3f(0, 0, random_rotation),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )

    def spawn_loop(
        self,
        path,
        class_name,
        p_name,
        coll,
        height_map,
        scale=1,
        object_scale_delta=0,
        allow_rot=True,
    ):
        for i, n in enumerate(coll):
            x, y = n
            x = float(x)
            y = float(y)
            mesh_scale = 10
            x_ind = x * mesh_scale 
            y_ind = y * mesh_scale
            mesh_height_modifier = 10
            # if x_ind >= 2560:
            #     print('x, overfilled', x_ind)
            #     x_ind = 2559
            # if y_ind >= 2560:
            #
            #     print('y, overfilled', y_ind)
            #     y_ind = 2559
            z = float(height_map[int(y_ind)][int(x_ind)]) / mesh_height_modifier   # was abs
            # second one is iterated fasted

            _p_name = f'{p_name}_{i}'
            self.spawn_asset(
                path,
                class_name,
                _p_name,
                x,
                y,
                z,
                scale,
                object_scale_delta,
                allow_rot,
            )

    def create_terrains(self, terrain_info):

        # create the parent

        omni.kit.commands.execute(
            'CreatePrimWithDefaultXform',
            prim_type='Xform',
            prim_path='/World/t',
            attributes={},
            select_new_prim=True,
        )

        for key in terrain_info:
            mesh_path = terrain_info[key].mesh_path
            scale = terrain_info[key].scale
            mat_path = terrain_info[key].material_path
            mat_name = mat_path.split('/')[-1]
            mat_name = mat_name.replace('.mdl', '')
            mesh_path = mesh_path.replace('.obj', '.usd')
            # spawn prim

            prim_p = f'/World/t/terrain{key}'

            stage = omni.usd.get_context().get_stage()
            scale = 0.01
            # X SCALE SHOULD BE NEGATIVE TO FLIP IT CORRECTLY
            random_rotation = 0.0
            x, y, z = 0, 0, 0
            add_reference_to_stage(usd_path=mesh_path, prim_path=prim_p)
            self.create_material_and_bind(
                mat_name, mat_path, prim_p, scale, stage
            )

        scale = 0.1
        random_rotation = 0.0
        x, y, z = 0, 0, 0
        # stage = self.usd_context.get_stage()

        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=f'/World/t',
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            # old_rotation_euler=Gf.Vec3f(-90, 0, 0),
            # old_rotation_order=Gf.Vec3i(0, 1, 2),
            # new_rotation_euler=Gf.Vec3f(-90, 0, -180),
            # new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=f'/World/t',
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_scale=Gf.Vec3f(scale, scale, scale),
            old_translation=Gf.Vec3f(x, y, z),
            new_translation=Gf.Vec3f(x, y, z),
            # old_rotation_euler=Gf.Vec3f(-90, 0, 0),
            # old_rotation_order=Gf.Vec3i(0, 1, 2),
            # new_rotation_euler=Gf.Vec3f(-90, 0, -180),
            # new_rotation_order=Gf.Vec3i(0, 1, 2),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False,
        )

    async def spawn_all(self, obs_to_spawn, object_dict, height_map):
        length = len(obs_to_spawn)
        counter = 1
        for key in obs_to_spawn:

            obj = object_dict[key]
            path = object_dict[key].usd_path

            print(
                'trying to spawn ',
                path,
                ' ',
                counter,
                ' / ',
                length,
                ' with ',
                len(obs_to_spawn[key]),
                ' objects',
            )
            class_name = obj.class_name
            if class_name == '':
                class_name = obj.unique_id
            self.spawn_loop(
                path,
                class_name,
                f'{obj.unique_id}_',
                obs_to_spawn[key],
                height_map,
                scale=obj.object_scale,
                object_scale_delta=obj.object_scale_delta,
                allow_rot=obj.allow_y_rot,
            )
            await update_stage_async()
            # print("some time should have passed")
            # return
            counter += 1
        print('AREA GENERATION FINISHED')

    def generate_world_generator(self, obj_path, world_path):


        (
            obs_to_spawn,
            object_dict,
            terrain_info,
            meshGen,
        ) = AreaMaskGenerator.generate_world_from_file(obj_path, world_path)
        height_map = meshGen._points2
        self.create_terrains(terrain_info)
        meshGen.clean_up_files()

        return obs_to_spawn, object_dict, height_map


    def create_material_and_bind(
        self, mat_name, mat_path, prim_path, scale, stage
    ):

        obj_prim = stage.GetPrimAtPath(prim_path)
        mtl_created_list = []

        omni.kit.commands.execute(
            'CreateAndBindMdlMaterialFromLibrary',
            mdl_name=mat_path,
            mtl_name=mat_name,
            mtl_created_list=mtl_created_list,
        )
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
        omni.usd.create_material_input(
            mtl_prim,
            'project_uvw',
            True,
            Sdf.ValueTypeNames.Bool,
        )

        omni.usd.create_material_input(
            mtl_prim,
            'texture_scale',
            Gf.Vec2f(scale, scale),
            Sdf.ValueTypeNames.Float2,
        )
        cube_mat_shade = UsdShade.Material(mtl_prim)

        UsdShade.MaterialBindingAPI(obj_prim).Bind(
            cube_mat_shade, UsdShade.Tokens.strongerThanDescendants
        )

    # def generate_world(self, obj_path, world_path):
    #     print('Starting world gen')
    #
    #     if World.instance() is None:
    #         self._world = World(**self._world_settings)
    #         self.setup_scene()
    #     else:
    #         self._world = World.instance()
    #     print('checking if world is activev')
    #     print(self._world)
    #     obs_to_spawn, object_dict = AreaMaskGenerator.generate_world_from_file(
    #         obj_path, world_path
    #     )
    #     length = len(obs_to_spawn)
    #     counter = 1
    #     for key in obs_to_spawn:
    #         obj = object_dict[key]
    #         path = object_dict[key].usd_path
    #
    #         # print("checking if world is activev")
    #         # print(self._world)
    #         print('trying to spawn ', path, ' ', counter, ' / ', length)
    #         class_name = obj.class_name
    #         if class_name == '':
    #             class_name = obj.unique_id
    #         self.spawn_loop(
    #             path,
    #             class_name,
    #             f'{obj.unique_id}_',
    #             obs_to_spawn[key],
    #             scale=obj.object_scale,
    #             object_scale_delta=obj.object_scale_delta,
    #             allow_rot=obj.allow_y_rot,
    #         )
    #         counter += 1
    #     print('AREA GENERATION FINISHED')

    # omni.kit.commands.execute('ChangeProperty',
    #     prop_path=Sdf.Path('/World/t.xformOp:orient'),
    #     value=Gf.Quatd(0.7071067811865476, Gf.Vec3d(-0.7071067811865476, 0.0, 0.0)),
    #     prev=Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0)),
    #     )
    #
    #
    # omni.kit.commands.execute('ChangeProperty',
    #     prop_path=Sdf.Path('/World/t.xformOp:orient'),
    #     value=Gf.Quatd(6.123233995736766e-17, Gf.Vec3d(-4.329780281177467e-17, -0.7071067811865476, -0.7071067811865476)),
    #     prev=Gf.Quatd(0.7071067811865476, Gf.Vec3d(-0.7071067811865476, 0.0, 0.0)),
    # )
    # self.spawn_asset(
    #     mesh_path,
    #     'terrain',
    #     f'terrainmesh_{key}',
    #     0,
    #     0,
    #     0,
    #     1,
    #     0,
    #     False,
    # )

    # self.create_material_and_bind(mat_name,mat_path,)
    # def test_spawn1(self):
    #
    #     # asyncio.ensure_future(self.init_world())
    #     mesh_path = 'C:\\Users\\jonem\\Documents\\Kit\\apps\\Isaac-Sim\\exts\\IsaacSyntheticPerception\\com\\SyntheticPerception\\app\\PCG\\mesh_0.usd'
    #
    #     add_reference_to_stage(usd_path=mesh_path, prim_path='/World/terrain')
    #     # self.spawn_asset(
    #     #     mesh_path,
    #     #     'terrain',
    #     #     f'terrainmesh',
    #     #     0,
    #     #     0,
    #     #     0,
    #     #     1,
    #     #     0,
    #     #     False,
    #     # )

    # def add_asset_to_stage(
    #     self, asset_path, prim_name, prim_path, scene, **kwargs
    # ):
    #     # print('adding asset to stage ', asset_path, prim_path)
    #
    #     add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
