import asyncio
import random

import omni
import numpy as np
from .AreaMaskGenerator import generate_world_from_file

from omni.isaac.core.utils.semantics import get_semantics
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube

from pxr import Usd, Gf
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)

from pxr import UsdShade, Sdf



class WorldManager:
    def __init__(self) -> None:
        self.__undefined_class_string = "NAN"
        self.occupancy = []
        self._o = "[World generator] "

    def add_semantic(self, p, prim_class):
        """Adds semantic to prim"""
        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if 'Semantics' not in sem_dict:
            # print(
            #     'adding semantics and collider to ',
            #     p.GetPrimPath(),
            #     ' of class ',
            #     prim_class,
            # )
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

                    # self.add_semantic(prim_ref, prim_class)
                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        len_of_child = len(
                            str(prim_child.GetPrimPath()).split('/')
                        )
                        # print(len_of_prim, ' : ', len_of_child)
                        if abs(len_of_prim - len_of_child) == 1:
                            # print(prim_child)
                            self.add_semantic(prim_child, prim_class)

                    completed_classes.append(prim_class)
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
        orign_p_name = "",
        override=False,
        rot = (0,0,0),
    ):

        prim_path = '/World/' + 'class_' + class_name + '/' + prim_name

        # if not override:
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
        
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        # prim.GetReferences().AddReference(assetPath=asset_path, primPath=prim_path)
        prim.SetInstanceable(True)

        collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
        sem = Semantics.SemanticsAPI.Apply(prim, 'Semantics')
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set('class')
        sem.GetSemanticDataAttr().Set(class_name)


        # omni.kit.commands.execute('CopyPrim',
        #     path_from=orign_p_name,
        #     path_to=prim_path,
        #     duplicate_layers=False,
        #     combine_layers=False,
        #     exclusive_select=False,
        #     flatten_references=False,
        #     copy_to_introducing_layer=False)
        # here we want to modify the scale
        low_lim = scale - object_scale_delta
        high_lim = scale + object_scale_delta
        scale = random.uniform(low_lim, high_lim) #/ 100

        random_rotation = 0
        if allow_rot:
            random_rotation = random.uniform(0, 360)


        # omni.kit.commands.execute('CreatePayloadCommand',
        #     usd_context=omni.usd.get_context(),
        #     path_to=Sdf.Path(prim_path),
        #     asset_path=asset_path,
        #     instanceable=True)
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
            override=False
            # if i == 1:
            #
            #     prim_path = '/World/' + 'class_' + class_name + '/' + p_name 
            #
            #     add_reference_to_stage(usd_path=path, prim_path=prim_path)
            #
            #     override=True
        
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
            z = float(height_map[int(y_ind/10)][int(x_ind/10)])# / mesh_height_modifier   # was abs

            cc =(int(y_ind/10),int(x_ind/10) )
            ind = np.ravel_multi_index(cc, (len(height_map), len(height_map)))
            # print(np.asarray(self.t_normals))
            poss_rot = np.asarray(self.t_normals)[ind]
            # print("triangle normals")
            # print(poss_rot)
            # second one is iterated fasted
            if self.occupancy[int(y_ind/10)][int(x_ind/10)] != 0:
                # print("skipping oj spawn")
                continue

            self.occupancy[int(y_ind/10)][int(x_ind/10)]= 1 
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
                override = override,
                orign_p_name = p_name,
                rot = poss_rot
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

            prim_p = f'/World/t/class_{mat_name}'
            # prim_p = f'/World/t/terrain{key}'

            stage = omni.usd.get_context().get_stage()
            scale = 1#0.01
            # X SCALE SHOULD BE NEGATIVE TO FLIP IT CORRECTLY
            random_rotation = 0.0
            x, y, z = 0, 0, 0
            add_reference_to_stage(usd_path=mesh_path, prim_path=prim_p)
            self.create_material_and_bind(
                mat_name, mat_path, prim_p, scale, stage
            )
            prim=stage.GetPrimAtPath(prim_p)
            collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
            sem = Semantics.SemanticsAPI.Apply(prim, 'Semantics')
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set('class')
            sem.GetSemanticDataAttr().Set(mat_name)

        scale = 1#0.1
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

    def spawn_all(self, obs_to_spawn, object_dict, height_map, normals):
        self.t_normals = normals
        length = len(obs_to_spawn)
        counter = 1
        for key in obs_to_spawn:

            obj = object_dict[key]
            path = object_dict[key].usd_path
            print(f"{self._o} Spawning {len(obs_to_spawn[key])} of {path}. {counter} / {length}")
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
            print("spawned, now we wait till stage loads")
            update_stage()
            # print("some time should have passed")
            # return
            counter += 1

    def generate_world_generator(self, obj_path, world_path):


        print("Tring to generator worldf rom file")
        (
            obs_to_spawn,
            object_dict,
            terrain_info,
            meshGen,
        ) =generate_world_from_file(obj_path, world_path)
        height_map = meshGen._points2
        self.occupancy = np.zeros((len(height_map),len(height_map)))
        self.create_terrains(terrain_info)
        meshGen.clean_up_files()

        return obs_to_spawn, object_dict, height_map, meshGen.normals


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

    def create_world(self,  world_path, obj_path):

        (
            obs_to_spawn,
            object_dict,
            height_map,
            normals
        ) =self.generate_world_generator(
            world_path, obj_path
        )

        self.spawn_all(obs_to_spawn, object_dict, height_map, normals)
        update_stage()
        stage = omni.usd.get_context().get_stage()
        self.__add_semantics_to_all2(stage)
