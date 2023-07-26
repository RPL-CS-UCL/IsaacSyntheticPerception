import asyncio
import random
import omni
import numpy as np
from PCG import AreaMaskGenerator

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

def create_material_and_bind(
     mat_name, mat_path, prim_path, scale, stage
):

    print("Trying to use scale ", type(scale))
    scale = float(scale)

    print("converted to scale ", type(scale))

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
def create_terrains(terrain_info):

    # create the parent

    omni.kit.commands.execute(
        'CreatePrimWithDefaultXform',
        prim_type='Xform',
        prim_path='/World/t',
        attributes={},
        select_new_prim=True,
    )

    scale = 1
    #0.01
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
        print("setting mat scale of ", prim_p, " to ", scale)
        # X SCALE SHOULD BE NEGATIVE TO FLIP IT CORRECTLY
        x, y, z = 0, 0, 0
        add_reference_to_stage(usd_path=mesh_path, prim_path=prim_p)
        create_material_and_bind(
            mat_name, mat_path, prim_p, scale, stage
        )

    random_rotation = 0.0
    x, y, z = 0, 0, 0
    # stage = self.usd_context.get_stage()
    scale = 1
    scale = float(scale)
    print("Setting scale to ", scale)

    omni.kit.commands.execute(
        'TransformPrimSRTCommand',
        path=f'/World/t',
        old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
        new_scale=Gf.Vec3f(scale, scale, scale),
        old_translation=Gf.Vec3f(x, y, z),
        new_translation=Gf.Vec3f(x, y, z),
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
        time_code=Usd.TimeCode(),
        had_transform_at_key=False,
    )

def spawn_asset(
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
        z = float(height_map[int(y_ind/10)][int(x_ind/10)])# / mesh_height_modifier   # was abs
        _p_name = f'{p_name}_{i}'
        spawn_asset(
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
def spawn_all_non(obs_to_spawn, object_dict, height_map):
    length = len(obs_to_spawn)
    counter = 1
    for key in obs_to_spawn:
        print("spawning ", key)
        obj = object_dict[key]
        path = object_dict[key].usd_path
        class_name = obj.class_name
        if class_name == '':
            class_name = obj.unique_id
        spawn_loop(
            path,
            class_name,
            f'{obj.unique_id}_',
            obs_to_spawn[key],
            height_map,
            scale=obj.object_scale,
            object_scale_delta=obj.object_scale_delta,
            allow_rot=obj.allow_y_rot,
        )
        update_stage()
        # print("some time should have passed")
        # return
        counter += 1
    print("finished spawning all")
async def spawn_all(obs_to_spawn, object_dict, height_map):
    length = len(obs_to_spawn)
    counter = 1
    for key in obs_to_spawn:

        obj = object_dict[key]
        path = object_dict[key].usd_path
        class_name = obj.class_name
        if class_name == '':
            class_name = obj.unique_id
        spawn_loop(
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
    print("finished spawning all")
def generate_world_generator( obj_path, world_path):


    (
        obs_to_spawn,
        object_dict,
        terrain_info,
        meshGen,
    ) = AreaMaskGenerator.generate_world_from_file(obj_path, world_path)
    height_map = meshGen._points2
    create_terrains(terrain_info)
    meshGen.clean_up_files()

    return obs_to_spawn, object_dict, height_map

def create_world(obj_path, world_path):

        (
            obs_to_spawn,
            object_dict,
            height_map,
        ) =generate_world_generator(
            world_path, obj_path
        )

        # asyncio.ensure_future(self.sample._on_load_world_async())
        # asyncio.ensure_future(
        #     spawn_all(obs_to_spawn, object_dict, height_map)
        # )

        spawn_all_non(obs_to_spawn, object_dict, height_map)

