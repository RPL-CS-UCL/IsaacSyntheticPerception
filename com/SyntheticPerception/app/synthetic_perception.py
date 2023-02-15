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
from .sensors import Lidar
from omni.isaac.core.utils.stage import get_stage_units
from .syntehtic_data_watch import SyntheticDataWatch,SyntheticDataWatch_V2
def transform(prim_path, pos,ori,scale):
    from pxr import Usd, Gf
    import omni.kit.commands
    if "2021" in VERSION:
        omni.kit.commands.execute('TransformPrimSRTCommand',
            path=prim_path,
            old_translation=Gf.Vec3f(0, 0, 0),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            new_rotation_euler=Gf.Vec3f(0, 0, 0),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False)
        
        omni.kit.commands.execute('TransformPrimSRTCommand',
            path=prim_path,
            old_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            new_rotation_euler=Gf.Vec3f(ori[0], ori[1], ori[2]),
            new_rotation_order=Gf.Vec3i(2, 1, 0),#Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(scale[0], scale[1], scale[2]),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False)

    else:
        print ("Not tested - it may be failing to translate")
        print(scale)
        omni.kit.commands.execute('TransformPrimSRTCommand',
            path=prim_path,
            old_translation=Gf.Vec3f(0, 0, 0),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            new_rotation_euler=Gf.Vec3f(0, 0, 0),
            new_rotation_order=Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False)
        
        omni.kit.commands.execute('TransformPrimSRTCommand',
            path=prim_path,
            old_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            old_rotation_euler=Gf.Vec3f(0, 0, 0),
            old_rotation_order=Gf.Vec3i(0, 1, 2),
            old_scale=Gf.Vec3f(1.0, 1.0, 1.0),
            new_translation=Gf.Vec3f(pos[0], pos[1], pos[2]),
            new_rotation_euler=Gf.Vec3f(ori[0], ori[1], ori[2]),
            new_rotation_order=Gf.Vec3i(2, 1, 0),#Gf.Vec3i(0, 1, 2),
            new_scale=Gf.Vec3f(scale[0], scale[1], scale[2]),
            time_code=Usd.TimeCode(),
            had_transform_at_key=False) 

    return
class SyntheticPerception(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.__created_objs = []
        self.save_count = 0
        self.pri = None
        self.obstacles = []
        self.__undefined_class_string = "undef"
        self.__sensor = None
        self.sd_watch = SyntheticDataWatch_V2("/home/jon/Documents/temp")

    def setup_scene(self):
        return

    async def setup_pre_reset(self):
        return

    def world_cleanup(self):
        self.remove_all_objects()
        return
    def make_camera_stand(self):
        prim_path = "/World/CameraStand"
        prim_name = "camera_stand"
        prim = XFormPrim(name=prim_name, prim_path=prim_path,
                        position =np.asarray([-0.77,1.883, 1.01])/get_stage_units(),
                        orientation = np.asarray([-0.18648,-0.10337,0.47367, 0.8545]),
                        )
        if "2021" in VERSION:
            transform (prim_path, np.array([-50.,240.,100.]),
                    np.array([12.0,5.,110.]), np.array([1.,1.,1.]))
        else:
            transform (prim_path, np.array([-50.,240.,100.])/100,
                     np.array([12.0,5.,110.]), np.array([1.,1.,1.]))

        omni.kit.commands.execute('CreatePrimWithDefaultXform',
            prim_path = "/World/CameraStand/Camera",
            prim_type = 'Camera',
            attributes={'focusDistance': 400, 'focalLength': 2.7,
                        'horizontalAperture': 2.682,
                        'verticalAperture': 1.509})


        prim_path = "/World/CameraStand_Closeup"
        prim_name = "camera_stand_closup"
        prim = XFormPrim(name=prim_name, prim_path=prim_path,
                        position =np.asarray([-0.77,1.883, 1.01])/get_stage_units(),
                        orientation = np.asarray([-0.18648,-0.10337,0.47367, 0.8545]),
                        )
        transform (prim_path, np.array([10.,120.,50.])/100,
                     np.array([12.0,5.,110.]), np.array([1.,1.,1.]))

        omni.kit.commands.execute('CreatePrimWithDefaultXform',
            prim_path = "/World/CameraStand_Closeup/CameraCloseup",
            prim_type = 'Camera',
            attributes={'focusDistance': 400, 'focalLength': 2.7,
                        'horizontalAperture': 2.682,
                        'verticalAperture': 1.509})
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

    def init_sensor_and_semantics(self):

        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        self.__sensor = Lidar()
        self.__add_semantics_to_all(stage)
        self.stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        self.timeline = (
            omni.timeline.get_timeline_interface()
        )  # Used to interact with simulation
        self.make_camera_stand()

        return

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
        stage = omni.usd.get_context().get_stage()  # Used to access Geometry
        omni.kit.commands.execute(
            "AddPhysicsSceneCommand",
            stage=stage,
            path="/World/PhysicsScene",
            context=omni.usd.get_context(),
        )

    def remove_all_objects(self):
        for i in reversed(range(len(self.__created_objs))):
            try:
                self._world.scene.remove_object(self.__created_objs[i])
            except:
                pass  # already deleted from world
            del self.__created_objs[i]

    async def final_fn(self):
        print("calling camera test")
        campath = "/World/CameraStand_Closeup/CameraCloseup"
        campath = "/World/Camera"
        campath = "/World/CameraStand_Closeup/CameraCloseup"
        rp = rep.create.render_product(campath, resolution=(640, 480))
        # set viewport
        topics = ["rgb", "full_pointcloud", "instanceSegmentation",
                  "camera", "depth"]
        from omni.kit.viewport.utility import get_active_viewport
        
        viewport = get_active_viewport()
        if not viewport: raise RuntimeError("No active Viewport")
        
        # Set the Viewport's active camera to the
        # camera prim path you want to switch to.
        viewport.camera_path = campath
        gt = asyncio.ensure_future(self.sd_watch.snap_async(topics, rp, viewport = viewport))
        del rp
        return
