# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
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
from pxr import UsdGeom, Gf, UsdPhysics, Semantics              # pxr usd imports used to create cube
from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.range_sensor import _range_sensor 
from omni.isaac.core.utils.semantics import get_semantics
import omni
import asyncio  
import omni.kit.commands
import omni.timeline
from pxr import Sdf
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.rotations import euler_angles_to_quat, matrix_to_euler_angles
from omni.isaac.core.utils.semantics import add_update_semantics
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_property
import omni.kit.viewport
from time import sleep
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
import omni.replicator.core as rep 

class ExtensionName(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self.obstacles = []
        self.save_count = 0
        self.pri = None


    def setup_scene(self):
        return

    async def setup_pre_reset(self):
        return

    def world_cleanup(self):
        self.remove_all_objects()
        return

    def add_semantic(self, p, prim_class):

        sem_dict = get_semantics(p)
        collisionAPI = UsdPhysics.CollisionAPI.Apply(p)
        if "Semantics" not in sem_dict:
            print("adding semantics and collider to ", p.GetPrimPath(), " of class ", prim_class)
            sem = Semantics.SemanticsAPI.Apply(p, "Semantics")
            sem.CreateSemanticTypeAttr()
            sem.CreateSemanticDataAttr()
            sem.GetSemanticTypeAttr().Set("class")
            sem.GetSemanticDataAttr().Set(prim_class)

    def add_semantics_all(self, stage):
        prim_class = "undef" 
        for prim_ref in stage.Traverse():
            prim_ref_name = str(prim_ref.GetPrimPath())
            for word in prim_ref_name.split("/"):
                if "class" in word:
                    prim_class =  word
            if prim_ref.GetPrimTypeInfo().GetTypeName() == "Mesh":
                print("adding straight to mesh")
                self.add_semantic(prim_ref, prim_class)

            for i in range(len(prim_ref.GetChildren())):
                prim_child =prim_ref.GetChildren()[i] 
                if prim_child.GetPrimTypeInfo().GetTypeName() == "Mesh":
                    p = stage.GetPrimAtPath(prim_child.GetPrimPath())
                    for word in str(prim_child.GetPrimPath()).split("/"):
                        if "class" in word:
                            prim_class =  word
                    self.add_semantic(p,prim_class)


    def add_to_scene(self):
        self.world_cleanup()
        stage = omni.usd.get_context().get_stage()
        # timeline = omni.timeline.get_timeline_interface()

        # Create the lidar
        lidarPath = "/LidarName"
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=lidarPath,
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
            enable_semantics=False
        )
        UsdGeom.XformCommonAPI(prim).SetTranslate((2.0, 0.0, 4.0))
        self.obstacles.append(prim)
        self.add_semantics_all(stage)
        self.pri = prim
        self.stage = omni.usd.get_context().get_stage()                      # Used to access Geometry
        self.timeline = omni.timeline.get_timeline_interface()               # Used to interact with simulation

        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR
        self.lidarPath = "/LidarName"
        return

    def clear_max_lidar_points(self, pc, sem, lidar_pos,max_dist):
        new_points = []
        new_sems = []
        for seq_id in range(len(pc)):
            for point_id in range(len(pc[seq_id])):
                point = pc[seq_id][point_id]
                dist = np.linalg.norm(point-lidar_pos)
                if dist < max_dist-10:
                    new_points.append(pc[seq_id][point_id])
                    new_sems.append(sem[seq_id][point_id])

        return np.array(new_points), np.array(new_sems)

    async def save_lidar_data(self):
        # semantic_ = pri.GetAttribute("enabled")
        # semantic_.Set(1)
        # await asyncio.sleep(1.0)
        # self.timeline.pause()
        transform = Gf.Transform()
        transform.SetMatrix(UsdGeom.Xformable(self.pri).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))

        pointcloud = self.lidarInterface.get_point_cloud_data("/World"+self.lidarPath)
        semantics = self.lidarInterface.get_semantic_data("/World"+self.lidarPath)
        position = transform.GetTranslation()
        # semantic_.Set(0)
        print(position)
        pointcloud,semantics = self.clear_max_lidar_points(pointcloud, semantics, position, 100)
        np.save(f"/home/jon/Documents/RandLA-Net/pc_{self.save_count}.npy",np.array(pointcloud) )
        np.save(f"/home/jon/Documents/RandLA-Net/sem_{self.save_count}.npy", np.array(semantics))
        self.save_count += 1

    async def setup_post_load(self):
        stage = omni.usd.get_context().get_stage()                      # Used to access Geometry
        omni.kit.commands.execute('AddPhysicsSceneCommand',stage = stage, path='/World/PhysicsScene',context = omni.usd.get_context()) 
    
    def remove_all_objects(self):
        for i in reversed(range(len(self.obstacles))):
            try:
                self._world.scene.remove_object(self.obstacles[i])
            except:
                pass # already deleted from world
            del self.obstacles[i]


