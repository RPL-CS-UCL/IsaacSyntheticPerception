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
from omni.isaac.synthetic_utils import SyntheticDataHelper
from .syntehtic_data_watch import SyntheticDataWatch,SyntheticDataWatch_V2

from omni.kit.viewport.utility import get_active_viewport
def get_meters_per_unit():
    from pxr import UsdGeom
    stage = omni.usd.get_context().get_stage()
    return UsdGeom.GetStageMetersPerUnit(stage)

def gf_as_numpy(gf_matrix)->np.array:
    """Take in a pxr.Gf matrix and returns it as a numpy array.
    Specifically it transposes the matrix so that it follows numpy
    matrix rules.

    Args:
        gf_matrix (Gf.Matrix_d): Gf matrix to convert

    Returns:
        np.array:
    """
    # Convert matrix/vector to numpy array
    return np.array(list(gf_matrix)).T

def get_intrinsic_matrix(viewport):
    # Get camera params from usd
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(viewport.get_active_camera())
    focal_length = prim.GetAttribute("focalLength").Get()
    horiz_aperture = prim.GetAttribute("horizontalAperture").Get()
    x_min, y_min, x_max, y_max = viewport.get_viewport_rect()
    width, height = x_max - x_min, y_max - y_min

    # Pixels are square so we can do:
    vert_aperture = height / width * horiz_aperture

    # Compute focal point and center
    focal_x = width * focal_length / horiz_aperture
    focal_y = height * focal_length / vert_aperture
    center_x = width * 0.5
    center_y = height * 0.5
    
    # Turn into matrix
    intrinsic_matrix = np.array([[focal_x, 0, center_x],
                                 [0, focal_y, center_y],
                                 [0, 0, 1]])
    
    return intrinsic_matrix

def get_extrinsic_matrix(viewport, meters=False):
    from pxr import UsdGeom
    # Get camera pose
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(viewport.get_active_camera())
    camera_pose = gf_as_numpy(UsdGeom.Camera(camera_prim).GetLocalTransformation())
    if meters:
        camera_pose[:,3] = camera_pose[:,3]*get_meters_per_unit()
    
    view_matrix = np.linalg.inv(camera_pose)
    return view_matrix

def freq_count(v:np.array)->np.array:
    """Return the number of times each element in an array occur

    Args:
        v (np.array): 1D array to count

    Returns:
        np.array: Frequency list [[num, count], [num, count],...]
    """
    unique, counts = np.unique(v, return_counts=True)
    return np.asarray((unique, counts)).T

def pointcloud_from_mask_and_depth(depth:np.array, mask:np.array, mask_val:int, intrinsic_matrix:np.array, extrinsic_matrix:np.array=None):
    depth = np.array(depth).squeeze()
    mask = np.array(mask).squeeze()
    # Mask the depth array
    masked_depth = np.ma.masked_where(mask!=mask_val, depth)
    masked_depth = np.ma.masked_greater(masked_depth, 8000)
    # Create idx array
    idxs = np.indices(masked_depth.shape)
    u_idxs = idxs[1]
    v_idxs = idxs[0]
    # Get only non-masked depth and idxs
    z = masked_depth[~masked_depth.mask]
    compressed_u_idxs = u_idxs[~masked_depth.mask]
    compressed_v_idxs = v_idxs[~masked_depth.mask]
    # Calculate local position of each point
    # Apply vectorized math to depth using compressed arrays
    cx = intrinsic_matrix[0,2]
    fx = intrinsic_matrix[0,0]
    x = (compressed_u_idxs - cx) * z / fx
    cy = intrinsic_matrix[1,2]
    fy = intrinsic_matrix[1,1]
    # Flip y as we want +y pointing up not down
    y = -((compressed_v_idxs - cy) * z / fy)

    # Apply camera_matrix to pointcloud as to get the pointcloud in world coords
    if extrinsic_matrix is not None:
        # Calculate camera pose from extrinsic matrix
        camera_matrix = np.linalg.inv(extrinsic_matrix)
        # Create homogenous array of vectors by adding 4th entry of 1
        # At the same time flip z as for eye space the camera is looking down the -z axis
        w = np.ones(z.shape)
        x_y_z_eye_hom = np.vstack((x, y, -z, w))
        # Transform the points from eye space to world space
        x_y_z_world = np.dot(camera_matrix, x_y_z_eye_hom)[:3]
        return x_y_z_world.T
    else:
        x_y_z_local = np.vstack((x, y, z))
        return x_y_z_local.T
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
        self._rp = None

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

        self.init_camera()

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
        # if self._rp is None:
        rp = rep.create.render_product(campath, resolution=(640, 480))
                    # set viewport
        topics = ["rgb", "full_pointcloud", "instanceSegmentation",
                  "camera", "depth"]
        
        viewport = get_active_viewport()
        if not viewport: raise RuntimeError("No active Viewport")
        
        # Set the Viewport's active camera to the
        # camera prim path you want to switch to.
        viewport.camera_path = campath
        gt = asyncio.ensure_future(self.sd_watch.snap_async(topics, rp, viewport = viewport))
        del rp
        return


    import omni.replicator.core as rep
    def init_camera(self):
        self.cam = rep.create.camera(position=(0,0,0))
        self.rp = rep.create.render_product(self.cam, (512, 512))

        self.rgb_annot = self.rep.AnnotatorRegistry.get_annotator("rgb")
        self.rgb_annot.attach(self.rp)

        self.depth_annot = self.rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self.depth_annot.attach(self.rp)

        # self.pc_annot = self.rep.AnnotatorRegistry.get_annotator("pointcloud")
        # self.pc_annot.attach(self.rp)

        self.sem_annot = self.rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
        self.sem_annot.attach(self.rp)

    async def test(self):
        # from omni.isaac.sensor import Camera
        # c = Camera("/World/FakeCam")
        # c.add_semantic_segmentation_to_frame()
        # print(c.get_vertical_fov())
        # import omni.replicator.core as rep
        # cone = rep.create.cone()
        #
        # cam = rep.create.camera(position=(0,0,0), look_at=cone)
        # rp = rep.create.render_product(cam, (512, 512))
        #
        # rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        # rgb.attach(rp)
        # depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        # depth.attach(rp)

        await rep.orchestrator.step_async()
        rgb_data = self.rgb_annot.get_data()
        print(rgb_data.shape)
        np.save("/home/jon/Documents/temp/image.npy", rgb_data)
        depth_data = self.depth_annot.get_data()
        print(depth_data.shape)
        np.save("/home/jon/Documents/temp/depth.npy", depth_data)

        # pc_data = self.pc_annot.get_data()
        # print(pc_data)
        # np.save("/home/jon/Documents/temp/pc.npy", pc_data["data"])
        # np.save("/home/jon/Documents/temp/pc_col.npy", pc_data["info"]["pointRgb"])

        sem_data = self.sem_annot.get_data()
        print(sem_data)
        np.save("/home/jon/Documents/temp/sem.npy", sem_data)
