from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
import numpy as np
import omni.replicator.core as rep
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control


class DepthCamera:
    def __init__(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        image_size=(512, 512),
        attach=True,
        parent='/World/DepthCamera',
        name='DepthCamera',
    ) -> None:
        self.__rgb_annot: Annotator
        self.__save_path = ''
        self.__pos = position
        self.__rot = rotation
        self.__image_size = image_size
        self.__attach = attach
        self.__name = name
        self.__focal_length = 24.0
        self.__focus_distance = 400.0
        self.__f_stop = 0.0
        self.__horizontal_aperture = 20.955
        self.__horizontal_aperture_offset = 0.0
        self.__vertical_aperture_offset = 0.0
        self.__clipping_range = (1.0, 10000000.0)
        self.__resolution = (512,512)

    def init_sensor(self, parent):
        self.__cam = rep.create.camera(
            position=self.__pos, parent=parent, name=self.__name, rotation = self.__rot,
            focal_length = self.__focal_length,
            focus_distance = self.__focus_distance,
            f_stop = self.__f_stop,
            horizontal_aperture = self.__horizontal_aperture,
            horizontal_aperture_offset = self.__horizontal_aperture_offset,
            vertical_aperture_offset = self.__vertical_aperture_offset,
            clipping_range = self.__clipping_range


        )
        self.__rp: og.Node = rep.create.render_product(
            self.__cam, self.__resolution
        )
        if self.__attach:
            self.__init_annotators()
            self.__attach_annotoators()

    def construct_pc(self, rgb_image, depth_image):
        pass

    def __init_annotators(self):
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator('rgb')
        self.depth_annot = rep.AnnotatorRegistry.get_annotator(
            'distance_to_camera'
        )
        # self.pc_annot = rep.AnnotatorRegistry.get_annotator("pointcloud")
        self.sem_annot = rep.AnnotatorRegistry.get_annotator(
            'semantic_segmentation'
        )

    def __attach_annotoators(self):
        self.depth_annot.attach(self.__rp)
        self.rgb_annot.attach(self.__rp)
        self.sem_annot.attach(self.__rp)
        # self.pc_annot.attach(self.rp)

    def __detatch_annototators(self):
        self.depth_annot.detach(self.__rp)
        self.rgb_annot.detach(self.__rp)
        self.sem_annot.detach(self.__rp)
        # self.pc_annot.dettach(self.rp)

    async def sample_sensor(self):
        await rep.orchestrator.step_async()

        rgb_data = self.rgb_annot.get_data()
        np.save('/home/jon/Documents/temp/image.npy', rgb_data)

        depth_data = self.depth_annot.get_data()
        np.save('/home/jon/Documents/temp/depth.npy', depth_data)

        sem_data = self.sem_annot.get_data()
        np.save('/home/jon/Documents/temp/sem.npy', sem_data)
        return
    def read_from_json(self,data):
        # We have been given data["LIDAR"]
        for instance_ids in data:
            camera_settings = data[instance_ids]
            self.__name = camera_settings["name"]
            self.__focal_length = camera_settings["focal_length"] 
            self.__focus_distance = camera_settings["focus_distance"]
            self.__f_stop = camera_settings["f_stop"] 
            self.__horizontal_aperture = camera_settings["horizontal_aperture"]
            self.__horizontal_aperture_offset = camera_settings["horizontal_aperture_offset"]
            self.__vertical_aperture_offset = camera_settings["vertical_aperture_offset"]
            self.__clipping_range = camera_settings["clipping_range"]
            self.__resolution = camera_settings["resolution"]
