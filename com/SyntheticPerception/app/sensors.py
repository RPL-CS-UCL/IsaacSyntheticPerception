from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from omni.isaac.range_sensor import _range_sensor
import omni
import asyncio
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from time import sleep
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
import omni.replicator.core as rep


class DepthCamera:
    def __init__(
        self, position=(0, 0, 0), rotation=(0, 0, 0), image_size=(512, 512), attach=True
    ) -> None:
        self.__cam = rep.create.camera(position=position)
        self.__rp = rep.create.render_product(self.__cam, image_size)

        if attach:
            self.__init_annotators()
            self.__attach_annotoators()

    def __init_annotators(self):
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        # self.pc_annot = rep.AnnotatorRegistry.get_annotator("pointcloud")
        self.sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")

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
        np.save("/home/jon/Documents/temp/image.npy", rgb_data)

        depth_data = self.depth_annot.get_data()
        np.save("/home/jon/Documents/temp/depth.npy", depth_data)

        sem_data = self.sem_annot.get_data()
        np.save("/home/jon/Documents/temp/sem.npy", sem_data)
        return


class Lidar:
    def __init__(
        self,
        path="/Lidar1",
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
        enable_semantics=False,
        origin_pos=(2.0, 0.0, 4.0),
    ):
        result, self.__lidar_prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path=path,
            parent=parent,
            min_range=min_range,
            max_range=max_range,
            draw_points=draw_points,
            draw_lines=draw_lines,
            horizontal_fov=horizontal_fov,
            vertical_fov=vertical_fov,
            horizontal_resolution=horizontal_resolution,
            vertical_resolution=vertical_resolution,
            rotation_rate=rotation_rate,
            high_lod=high_lod,
            yaw_offset=yaw_offset,
            enable_semantics=enable_semantics,
        )
        UsdGeom.XformCommonAPI(self.__lidar_prim).SetTranslate(origin_pos)
        self.__lidar_path = parent + path
        self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
        self.__max_range = max_range

    def get_pc_and_semantic(self, save_path=None):
        pointcloud = self.__lidarInterface.get_point_cloud_data(self.__lidar_path)
        semantics = self.__lidarInterface.get_semantic_data(self.__lidar_path)
        lidar_position = self.__get_position()
        pointcloud, semantics = self.__clear_max_lidar_points(
            pointcloud, semantics, lidar_position, self.__max_range
        )

        if save_path is not None:
            np.save(
                f"{save_path}_pc.npy",
                np.array(pointcloud),
            )
            np.save(
                f"{save_path}_sem.npy",
                np.array(semantics),
            )
        return pointcloud, semantics

    def __get_position(self):
        transform = Gf.Transform()
        transform.SetMatrix(
            UsdGeom.Xformable(self.__lidar_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
        )
        return transform.GetTranslation()

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
