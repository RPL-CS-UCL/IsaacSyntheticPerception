import pathlib
from omni.syntheticdata.scripts.sensors import enable_sensors
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from omni.isaac.range_sensor import _range_sensor
import omni
import omni.kit.commands
import omni.timeline
import omni.kit.viewport
from pxr import Usd, Gf, UsdGeom
import omni.kit.commands
import numpy as np
import omni.replicator.core as rep
import numpy as np
from typing import Any, Dict, Sequence, Tuple, Union
import omni.graph.core as og
from omni.replicator.core.scripts.annotators import Annotator

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control

from omni.syntheticdata._syntheticdata import acquire_syntheticdata_interface
from pxr import Sdf
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import inspect
from sensor_msgs import point_cloud2

from omni.syntheticdata import helpers
from omni.isaac.core.utils.prims import get_prim_at_path  # , get_prim_property


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
        self.__path = "/" + path
        self.__min_range = min_range
        self.__max_range = max_range
        self.__draw_points = draw_points
        self.__draw_lines = draw_lines
        self.__horizontal_fov = horizontal_fov
        self.__vertical_fov = vertical_fov
        self.__horizontal_resolution = horizontal_resolution
        self.__vertical_resolution = vertical_resolution
        self.__rotation_rate = rotation_rate
        self.__high_lod = high_lod
        self.__yaw_offset = yaw_offset
        self.__enable_semantics = enable_semantics
        self.__origin_pos = origin_pos
        self.__rotation = [0.0, 0.0, 0.0]
        self.sample_count = 0
        self.save_path = None

        self._syn_interface =None
        self._o = "[LiDAR] "
        self.sequence = 0
    

    def _init_seq_folder(self,int_seq):
        seq = format(int_seq, '02d')

        velo_path = pathlib.Path(f"{self.save_path}/sequences_raw/{seq}/velodyne")
        if not velo_path.is_dir():
            velo_path.mkdir(parents=True, exist_ok=True)

        label_path = pathlib.Path(f"{self.save_path}/sequences_raw/{seq}/labels")
        if not label_path.is_dir():
            label_path.mkdir(parents=True, exist_ok=True)

        # pathlib.Path(path + "/velodyneLabels").mkdir(parents=True, exist_ok=True)
    def init_output_folder(self, path):
        print(f"{self._o} Initializing output folders")
        self.save_path = path

        self._init_seq_folder(0)
        # pathlib.Path(path + "/velodyne").mkdir(parents=True, exist_ok=True)
        #
        # pathlib.Path(path + "/velodyneLabels").mkdir(parents=True, exist_ok=True)

    def read_from_json(self, data):
        # We have been given data["LIDAR"]
        # for instance_ids in data:
        lidar_settings = data
        print(lidar_settings["name"])
        self.__path = "/" + lidar_settings["name"]
        self.__min_range = lidar_settings["min_range"]
        self.__max_range = lidar_settings["max_range"]
        self.__draw_points = lidar_settings["draw_points"]
        self.__draw_lines = lidar_settings["draw_lines"]
        self.__horizontal_fov = lidar_settings["horizontal_fov"]
        self.__vertical_fov = lidar_settings["vertical_fov"]
        self.__horizontal_resolution = lidar_settings["horizontal_resolution"]
        self.__vertical_resolution = lidar_settings["vertical_resolution"]
        self.__rotation_rate = lidar_settings["rotation_rate"]
        self.__high_lod = lidar_settings["high_lod"]
        self.__yaw_offset = lidar_settings["yaw_offset"]
        self.__enable_semantics = lidar_settings["enable_semantics"]
        self.__origin_pos = lidar_settings["origin_pos"]
        self.__rotation = lidar_settings["rotation"]

    def init_sensor(self, parent):
        print(f"init the lidar {parent}")
        lidar_config = "Example_Rotary"
        lidar_config = "OS0_128ch20hz1024res"
        _, self.__lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self.__path,
            parent=parent,
            config=lidar_config,
            translation=(0, 0, 1.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        )
        self.hydra_texture = rep.create.render_product(
            self.__lidar_prim.GetPath(), [1, 1], name="Isaac"
        )

        print("=== Trying to init annotator")
        self.annotator = rep.AnnotatorRegistry.get_annotator(
            "RtxSensorCpuIsaacCreateRTXLidarScanBuffer"
        )
        self.annotator.initialize(
            transformPoints=False,
            outputTimestamp=True,
            outputObjectId=True,
            outputDistance=True,
            outputIntensity=True,
            outputBeamId=True,
            outputEmitterId=True,
        )  # outputElevation=True)
        self.annotator.attach([self.hydra_texture])

        # print("=== Trying to init writer")
        self.writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud" + "Buffer")
        self.writer.attach([self.hydra_texture])
        # self.annotator2 = rep.AnnotatorRegistry.get_annotator(
        #     "RtxSensorCpuIsaacReadRTXLidarData"
        # )
        # argspec = inspect.getfullargspec(self.annotator.initialize)
        # default_values = argspec.defaults
        # kwarg_names = argspec.args[-len(default_values) :] if default_values else []

        # print("Keyword argument names:", kwarg_names)

        # self.annotator2.initialize(outputData=True)
        # self.annotator2.attach([self.hydra_texture])

        # self.writer2= rep.writers.get("RtxLidar" + "ROS1PublishPointCloud")
        # self.writer2.initialize(topicName="ouster/points", frameId="camera_init")
        # self.writer2.attach([self.hydra_texture])

        # ====== outdated physx lidar, left incase
        # self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
        # _, self.__lidar_prim = omni.kit.commands.execute(
        #     'RangeSensorCreateLidar',
        #
        #     path=self.__path,
        #     parent=parent,
        #     min_range=self.__min_range,
        #     max_range=self.__max_range,
        #     draw_points=self.__draw_points,
        #     draw_lines=self.__draw_lines,
        #     horizontal_fov=self.__horizontal_fov,
        #     vertical_fov=self.__vertical_fov,
        #     horizontal_resolution=self.__horizontal_resolution,
        #     vertical_resolution=self.__vertical_resolution,
        #     rotation_rate=self.__rotation_rate,
        #     high_lod=self.__high_lod,
        #     yaw_offset=self.__yaw_offset,
        #     enable_semantics=self.__enable_semantics,
        # )
        # UsdGeom.XformCommonAPI(self.__lidar_prim).SetTranslate(
        #     self.__origin_pos
        # )
        self.__lidar_path = parent + self.__path
        # print(f'lidar path should be {self.__lidar_path}')
        # self.__lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

    def init_ros(self):
        print("initialising ros for lidar")
        rospy.init_node("lidarnod2", anonymous=True)
        self.pub = rospy.Publisher("/ouster/points", PointCloud2, queue_size=10)

        self.obj_pub = rospy.Publisher("/objects", String, queue_size=10)
        # rospy.init_node('point_cloud_listener', anonymous=True)
        # rospy.Subscriber("/ouster/points", PointCloud2, self.point_cloud_callback)
        # rospy.spin()

    def sample_sensor(self):
        self.get_pc_and_semantic()
        self.sample_count += 1

    def array_to_pointcloud2(self, cloud_arr, stamp=None, frame_id=None):
        """Converts a numpy record array to a sensor_msgs.msg.PointCloud2."""

        cloud_msg = PointCloud2()

        if stamp is not None:
            cloud_msg.header.stamp = stamp
        if frame_id is not None:
            cloud_msg.header.frame_id = frame_id
        cloud_msg.height = cloud_arr.shape[0]
        cloud_msg.width = cloud_arr.shape[1]
        cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
        cloud_msg.is_bigendian = False  # assumption
        cloud_msg.point_step = cloud_arr.dtype.itemsize
        cloud_msg.row_step = cloud_msg.point_step * cloud_arr.shape[1]
        cloud_msg.is_dense = all(
            [np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names]
        )
        cloud_msg.data = cloud_arr.tostring()
        return cloud_msg

    def create_point_cloud2(
        self, points, intensities, timestamps, rings, ranges, frame_id="os_lidar"
    ):
        # Create header
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        # print("header in nano", header.stamp.to_nsec())
        header.frame_id = frame_id

        # Create fields for PointCloud2
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),  # 4,
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),  # 8
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),  # 12
            PointField(
                name="intensity", offset=16, datatype=PointField.FLOAT32, count=1
            ),
            PointField(name="t", offset=20, datatype=PointField.UINT32, count=1),
            PointField(
                name="reflectivity", offset=24, datatype=PointField.UINT16, count=1
            ),
            PointField(name="ring", offset=26, datatype=PointField.UINT16, count=1),
            PointField(name="ambient", offset=28, datatype=PointField.UINT16, count=1),
            PointField(name="range", offset=32, datatype=PointField.UINT32, count=1),
        ]

        # Convert data to bytes
        buf = []
        for point, intensity, timestamp, ring, rangee in zip(
            points, intensities, timestamps, rings, ranges
        ):
            # print(ring)
            # Pack each point data with padding to align the total size to 40 bytes
            point_data = struct.pack("fff", *point)  # 12 bytes for x, y, z

            point_data += b"\x00" * 4  # 6 bytes of padding to reach 40 bytes total
            point_data += struct.pack("f", intensity)  # 4 bytes for intensity
            point_data += struct.pack(
                "I", int(timestamp / 10000)
            )  # 4 bytes for timestamp

            point_data += struct.pack("H", int(5))  # 2 bytes for reflectivity

            point_data += struct.pack("H", int(ring))  # 2 bytes for ring
            point_data += struct.pack("H", int(3))  # 2 bytes for ambient

            point_data += b"\x00" * 2  # 6 bytes of padding to reach 40 bytes total
            point_data += struct.pack("I", int(rangee))  # 4 bytes for range
            point_data += b"\x00" * 4  # 6 bytes of padding to reach 40 bytes total

            buf.append(point_data)

        # Create PointCloud2 message
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = fields
        pc2.point_step = 40  # size of point in bytes, including padding
        pc2.row_step = pc2.point_step * pc2.width
        pc2.data = b"".join(buf)

        return pc2

    # def point_cloud_callback(self,point_cloud):
    #     # Assuming the structure based on the provided fields and point_step of 30 bytes
    #     print(point_cloud.point_step)
    #     # assert point_cloud.point_step == 36, "Point step does not match expected size"
    #
    #     print(" ------ ")
    #
    #     header_stamp_sec = point_cloud.header.stamp.to_sec()  # Convert to seconds
    #     header_stamp_nsec = point_cloud.header.stamp.to_nsec()  # Convert to nanoseconds
    #     print(f"Header Stamp: {header_stamp_sec} seconds, {header_stamp_nsec} nanoseconds")
    #     # Decode each point in the point cloud
    #     for p in point_cloud2.read_points(point_cloud, skip_nans=True, field_names=("x", "y", "z", "intensity", "reflectivity", "ambient", "t", "ring", "range")):
    #         # p will contain the values in the order specified in field_names
    #         x, y, z, intensity, reflectivity, ambient, timestamp, ring, range = p
    #         print(f"{header_stamp_sec} seconds, {header_stamp_nsec} nanosecondsPoint: x={x}, y={y}, z={z}, intensity={intensity}, timestamp={timestamp}, ring={ring}, range={range}")
    #         # Process the point here (e.g., print it or store it for further processing)
    def point_cloud_callback(self, point_cloud):
        # Assuming the structure based on the provided fields and point_step
        print(point_cloud.point_step)

        print(" ------ ")

        header_stamp_sec = point_cloud.header.stamp.to_sec()  # Convert to seconds
        header_stamp_nsec = point_cloud.header.stamp.to_nsec()  # Convert to nanoseconds
        print(
            f"Header Stamp: {header_stamp_sec} seconds, {header_stamp_nsec} nanoseconds"
        )

        # Calculate the number of points in the cloud
        num_points = len(point_cloud.data) // point_cloud.point_step

        # Iterate over each point in the point cloud
        for i in range(num_points):
            # Calculate the starting byte of the current point
            start_byte = i * point_cloud.point_step
            # Decode each field based on its offset and type
            x = struct.unpack_from("f", point_cloud.data, start_byte + 0)[0]
            y = struct.unpack_from("f", point_cloud.data, start_byte + 4)[0]
            z = struct.unpack_from("f", point_cloud.data, start_byte + 8)[0]
            intensity = struct.unpack_from("f", point_cloud.data, start_byte + 16)[0]
            timestamp = struct.unpack_from("I", point_cloud.data, start_byte + 20)[0]
            reflectivity = struct.unpack_from("H", point_cloud.data, start_byte + 24)[0]
            ring = struct.unpack_from("H", point_cloud.data, start_byte + 26)[0]
            ambient = struct.unpack_from("H", point_cloud.data, start_byte + 28)[0]
            range2 = struct.unpack_from("I", point_cloud.data, start_byte + 32)[0]

            print(
                f"{header_stamp_sec} seconds, {header_stamp_nsec} nanoseconds - Point: x={x}, y={y}, z={z}, intensity={intensity}, timestamp={timestamp}, ring={ring}, range={range2}"
            )

    def get_pc_and_semantic(self, save_path="/home/jon/Documents/temp/a"):
        ob_ids = self.annotator.get_data()["objectId"]  # ids
        unique_obs_ids = set(ob_ids)
        semantic_labels = []


        
        # current_velo_save_dir = pathlib.Path(f"{self.save_path}velodyne/{format(self.sequence, '02d')}")
        # if not current_velo_save_dir.is_dir():
        #     current_velo_save_dir .mkdir(parents=True, exist_ok=True)
        # current_velo_label_save_dir = pathlib.Path(f"{self.save_path}velodyneLabels/{format(self.sequence, '02d')}")
        #
        # if not current_velo_label_save_dir.is_dir():
        #     current_velo_label_save_dir.mkdir(parents=True, exist_ok=True)
        # ==================================================
        # code for the llm graph stuff
        # llm_strings = []
        # for u_obj in unique_obs_ids:
        #     primpath = (
        #         acquire_syntheticdata_interface().get_uri_from_instance_segmentation_id(
        #             u_obj
        #         )
        #         )
        #     primpath_split = primpath.split("/")
        #     llm_strings.append(primpath_split[-1])
        # print(llm_strings)
        # self.obj_pub.publish(",".join(llm_strings))
        # return
        # ==============================================
        if self._syn_interface is None:
            self._syn_interface = acquire_syntheticdata_interface()

        # Get all the unique objects and get their class labels:
        obj_to_class = {}
        for u_obj in unique_obs_ids:
            primpath = (
                self._syn_interface.get_uri_from_instance_segmentation_id(
                    u_obj
                )
            )
            # we most likely got the most child node, but we will need its most
            # obj parent
            primpath_split = primpath.split("/")
            path_id = 0
            for word in primpath_split:
                path_id += 1
                if "class" in word:
                    break
            # path id will be at the obj path of class, we want the obj just below it
            # path_id +=1
            primpath = "/".join(primpath_split[: path_id + 1])

            prim = get_prim_at_path(primpath)
            semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics")
            type_attr = semantic_api.GetSemanticTypeAttr()
            data_attr = semantic_api.GetSemanticDataAttr()
            obj_to_class[u_obj] = data_attr.Get()
            if obj_to_class[u_obj] is None:
                obj_to_class[u_obj] = "None"

            # print(f"querying ", primpath, "  :  ", obj_to_class[u_obj])

        raw_mapping = helpers.get_instance_mappings()
        self.mapping = {"None": 9999}
        for data in raw_mapping:
            class_id = data[2]
            class_name = data[3]
            if class_id not in self.mapping:
                self.mapping[class_name] = class_id
        # print(self.mapping)
        # print(obj_to_class)

        object_id_to_class_name = np.vectorize(lambda x: obj_to_class[x])(ob_ids)

        final_class_ids = np.vectorize(lambda x: self.mapping[x])(
            object_id_to_class_name
        )

        pc = self.annotator.get_data()["data"]
        # print(final_class_ids)

        # ======= The following are other possible data we can get
        t = self.annotator.get_data()["timestamp"]
        intensities = self.annotator.get_data()["intensity"]
        beamId = self.annotator.get_data()["beamId"]
        distance = self.annotator.get_data()["distance"]

        # ===== the following line is needed to convert and publish
        msg = self.create_point_cloud2(pc, intensities, t,beamId,distance)
        self.pub.publish(msg)
        # =================================
        
        np.save(f"{self.save_path}sequences_raw/{format(self.sequence, '02d')}/velodyne/{self.sample_count}.npy",pc)
        np.save(f"{self.save_path}sequences_raw/{format(self.sequence, '02d')}/labels/{self.sample_count}.npy",final_class_ids, allow_pickle=True)
        if self.sample_count > 1000:
            self.sequence +=1
            self.sample_count = 0
        # return pointcloud, semantics
        return None, None

    def __get_position(self):
        transform = Gf.Transform()
        transform.SetMatrix(
            UsdGeom.Xformable(self.__lidar_prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
        )
        return transform.GetTranslation()

    def __clear_max_lidar_points(self, pc, sem, lidar_pos, max_dist):
        # print(lidar_pos, max_dist)
        new_points = []
        new_sems = []
        for seq_id in range(len(pc)):
            for point_id in range(len(pc[seq_id])):
                point = pc[seq_id][point_id]
                dist = np.linalg.norm(point - lidar_pos)

                # print(point, " , ", lidar_pos, " , ", dist)
                if dist < max_dist - 20:
                    new_points.append(pc[seq_id][point_id])
                    new_sems.append(sem[seq_id][point_id])

        return np.array(new_points), np.array(new_sems)
