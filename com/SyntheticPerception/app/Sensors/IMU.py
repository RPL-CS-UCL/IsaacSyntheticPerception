import pathlib
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
import omni

from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.dynamic_control import _dynamic_control

from pxr import Usd, Gf, UsdGeom
from omni.isaac.sensor import _sensor

from sensor_msgs.msg import Imu
from omni.isaac.sensor import IMUSensor
from geometry_msgs.msg import Quaternion
import rospy
class IMUSensor:
    def __init__(
        self,
        position=(0, 0, 0),
        rotation=(0, 0, 0),
        orientation=(1, 1, 1, 1),
        parent='/World',
        name='/DepthCamera',
    ) -> None:
        self.__pos = position
        self.__ori = orientation
        self.__rot = rotation
        self.__name = name
        # self.__imu_prim
        self.__path = ''
        self.save_path = ""
        self.sample_count = 0
        #     self.__attach_annotoators()
        self._o = "[IMUSensor] "

    def init_ros(self):
        self.pub = rospy.Publisher('/ouster/imu', Imu, queue_size=10)
        pass

    def init_output_folder(self, path):

        print(f"{self._o} Initializing output folders")
        self.save_path =path +"/posesIMU" 

        pathlib.Path(self.save_path).mkdir(parents=True, exist_ok=True)
        

    def init_sensor(self, parent):
        x,y,z = self.__pos
        qx,qw,qy,qz = self.__rot
        self.__path = parent + "/" + self.__name
        result, self.__imu_prim = omni.kit.commands.execute(
            'IsaacSensorCreateImuSensor',
            path='/' + self.__name,
            parent=parent,
            sensor_period=-1.0,
            translation=Gf.Vec3d(x,y,z),
            orientation=Gf.Quatd(qx,qw,qy,qz),
            visualize=True,
        )

        self._is = _sensor.acquire_imu_sensor_interface()

    def read_from_json(self, data):
        # We have been given data["LIDAR"]
        self.__name = data['name']
        self.__pos = data['position']
        self.__rot =data['rotation']

    def sample_sensor(self):
        # print(self.__path)
        # return

        # await rep.orchestrator.step_async()
        reading = self._is.get_sensor_readings(self.__path,  read_gravity =False)
        # print("b: ", reading)
        imu_msg = self.numpy_to_imu_msg(reading[0]) 
        self.pub.publish(imu_msg)
        self.pub.publish(self.numpy_to_imu_msg(reading[0]))
        self.pub.publish(self.numpy_to_imu_msg(reading[0]))
        self.pub.publish(self.numpy_to_imu_msg(reading[0]))

        # np.save(f"{self.save_path}/{self.sample_count}.npy",reading)

        self.sample_count += 1
    def numpy_to_imu_msg(self, np_array):
        # print("imu in func ", np_array)
        imu_msg = Imu()

        # Assuming your array is structured as:
        # time, lin_acc_x, lin_acc_y, lin_acc_z, ang_acc_x, ang_acc_y, ang_acc_z, orix, oriy, oriz, oriw, boolean

        # Linear acceleration
        imu_msg.linear_acceleration.x = np_array[1]
        imu_msg.linear_acceleration.y = np_array[2]
        imu_msg.linear_acceleration.z = np_array[3]

        # Angular velocity
        imu_msg.angular_velocity.x = np_array[4]
        imu_msg.angular_velocity.y = np_array[5]
        imu_msg.angular_velocity.z = np_array[6]

        # Orientation (assuming it's a quaternion)
        # imu_msg.orientation = Quaternion(np_array[7][0], np_array[7][1], np_array[7][2], np_array[7][3])
        imu_msg.orientation = Quaternion(0,0,0,1)

        # Header
        imu_msg.header.stamp = rospy.Time.now()#+ rospy.Duration(1)#.to_nsec()+ 0.1 
        imu_msg.header.frame_id = "os_imu"  # Change to your frame
        # print("imu", imu_msg.header.stamp)

        return imu_msg
