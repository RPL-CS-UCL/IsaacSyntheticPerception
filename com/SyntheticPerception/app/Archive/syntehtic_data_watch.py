import os, sys 

from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.syntheticdata.scripts.sensors
from omni.syntheticdata.scripts.sensors import get_normals
import omni.kit.viewport
try: 
    import omni.isaac.version as v
    VERSION = v.get_version()[0]
except:
    VERSION = "2021"

import asyncio
from PIL import Image
import numpy as np
import warnings
from copy import copy

# 2022.2 imports
try: 
    import omni.replicator.core as rep 
    from omni.isaac.core.utils.viewports import project_depth_to_worldspace, backproject_depth
except:pass

class SyntheticDataWatch_V2(object):
    def __init__(self, savedir, render_product=None, robot=None):
        self.configure_saving(savedir)
        
        self.annotators = dict(
            rgb = rep.AnnotatorRegistry.get_annotator("rgb"),
            instanceSegmentation = rep.AnnotatorRegistry.get_annotator("instance_segmentation"),
            # pointcloud = rep.AnnotatorRegistry.get_annotator("pointcloud"),
            depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"),
            camera = rep.AnnotatorRegistry.get_annotator("camera_params")
        )
        self.sdh_topics = list(self.annotators.keys())
        print(" ============== sdh topics ========== o" )
        print(self.sdh_topics)
        self._max_clip_range = 30.

        self.set_robot(robot)
        if render_product is not None: self.set_render(render_product)
        self.cnt = 0

    def set_render(self, render_product):
        if render_product is None: raise ValueError("No render_product passed to watcher")
        # rp = rep.create.render_product("/World/CameraStand_Closup/CameraCloseup", 
        #                                                 resolution=(240, 320))
        rp = render_product
        for k in self.sdh_topics:
            self.annotators[k].attach(rp)
        return

    def unset_render(self, render_product):
        print ("Unsetting the render product...")
        if render_product is None: raise ValueError("No render_product passed to watcher")
        rp = render_product
        for k in self.sdh_topics:
            rep.AnnotatorRegistry.detach(self.annotators[k], rp)
        return


    def configure_saving(self, savedir):
        if savedir is not None:
            self.makedir(savedir)
            for i in range(1000):
                if os.path.exists(savedir+"/%d"%i):
                    continue
                else:
                    self.savedir = savedir+"/%d"%i
                    print (f"Saving in {self.savedir}")
                    self.is_saving_on = True
                    break
            else:
                raise ValueError("Out of range for savedir names")
        else:
            self.is_saving_on = False

    def set_robot(self, robot):
        self.robot=robot
        pass

    async def get(self, topics, viewport=None):
        """ Returns a dict with all requested data. """
        sdh_topics = [t for t in topics if t in self.sdh_topics]
        non_sdh_topics = [t for t in topics if t not in self.sdh_topics]

        await rep.orchestrator.step_async()
        gt =dict()
        for t in sdh_topics:
            gt[t] = self.annotators[t].get_data(device="cpu")

        for t in non_sdh_topics:
            if t == "joint_positions":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                gt[t] = self.robot.get_joint_positions().tolist()
            elif t == "applied_joint_positions":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                gt[t] =  self.robot.get_applied_action().joint_positions.tolist()
            elif t == "end_effector":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                #gt[t] = self.robot.EE.get_world_pose()
                gt[t] = self.robot.EE.get_world_pose()
                #gt[t] = tf_matrix_from_pose(gt[t][0], gt[t][1])
            # elif t == "camera":
            #     gt[t] = self.sd_helper.get_camera_params(self.viewport)
            #     gt[t]["clipping_range"] = (gt[t]["clipping_range"][0],gt[t]["clipping_range"][1])
            elif t=="full_pointcloud": 
                if not "depth" in list(gt.keys()): raise ValueError("No depth provided")      
                depth = gt["depth"]         
                pts = project_depth_to_worldspace(depth, viewport, self._max_clip_range)
                pts = np.asarray(pts).reshape(depth.shape[0], depth.shape[1],-1)
                gt[t] = pts
            else:
                warnings.warn("topic %s is not handled by SD Get Watch"%t)
        rep.orchestrator.stop()

        return copy(gt)

    def makedir(self, path):
        os.makedirs(path, exist_ok=True)

    def save_topics(self, gt, filenum, topics):
        if not self.is_saving_on:
            raise ValueError("Cannot save topics without savedir")
        for t in list(gt.keys()):
            print ("Snap - Internal", t)
            self.makedir("%s/%s/"%(self.savedir,t))
            if t in ["rgb"]:
                #im = gt[t].numpy()[:,:,:3]
                im = gt[t][:,:,:3]
                im = Image.fromarray(im)
                im.save("%s/%s/%s.png"%(self.savedir,t,filenum))
            elif t in ["depth"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), gt[t])
            elif t in ["instanceSegmentation"]:
                # im is List[
                #           (ID, Path, ID?, class, 
                #            List[sub part ids], 
                #            empty string?)]
                instance_data, instance_mapping = gt[t]["data"], gt[t]["info"]
                # instance_data, instance_mapping = gt[t]["data"].numpy(), gt[t]["info"]
                isg = dict(data = instance_data, mapping = instance_mapping)
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), isg)
                #np.save("%s/%s_map/%s.npy"%(self.savedir,t,filenum), instance_data)
            elif t in ["joint_positions", "applied_joint_positions"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), gt[t])
            elif t in ["end_effector", "camera"]:
                ee = np.asarray(gt[t], dtype=object)
                np.save("%s/%s/%s"%(self.savedir,t,filenum), ee)
            elif t in ["pointcloud"]:
                # pcd_dict = dict(data = gt[t]["data"].numpy(), mapping = gt[t]["info"])
                pcd_dict = dict(data = gt[t]["data"], mapping = gt[t]["info"])
                np.save("%s/%s/%s"%(self.savedir,t,filenum), pcd_dict)
            elif t in ["full_pointcloud"]:
                np.save("%s/%s/%s"%(self.savedir,t,filenum), gt[t])
            else:
                warnings.warn("topic %s is not handled by SD Save Watch saving"%t)

    async def snap_async(self, topics, rp, filename=None, timestamp=None, viewport=None):
        if not self.is_saving_on:
            raise ValueError("Cannot save topics without savedir")
        # Make dir again, with subfolder. Only in runs when snap is called
        self.set_render(rp)
        self.makedir(self.savedir) 
        self.cnt+=1
        if timestamp is not None:
            filename = str(timestamp)
        else:
            filename = "%d"%self.cnt

        gt = await self.get(topics, viewport)
        self.save_topics(gt, filenum=filename, topics=topics)
        self.unset_render(rp)
        return gt

class SyntheticDataWatch():
    def __init__(self, savedir, viewport=None, robot=None):
        if savedir is not None:
            self.makedir(savedir)
            for i in range(1000):
                if os.path.exists(savedir+"/%d"%i):
                    continue
                else:
                    self.savedir = savedir+"/%d"%i
                    print (f"Saving in {self.savedir}")
                    self.is_saving_on = True
                    break
            else:
                raise ValueError("Out of range for savedir names")
        else:
            self.is_saving_on = False
        self.set_viewport(viewport)
        self.set_robot(robot)

        self.sd_helper = SyntheticDataHelper()
        self.sd_helper.sensor_helpers["normals"] = omni.syntheticdata.sensors.get_normals
        self.sd_helper.sensor_types["normals"] = self.sd_helper.sd.SensorType.Normal
        self.sdh_topics = ["rgb", "depth", "instanceSegmentation", "normals"]
        asyncio.ensure_future(
            self.sd_helper.initialize_async(self.sdh_topics, self.viewport))
        self.cnt = 0
        return

    def set_viewport(self, viewport):
        if viewport is None:
            if "2021" in VERSION:
                viewport_handle = omni.kit.viewport.get_viewport_interface()
            else: 
                viewport_handle = omni.kit.viewport_legacy.get_viewport_interface()
            self.viewport = viewport_handle.get_viewport_window()
        else:
            self.viewport = viewport

    def set_robot(self, robot):
        self.robot=robot
        pass

    def get(self, topics):
        """ Returns a dict with all requested data. """
        sdh_topics = [t for t in topics if t in self.sdh_topics]
        non_sdh_topics = [t for t in topics if t not in self.sdh_topics]
        gt = self.sd_helper.get_groundtruth(
                                sdh_topics, 
                                self.viewport,
                                verify_sensor_init=False)
        for t in non_sdh_topics:
            if t == "joint_positions":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                gt[t] = self.robot.get_joint_positions().tolist()
            elif t == "applied_joint_positions":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                gt[t] =  self.robot.get_applied_action().joint_positions.tolist()
            elif t == "end_effector":
                if self.robot is None: raise ValueError("Robot topic requested but no robot is set")
                #gt[t] = self.robot.EE.get_world_pose()
                gt[t] = self.robot.EE.get_world_pose()
                #gt[t] = tf_matrix_from_pose(gt[t][0], gt[t][1])
            elif t == "camera":
                gt[t] = self.sd_helper.get_camera_params(self.viewport)
                gt[t]["clipping_range"] = (gt[t]["clipping_range"][0],gt[t]["clipping_range"][1])
                # prim = get_current_stage().GetPrimAtPath(
                #         "/World/CameraStand/Camera")
                # prim_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
                # prim_tf = np.asarray(prim_tf).T
                # gt[t] = prim_tf
            elif t == "normals":
                gt[t] = get_normals(self.viewport)
            else:
                warnings.warn("topic %t is not handled by SD Get Watch"%t)
        return copy(gt)
    def makedir(self, path):
        os.makedirs(path, exist_ok=True)

    def save_topics(self, gt, filenum):
        if not self.is_saving_on:
            raise ValueError("Cannot save topics without savedir")
        for t in list(gt.keys()):
            print ("Snap - Internal", t)
            self.makedir("%s/%s/"%(self.savedir,t))
            if t in ["rgb"]:
                im = Image.fromarray(gt[t])
                im.save("%s/%s/%s.png"%(self.savedir,t,filenum))
            elif t in ["depth"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), gt[t])
            elif t in ["instanceSegmentation"]:
                # im is List[
                #           (ID, Path, ID?, class, 
                #            List[sub part ids], 
                #            empty string?)]
                instance_data, instance_mapping = gt[t][0], gt[t][1]
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), gt[t])
                #np.save("%s/%s_map/%s.npy"%(self.savedir,t,filenum), instance_data)
            elif t in ["joint_positions", "applied_joint_positions"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filenum), gt[t])
            elif t in ["end_effector", "camera"]:
                ee = np.asarray(gt[t], dtype=object)
                np.save("%s/%s/%s"%(self.savedir,t,filenum), ee)
            else:
                warnings.warn("topic %s is not handled by SD Save Watch"%t)


    def snap(self, topics, filename=None, timestamp=None):
        if not self.is_saving_on:
            raise ValueError("Cannot save topics without savedir")
        # Make dir again, with subfolder. Only in runs when snap is called
        self.makedir(self.savedir) 
        self.cnt+=1
        if timestamp is not None:
            filename = str(timestamp)
            if filename is None:
                filename = "%d"%self.cnt

        gt = self.get(topics)
        for t in list(gt.keys()):
            print ("Snap - Internal", t)
            self.makedir("%s/%s/"%(self.savedir,t))
            if t in ["rgb"]:
                im = Image.fromarray(gt[t])
                im.save("%s/%s/%s.png"%(self.savedir,t,filename))
            elif t in ["depth"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filename), gt[t])
            elif t in ["instanceSegmentation"]:
                # im is List[
                #           (ID, Path, ID?, class, 
                #            List[sub part ids], 
                #            empty string?)]
                instance_data, instance_mapping = gt[t][0], gt[t][1]
                np.save("%s/%s/%s.npy"%(self.savedir,t,filename), instance_data)
            elif t in ["joint_positions", "applied_joint_positions"]:
                np.save("%s/%s/%s.npy"%(self.savedir,t,filename), gt[t])
            elif t in ["end_effector", "camera"]:
                ee = np.asarray(gt[t], dtype=object)
                np.save("%s/%s/%s"%(self.savedir,t,filename), ee)
            else:
                warnings.warn("topic %s is not handled by SD Save Watch"%t)
    
    async def snap_async(self, topics, filename=None, timestamp=None):
        self.snap(topics,filename, timestamp)
        return 
