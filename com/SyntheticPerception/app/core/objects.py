from pxr import Sdf, Usd, Gf, UsdGeom

from omni.isaac.core.utils.stage import get_stage_units
from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
)
import omni
from omni.isaac.dynamic_control import _dynamic_control
from pxr import (
    UsdGeom,
    Gf,
    UsdPhysics,
    Semantics,
)  # pxr usd imports used to create cube
from core.physxutils import setRigidBody
import numpy as np


class Object:
    """
    A class that represents a phsyical entity in an environment. 
    Has methods to move with velocity and set translation,scale and orientation
    """
    def __init__(
        self,
        position,
        orientation,
        scale,
        prim_name,
        parent_path,
        stage,
        usd_path=None,
        semantic_class='None',
        instanceable=False,
        visibility='inherited',
        disable_gravity=True,
    ) -> None:

        self._usd_path = usd_path
        self._prim_name = prim_name
        self._prim_path = f'{parent_path}/{prim_name}'

        self._stage = stage
        self._scale = Gf.Vec3d(scale[0], scale[1], scale[2])
        self._translate = Gf.Vec3d(position[0], position[1], position[2])
        self._orientation = Gf.Quatf(
            orientation[0], orientation[1], orientation[2], orientation[3]
        )  # self._initial_orientation

        self._initial_translate = self._translate
        self._initial_scale = self._scale
        self._initial_orientation = self._orientation
        if not usd_path is None:
            print("LOADING USD ")
            add_reference_to_stage(
                usd_path=self._usd_path, prim_path=self._prim_path
            )
        else:
            print(" CREATING CUBE")
            omni.kit.commands.execute(
                'CreateMeshPrimWithDefaultXform',
                prim_type='Cube',
                prim_path=self._prim_path,
            )

        self._prim = self._stage.GetPrimAtPath(self._prim_path)
        self._xform = UsdGeom.Xformable(self._prim)
        self._semantic_class = semantic_class

        if instanceable:
            self._prim.SetInstanceable(True)

        if not usd_path is None:
            x = self._prim.GetAttribute(
            'xformOp:translate')
            if not x:
                self._translateOp = self._xform.AddTranslateOp()
            x = self._prim.GetAttribute(
            'xformOp:orient')
            if not x:
                self._orientOp = self._xform.AddOrientOp()
            x = self._prim.GetAttribute(
            'xformOp:scale')
            if not x:
                self._scaleOp = self._xform.AddScaleOp()

        self._translateOp = self._prim.GetAttribute(
            'xformOp:translate'
        )  
        self._orientOp = self._prim.GetAttribute('xformOp:orient')
        self._scaleOp = self._prim.GetAttribute('xformOp:scale')

        self._translateOp.Set(self._translate)
        self._orientOp.SetTypeName(Sdf.ValueTypeNames.Quatf)
        self._orientOp.Set(self._orientation)
        self._scaleOp.Set(self._scale)

        sem = Semantics.SemanticsAPI.Apply(self._prim, 'Semantics')
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set('class')
        sem.GetSemanticDataAttr().Set(self._semantic_class)

        if usd_path:
            setRigidBody(self._prim, 'convexHull', False)#sdfMesh
        else:
            setRigidBody(self._prim, 'convexHull', False)

        self._disable_gravity = disable_gravity
        self._dc_interface = (
            _dynamic_control.acquire_dynamic_control_interface()
        )
        self._rb = self._dc_interface.get_rigid_body(self._prim_path)
        #visibility = "inherited"
        self._prim.GetAttribute('visibility').Set(visibility)

        self._prim.GetAttribute('physxRigidBody:disableGravity').Set(
            self._disable_gravity
        )
        self.set_scale(self._scale)
        self.set_orient_quat(self._orientation)
        self.set_translate(self._translate)

    def apply_velocity(self, linear_veloc, angular_veloc) -> None:
        """
        Applys angular and or linear velocity to the object
        Args: linear_veloc: list[float], angular_veloc: list[float]
        Returns: None
        """

        self._rb = self._dc_interface.get_rigid_body(self._prim_path)
        self._dc_interface.set_rigid_body_linear_velocity(
            self._rb, linear_veloc
        )
        self._dc_interface.set_rigid_body_angular_velocity(
            self._rb, angular_veloc
        )


    def get_orientation(self) -> Gf.Vec3d:
        """
        Returns the orientation of the object.
        Args: None
        Returns: [] orientation 
        """
        orient = self._prim.GetAttribute('xformOp:orient').Get()
        quat_list = [orient.GetReal()] + list(orient.GetImaginary())
        return quat_list #[orient[0], orient[1], orient[2], orient[3]]
    def get_orientation_quat(self):
        """
        Returns the orientation of the object.
        Args: None
        Returns: [] orientation 
        """
        orient = self._prim.GetAttribute('xformOp:orient').Get()
        return orient

    def get_translate(self):
        """
        Returns the translation of the object.
        Args: None
        Returns: [] translation
        """
        translate = self._prim.GetAttribute('xformOp:translate').Get()
        return [translate[0], translate[1], translate[2]]
    def get_translate_vec(self):
        """
        Returns the translation of the object.
        Args: None
        Returns: [] translation
        """
        translate = self._prim.GetAttribute('xformOp:translate').Get()
        return translate

    def get_scale(self):
        """
        Returns the scale of the object.
        Args: None
        Returns: [] scale
        """
        scale = self._prim.GetAttribute('xformOp:scale').Get()
        return [scale[0], scale[1], scale[2]]

    def set_scale(self, value):
        """
        Sets the scale of the object.
        Args: [] scale
        Returns: None
        """
        self._scale = Gf.Vec3d(value[0], value[1], value[2])
        self._scaleOp.Set(self._scale)

    def set_translate(self, value):
        """
        Sets the translation of the object.
        Args: [] translation
        Returns: None
        """
        self._translate = Gf.Vec3d(value[0], value[1], value[2])
        self._translateOp.Set(self._translate)


    def set_orient(self, value):
        """
        Sets the orientation of the object.
        Args: [] orientation
        Returns: None
        """
        # value = value.astype(np.float32)
        self._orientation = Gf.Quatf(
            float(value[0]), float(value[1]), float(value[2]), float(value[3])
        )
        self._orientOp.Set(self._orientation)

    def set_orient_quat(self, value):
        """
        Sets the orientation of the object.
        Args: Gf.Quatf() orienation 
        Returns: None
        """
        self._orientation = value
        self._orientOp.Set(self._orientation)



    def reset(self):
        """
        Resets the translation, scale, and rotation of the object back to its start.
        Args: None
        Returns: None
        """
        self._translate = self._initial_translate
        self._scale = self._initial_scale
        self._orientation = self._orientation
        self.set_scale(self._scale)
        self.set_translate(self._translate)
        self.set_orient_quat(self._orientation)

    def change_start_and_reset(self, translate=None,scale=None,orientation=None):
        """
        Changes and resets the object to a new initial positon,orientation,scale
        Args: translate [], scale [], orient []
        Returns: None
        """
        if translate:
            self._initial_translate = translate

        if scale:
            self._initial_scale = scale

        if orientation:
            self._initial_orientation = orientation

        self.reset()

    def __repr__(self) -> str:
        output = f'===== object print ===== \nPrim Path: {self._prim_path} \nRotation: {self.get_rotation()} \nPosition: {self.get_translate()} \nscale: {self.get_rotation()}'
        return output
