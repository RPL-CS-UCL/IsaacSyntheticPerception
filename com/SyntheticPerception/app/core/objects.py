from pxr import Usd, Gf, UsdGeom

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
from pxr import Sdf, Usd

# from omni.physx.scripts import utils as physx_utils
# from physxutils import *
# import .physxutils
from .physxutils import setRigidBody
from omni.isaac.core.objects import DynamicCuboid


class Object:
    def __init__(
        self,
        position,
        rotation,
        scale,
        prim_name,
        parent_path,
        stage,
        usd_path=None,
        semantic_class="None",
        instanceable=False,
        visibility = "inherited",
        gravity = True
    ) -> None:
        self._initial_translate = position
        self._initial_rotate = rotation
        self._initial_scale = scale

        self._usd_path = usd_path
        self._prim_name = prim_name
        self._prim_path = f"{parent_path}/{prim_name}"

        self._stage = stage
        self._scale = Gf.Vec3d(scale[0], scale[1], scale[2])
        self._translate = Gf.Vec3d(position[0], position[1], position[2])
        self._rotate = Gf.Vec3d(rotation[0], rotation[1], rotation[2])

        print("here")
        print(self._prim_name, "   ", usd_path)
        if not usd_path is None:
            add_reference_to_stage(usd_path=self._usd_path, prim_path=self._prim_path)
        else:
            # self._prim = XFormPrim(
            #     name=self._prim_name,
            #     prim_path=self._prim_path,
            #     position=position,
            # )

            self._prim = DynamicCuboid(
                    prim_path=self._prim_path,# The prim path of the cube in the USD stage
                    name=self._prim_name,  # The unique name used to retrieve the object from the scene later on
                    position=position,
                    scale=scale
                )

        self._prim = self._stage.GetPrimAtPath(self._prim_path)
        self._xform = UsdGeom.Xformable(self._prim)

        self._semantic_class = semantic_class
        if instanceable:
            self._prim.SetInstanceable(True)

        if not usd_path is None:
            self._translateOp = self._xform.AddTranslateOp()
            self._translateOp.Set(self._translate)

            self._rotateXYZOp = self._xform.AddRotateXYZOp()
            self._rotateXYZOp.Set(self._rotate)

            self._scaleOp = self._xform.AddScaleOp()
            self._scaleOp.Set(self._scale)

        sem = Semantics.SemanticsAPI.Apply(self._prim, "Semantics")
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set(self._semantic_class)
        if usd_path:
            setRigidBody(self._prim, "sdfMesh", False)
            # setRigidBody(self._prim, "convexHull", False)
        self._dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self._rb = self._dc_interface.get_rigid_body(self._prim_path)

        self._prim.GetAttribute('visibility').Set(visibility)
        self._prim.GetAttribute('physxRigidBody:disableGravity').Set(gravity)

    def apply_velocity(self, linear_veloc, angular_veloc) -> None:
        """
        Applys angular and or linear velocity to the object
        Args: linear_veloc: list[float], angular_veloc: list[float]
        Returns: None
        """

        self._rb = self._dc_interface.get_rigid_body(self._prim_path)
        self._dc_interface.set_rigid_body_linear_velocity(self._rb, linear_veloc)
        self._dc_interface.set_rigid_body_angular_velocity(self._rb, angular_veloc)

    def get_rotation(self) -> Gf.Vec3d:
        """
        Returns the rotation of the object.
        Args: None
        Returns: [] rotationXYZ
        """
        rotate = self._prim.GetAttribute("xformOp:rotateXYZ").Get()
        return [rotate[0], rotate[1], rotate[2]]

    def get_translate(self):
        """
        Returns the translation of the object.
        Args: None
        Returns: [] translation
        """
        translate = self._prim.GetAttribute("xformOp:translate").Get()
        return [translate[0], translate[1], translate[2]]

    def get_scale(self):
        """
        Returns the scale of the object.
        Args: None
        Returns: [] scale
        """
        scale = self._prim.GetAttribute("xformOp:scale").Get()
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

    def set_rotateXYZ(self, value):
        """
        Sets the rotation of the object.
        Args: [] rotation
        Returns: None
        """
        self._rotate = Gf.Vec3d(value[0], value[1], value[2])
        self._rotateXYZOp.Set(self._rotate)

    def reset(self):
        """
        Resets the translation, scale, and rotation of the object back to its start.
        Args: None
        Returns: None
        """
        self._translate = self._initial_translate
        self._rotation = self._initial_rotate
        self._scale = self._initial_scale
        self.set_scale(self._scale)
        self.set_translate(self._translate)
        self.set_rotateXYZ(self._rotate)

    def __repr__(self) -> str:
        output = f"===== object print ===== \nPrim Path: {self._prim_path} \nRotation: {self.get_rotation()} \nPosition: {self.get_translate()} \nscale: {self.get_rotation()}"
        return output
