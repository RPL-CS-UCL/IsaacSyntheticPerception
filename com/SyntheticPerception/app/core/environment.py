from omni.isaac.kit import SimulationApp

from pxr import Usd, Gf, UsdGeom
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)
import omni
from omni.isaac.core import World
from omni.isaac.quadruped.robots import Anymal
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom
import time
import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb
from omni.isaac.core.utils.extensions import enable_extension

class Environment:
    def __init__(self) -> None:
        # self._world = World()
        self._step = 0
        self._objects = []

        # ==================== Isaac params

    def action_space(self):
        pass

    def observation_space(self):
        pass

    def step(self):
        pass

    def reset(self):
        pass


