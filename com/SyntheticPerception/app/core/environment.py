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
    """
    Class that represents the world, agents, and, objects that can exist in an environment
    """
    def __init__(self) -> None:
        # self._world = World()
        self._step = 0
        self._objects = []
        self._agents = []

        # ==================== Isaac params

    def setup(self):
        """
        This method should setup the environemnt, models, and spawn all assets
        Args: None
        Returns: None
        """
        pass

    def action_space(self):
        pass

    def observation_space(self):
        pass

    def step(self):
        #update objects


        #update agents
        for agent in self._agents:
            #get observations from agent
            obs = agent.getobs()
            #__________________________ = model.get_action(obs)
            linear_veloc, angular_veloc = 0,0
            agent.step(linear_veloc, angular_veloc)




    def reset(self):
        """
        Resets all objects and agents in the environment
        Args: None
        Returns: None
        """
        for obj in self._objects:
            obj.reset()
        for agent in self._agents:
            agent.reset()


