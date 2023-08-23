
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
from core.environment import Environment
from core.objects import Object
from core.rig import Agent 
class IsaacHandler:
    def __init__(self, physics_dt, render_dt, simulation_app) -> None:
        self.simulation_app = simulation_app


        enable_extension('omni.kit.asset_converter')
        self.simulation_app.update()
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
        )
        self._stage = omni.usd.get_context().get_stage()
        self._world.reset()
        self._needs_reset = False

        self.env = Environment()

    def setup(self):

        self._appwindow = omni.appwindow.get_default_app_window()
        # self._world.add_physics_callback(
        #     'AgentInteract', callback_fn=self.agent_interact
        # )

        self.env.setup()


    def step(self, render):
        if self._needs_reset:
            self._needs_reset = False
            self.env.reset()
            self._world.reset()
        
        # make the env step the environment
        self.env.step()
        # update the sim
        self._world.step(render=render)

    def run(self):
        while self.simulation_app.is_running():
            render = True
            self.step(render)
            if not self._world.is_simulating():
                self._needs_reset = True
