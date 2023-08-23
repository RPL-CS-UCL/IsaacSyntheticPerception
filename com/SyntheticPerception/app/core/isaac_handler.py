
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
from .environment import Environment
from .objects import Object
from .rig import Rig
class IsaacHandler:
    def __init__(self, physics_dt, render_dt) -> None:
        self.simulation_app = SimulationApp({'headless': False})


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
        self._world.add_physics_callback(
            'AgentInteract', callback_fn=self.agent_interact
        )

        pos = [0, 0, 0]
        orientation = [0, 0, 0,0]
        usd_path = "/home/jon/Documents/IsaacContent/ov-vegetation3dpack-01.100.1.0.linux-x86_64-ent-package/Trees/Black_Oak.usd"

        prim_name = "object222"
        parent_path = "/World"
        scale = [1.0, 1.0, 1.0]
        obj = Object(
            pos,
            orientation,
            scale,
            prim_name,
            parent_path,
            self._stage,
            usd_path=usd_path,
            instanceable=True,
        )
        # print(obj._prim.GetAttributes())
        # x =stage.GetPrimAtPath("/World/object")
        # print(x.GetAttributes())

        self.testrig = Rig(
            "/home/jon/Downloads/sensors.json",
            pos,
            orientation,
            scale,
            "RIG",
            parent_path,
            self._stage,
            disable_gravity=True,
            visibility="invisible",

        )

    def agent_interact(self, step_size):
        pass

    def step(self, render):
        if self._needs_reset:
            self._needs_reset = False
            self.env.reset()
            self._world.reset()

        self._world.step(render=render)
        self.env.step()

    def run(self):
        while self.simulation_app.is_running():
            render = True
            self.step(render)
            if not self._world.is_simulating():
                self._needs_reset = True
