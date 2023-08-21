from pxr import Usd, Gf, UsdGeom
from omni.isaac.core.utils.stage import (
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
    update_stage,
)

class World:
    def __init__(self) -> None:
        pass

    def get_height_map(self):
        pass

    def get_normal_map(self):
        pass

    def get_occupancy_map(self):
        pass


class Environment:
    def __init__(self) -> None:
        # self._world = World()
        self._step = 0 
        self._objects = []

    def action_space(self):
        pass

    def observation_space(self):
        pass

    def step(self, step_size):
        pass

    def reset(self):
        pass
