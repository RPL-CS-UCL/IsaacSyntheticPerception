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
import gym
from gym import spaces


"""
TO DO


fix the mappings from discrete to velocities
fix the info and get observations
sort the reward
"""


class Environment:
    """
    Class that represents the world, agents, and, objects that can exist in an environment
    """

    def __init__(self, name, action_repeat=1, size=(64, 64), seed=0) -> None:
        # self._world = World()
        self._step = 0
        self._objects = []
        self._agents = []
        self._agent = None
        physics_dt = 1/60
        render_dt = 1/60
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
        )
        # ### copied params
        self._size = size
        self._action_repeat = action_repeat
        self.reward_range = [-np.inf, np.inf]
        self.action_space = spaces.Discrete(4)

        """

        The following dictionary maps abstract actions from `self.action_space` to 

        the direction we will walk in if that action is taken.

        I.e. 0 corresponds to "right", 1 to "up" etc.

        """

        self._action_to_direction = {
            # linear
            0: np.array([1, 0, 0], [0, 0, 0]),
            1: np.array([0, 1, 0]),
            2: np.array([-1, 0, 0]),
            3: np.array([0, -1, 0]),
            # angular
            4: np.array([0, 0, 0], [1, 1, 1]),
            5: np.array([0, 1, 0]),
            6: np.array([-1, 0, 0]),
            7: np.array([0, -1, 0]),
        }

    @property
    def observation_space(self):
        spaces = {}
        spaces['image'] = gym.spaces.Box(
            0, 255, self._size + (3,), dtype=np.uint8
        )
        return gym.spaces.Dict(spaces)

    @property
    def action_space(self):
        return self.action_space

    def _get_obs(self):
        # return {"image": np.zeroslike(self._size)}
        obs = self._agent.get_observations()[0]
        return

    def _get_info(self):
        agent_pos = self._agent.get_translation()
        dist_to_target = 0 - agent_pos
        return {"discount": 0, "dist_to_target":0}

    def step(self, action):
        unpack_action = self._action_to_direction[action]
        linear_veloc = unpack_action[0]
        angular_veloc = unpack_action[1]


        # STEP AGENT HERE
        agent_alive = self._agent.step(linear_veloc,angular_veloc)
        self._world.step(render=True)

        # ensure agent doesnt leave, if it does kill and reset
        goal = [0, 0, 0]
        # check if agent is at goal
        terminated = not agent_alive


        obs = self._get_obs()
        info = self._get_info()

        # set reward as dist to goal
        reward = 1

        return obs, reward, terminated, False, info

    def reset(self):
        self._world.reset()
        self._agent.reset()
        info = self._get_info()
        obs = self._get_obs()
        return obs, info

    def render(self, *args, **kwargs):
        return None

    # def setup(self):
    #     """
    #     This method should setup the environemnt, models, and spawn all assets
    #     Args: None
    #     Returns: None
    #     """
    #     pass
    #
    # def action_space(self):
    #     pass
    #
    # def observation_space(self):
    #     pass
    #
    # def step(self):
    #     #update objects
    #
    #
    #     #update agents
    #     for agent in self._agents:
    #         #get observations from agent
    #         obs = agent.getobs()
    #         #__________________________ = model.get_action(obs)
    #         linear_veloc, angular_veloc = 0,0
    #         agent.step(linear_veloc, angular_veloc)
    #
    #
    #
    #
    # def reset(self):
    #     """
    #     Resets all objects and agents in the environment
    #     Args: None
    #     Returns: None
    #     """
    #     for obj in self._objects:
    #         obj.reset()
    #     for agent in self._agents:
    #         agent.reset()
    #
    #


    """

class IsaacHandler:
    def __init__(self, physics_dt, render_dt, simulation_app) -> None:
        self.simulation_app = simulation_app


        enable_extension('omni.kit.asset_converter')
        self.simulation_app.update()
        # self._world = World(
        #     stage_units_in_meters=1.0,
        #     physics_dt=physics_dt,
        #     rendering_dt=render_dt,
        # )
        self._stage = omni.usd.get_context().get_stage()
        # self._world.reset()
        self._needs_reset = False

        self.env = Environment("sim")

    def setup(self):

        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback(
            'AgentInteract', callback_fn=self.agent_interact
        )

        self.env.setup()


    def step(self, render):
        if self._needs_reset:
            self._needs_reset = False
            self.env.reset()
        
        # make the env step the environment
        self.env.step()
        # self.env._world.step(render=render)

    def run(self):
        while self.simulation_app.is_running():
            render = True
            self.step(render)
            if not self._world.is_simulating():
                self._needs_reset = True
    """
