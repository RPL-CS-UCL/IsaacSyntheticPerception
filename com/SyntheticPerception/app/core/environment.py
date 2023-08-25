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
from pxr import Gf, UsdGeom, Sdf
import time
import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb
from omni.isaac.core.utils.extensions import enable_extension
import gym
from gym import spaces
from core.rig import Agent
from core.objects import Object

"""
TO DO


sort the reward
"""


class Environment(gym.Env):
    """
    Class that represents the world, agents, and, objects that can exist in an environment
    """

    def __init__(self, action_repeat=1, size=(64, 64), seed=0) -> None:
        # self._world = World()
        self._step = 0
        self._objects = []
        self._agents = []

        self._agent = None
        self._goal = None
        self._step = 0
        self._length = 500

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
        self._action_space = spaces.Discrete(6)

        """

        The following dictionary maps abstract actions from `self.action_space` to 

        the direction we will walk in if that action is taken.

        I.e. 0 corresponds to "right", 1 to "up" etc.

        """

        
        self._action_to_direction = {
                # linear
                0: np.array([10, 0, 0, 0, 0, 0]),#forward
                1: np.array([-10, 0, 0, 0, 0, 0]),#back
                2: np.array([0, 10, 0, 0, 0, 0]),#left
                3: np.array([0, -10, 0, 0, 0, 0]),#right
                # angular
                4: np.array([0, 0, 0, 0,0,10]),#rotate right
                5: np.array([0, 0, 0,0,0,-10]),#rotate left
            }

    def setup_objects_agents_goals(self):
        pos = [20, 0, 0]
        rotation = [0, 0, 0,0]
       
        stage = omni.usd.get_context().get_stage()
    
        parent_path = "/World"
    
        # print(obj._prim.GetAttributes())
        # x =stage.GetPrimAtPath("/World/object")
        # print(x.GetAttributes())

        omni.kit.commands.execute('CreatePrimWithDefaultXform',
        prim_type='DistantLight',
        prim_path=None,
        attributes={'angle': 1.0, 'intensity': 3000},
        select_new_prim=True)

        omni.kit.commands.execute('AddGroundPlaneCommand',
        stage=stage,
        planePath='/GroundPlane',
        axis='Z',
        size=2500.0,
        position=Gf.Vec3f(0.0, 0.0, 0.0),
        color=Gf.Vec3f(1., 1., 1.))

        omni.kit.commands.execute('ChangeProperty',
            prop_path=Sdf.Path('/World/GroundPlane.xformOp:scale'),
            value=Gf.Vec3f(500.0, 1.0, 1.0),
            prev=Gf.Vec3f(1.0, 1.0, 1.0))
            
      

        self._agent = Agent(
            "/home/stuart/Downloads/sensors.json",
            pos,
            rotation,
            [1.0,1.0,1.0],
            "RIG",
            parent_path,
            stage,
            disable_gravity=True,
            visibility="invisible",

        )
        print("tryin to create ojbect")
        pos = [60, 30, 0]
        usd_path = "/home/stuart/Downloads/cone.usd"
        self._goal_object = Object(
                pos,
                rotation,
                [.2,.2,.2],
                "goal",
                parent_path,
                stage,
                usd_path=usd_path,
                instanceable=True,
            )
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
    @property
    def action_space(self):
        space = self._action_space
        space.discrete = True
        return space
    @property
    def observation_space(self):
        spaces = {}
        spaces['image'] = gym.spaces.Box(
            0, 255, self._size + (3,), dtype=np.uint8
        )
        return gym.spaces.Dict(spaces)

    # @property
    # def action_space(self):
    #     return self.action_space

    def _get_obs(self):
        # return {"image": np.zeroslike(self._size)}

        obs = self._agent.get_observations()[0]
        if len(obs) == 0:
            return {"image" : np.zeros((64,64,3))}
      
        obs = obs[:, :,  :-1]
     
        is_first = self._step == 0

        return {"image": obs,"is_terminal": False, "is_first": is_first}

    def _get_info(self):
        agent_pos = self._agent.get_translate()
        #dist_to_target = self._goal_pos - agent_pos
        goal_pos = self._goal_object.get_translate()
        x_diff = abs(agent_pos[0]-goal_pos[0])
        y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = x_diff + y_diff

        return {"discount": np.float32(0.997), "dist_to_target":dist}

    def step(self, action):

        self._step +=1
        
        self._goal_pos = self._goal_object.get_translate()
        unpack_action = self._action_to_direction[action]
        linear_veloc = unpack_action[:3]
        angular_veloc = unpack_action[3:]


        # STEP AGENT HERE
        agent_alive = self._agent.step(linear_veloc,angular_veloc)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._done = (not agent_alive) or (self._length and self._step >= self._length)


        # ensure agent doesnt leave, if it does kill and reset
        # check if agent is at goal
        terminated = self._done #not agent_alive
        obs = self._get_obs()
        info = self._get_info()

        # set reward as dist to goal
        reward = 0  
        threshold = 3
        agent_pos = self._agent.get_translate()
        goal_pos = self._goal_object.get_translate()
        x_diff = abs(agent_pos[0]-goal_pos[0])
        y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = x_diff + y_diff

        if dist < threshold:
            reward = 1
            terminated = True
        obs["is_terminal"] = terminated
    

        
        return obs, reward, terminated, info

    def reset(self):
        self._world.reset()
        range = 100
        self._agent.change_start_and_reset(translate=[np.random.uniform(0, range), np.random.uniform(0, range), 0])
        self._goal_object.change_start_and_reset(translate=[np.random.uniform(0, range), np.random.uniform(0, range), 0])
        info = self._get_info()
        obs = self._get_obs()
        obs["is_terminal"] = self._step == 0 
        return obs

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
