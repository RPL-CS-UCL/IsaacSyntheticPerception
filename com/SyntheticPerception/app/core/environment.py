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
from matplotlib import pyplot as plt
from PIL import Image
import math
"""
TO DO


sort the reward
"""
def angle_between_vectors(v1, v2):
        """Compute the angle (in degrees) between two vectors."""
        dot_product = np.dot(v1, v2)
        magnitude_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        cosine_angle = dot_product / magnitude_product
        angle_rad = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        return angle_deg
def rotate_vector_by_quaternion(vec, quat):
    vec_quat = Gf.Quatf(0, vec)
    rotated_quat = quat * vec_quat * quat.GetInverse()
    return rotated_quat.GetImaginary()
class RunningStats:
    def __init__(self):
        self.n = 0
        self.mean = 0
        self.M2 = 0

    def update(self, x):
        """
        Update the running statistics with new data point x.
        
        Parameters:
            x (float): New data point.
        """
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2

    @property
    def variance(self):
        """
        Get the variance of the data seen so far.
        
        Returns:
            float: Variance.
        """
        if self.n < 2:
            return float('nan')
        return self.M2 / self.n

    @property
    def standard_deviation(self):
        """
        Get the standard deviation of the data seen so far.
        
        Returns:
            float: Standard deviation.
        """
        return self.variance ** 0.5
class Environment(gym.Env):
    """
    Class that represents the world, agents, and, objects that can exist in an environment
    """

    def __init__(self, action_repeat=1, size=(64, 64), seed=0, id = 0) -> None:
        # self._world = World()
        self._step = 0
        self._objects = []
        self._agents = []

        self._agent = None
        self._goal = None
        self._step = 0
        self._length = 1000
        self._angle_reward_steps = 0

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
        self._action_space = spaces.Discrete(10)
        self.threshold = 15

        self.env_id = id
        self.agent_alive = True

        # average tracking
        self.stats = RunningStats()


        """

        The following dictionary maps abstract actions from `self.action_space` to 

        the direction we will walk in if that action is taken.

        I.e. 0 corresponds to "right", 1 to "up" etc.

        """

        velocity = 3
        diag_veloc = velocity * math.sqrt(2) / 2
        self._action_to_direction = {
                # linear
                9: np.array([velocity, 0, 0, 0, 0, 0]),#forward
                1: np.array([-velocity, 0, 0, 0, 0, 0]),#back
                2: np.array([0, velocity, 0, 0, 0, 0]),#left
                3: np.array([0, -velocity, 0, 0, 0, 0]),#right

                4: np.array([diag_veloc, diag_veloc, 0, 0, 0, 0]),#forward
                5: np.array([-diag_veloc, -diag_veloc, 0, 0, 0, 0]),#back
                6: np.array([diag_veloc, -diag_veloc, 0, 0, 0, 0]),#left
                7: np.array([-diag_veloc, diag_veloc, 0, 0, 0, 0]),#right
                # angular
                8: np.array([0, 0, 0, 0,0,1]),#rotate right
                0: np.array([0, 0, 0,0,0,-1]),#rotate left
            }

    def setup_objects_agents_goals(self):
        pos = [20, 0, 0]
        rotation = [0, 0, 0,0]
        stage = omni.usd.get_context().get_stage()
        offset = self.env_id*2500
    
        parent_path = f"/World_{id}"
    
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
        planePath='/GroundPlane_',
        axis='Z',
        size=2500.0,
        position=Gf.Vec3f(offset, 0.0, 0.0),
        color=Gf.Vec3f(1., 1., 1.))

        omni.kit.commands.execute('ChangeProperty',
            prop_path=Sdf.Path('{parent_path}/GroundPlane.xformOp:scale'),
            value=Gf.Vec3f(500.0, 1.0, 1.0),
            prev=Gf.Vec3f(1.0, 1.0, 1.0))
            
      
        agent_loc, goal_loc = self.get_valid_random_spawn(offset=self.env_id*2500)
        self._agent = Agent(
            "/home/stuart/Downloads/sensors.json",
            agent_loc,
            rotation,
            [1.,1.,1.],
            "AGENT",
            parent_path,
            stage,
            disable_gravity=True,
            visibility="invisible",

        )
        print("tryin to create ojbect")

        usd_path = "/home/stuart/Downloads/cone.usd"
        self._goal_object = Object(
                goal_loc,
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
        #img = Image.fromarray(obs, 'RGB')  # 'L' means grayscale. For RGB, use 'RGB'
        #img.save('/home/stuart/Desktop/image.png')

        return {"image": obs,"is_terminal": False, "is_first": is_first}

    def _get_info(self):
        agent_pos = self._agent.get_translate()
        #dist_to_target = self._goal_pos - agent_pos
        goal_pos = self._goal_object.get_translate()
        x_diff = abs(agent_pos[0]-goal_pos[0])
        y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = x_diff + y_diff

        return {"discount": np.float32(0.997), "dist_to_target":dist}
    def simulate_steps(self):

        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)

    def pre_step(self,action):
        self._goal_pos = self._goal_object.get_translate()
        unpack_action = self._action_to_direction[action]
        linear_veloc = unpack_action[:3]
        angular_veloc = unpack_action[3:]
        self.agent_alive = self._agent.step(linear_veloc,angular_veloc)

    def post_step(self, action):
        #if self._step // 100:
        #    print("Action : ", action, " : ", self._action_to_direction[action])

        self._step +=1
        
        agent_alive = self.agent_alive
        self._done = (not agent_alive) or (self._length and self._step >= self._length)

        # ensure agent doesnt leave, if it does kill and reset
        # check if agent is at goal
        terminated = self._done #not agent_alive
        obs = self._get_obs()
        info = self._get_info()

        # ================= REWARD CALCULATIONS

        # set reward as dist to goal
        reward = 0  
        
        agent_pos = self._agent.get_translate()
        agent_old_pos = self._agent._last_translate
        goal_pos = self._goal_object.get_translate()
        #x_diff = abs(agent_pos[0]-goal_pos[0])
        #y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(goal_pos[:2]))
        old_dist = np.linalg.norm(np.asarray(agent_old_pos[:2]) - np.asarray(goal_pos[:2]))
        dist_since_previous_step = old_dist - dist
    
        # update the running stats
        self.stats.update(dist_since_previous_step)


        # dist = x_diff + y_diff

      

        # Define your points
        agent_point = self._agent.get_translate_vec()
        goal_point =self._goal_object.get_translate_vec()

        # Compute desired direction
        desired_direction = goal_point - agent_point
        desired_direction.Normalize()

        # Define agent orientation as a quaternion
        # For demonstration, this is a unit quaternion (no rotation).
        quat = self._agent.get_orientation_quat()

        # Extract forward direction from the quaternion
        # Assuming agent's initial forward direction is along z-axis (0, 0, 1)
        forward_direction = rotate_vector_by_quaternion(Gf.Vec3f(1, 0, 0), quat)

        # Convert Gf.Vec3f to numpy arrays for computation
        desired_direction_np = np.array([desired_direction[0], desired_direction[1], desired_direction[2]])
        forward_direction_np = np.array([forward_direction[0], forward_direction[1], forward_direction[2]])
        angle = abs(angle_between_vectors(desired_direction_np, forward_direction_np))
        #print(angle)
        reward = 0
        
        #         #max can get is 1
        # if angle < 45:
        #     angle_reward += (1/(angle+1e-8))/10# .1 if looking direct 

        # 0.0023
        #max can get is 1
        # if angle < 1:
        #     angle = 1
        # if angle < 45:
        #     angle_reward = (1/(angle+1e-8))/10# .1 if looking direct 
     
        #     self._angle_reward_steps += 1
           
        # else:
        #     self._angle_reward_steps = 0
        # if self._angle_reward_steps > 30:
        #     angle_reward /= (self._angle_reward_steps - 30)
        
        # ============== ANGLE REWARD
        angle_reward = 0#1e-20
        if angle < 45:
            angle_reward = (1/(angle+1e-8))#*10# .1 if looking direct 
        #reward += angle_reward
        normalized_angle_reward = np.cos(np.deg2rad(np.asarray(angle_reward)))

       
        # ======= DISTANCE REWARD
        #scaled dist to target
        # max 3.33 +-1
        # min 0.2 +- 1
        # if dist < 200:
        def normalize_progress(progress_reward):
            epsilon = 1e-10
            self.stats.update(progress_reward)
            if self.stats.standard_deviation < epsilon:
                # If the standard deviation is extremely small, just return the un-normalized reward.
                # This can happen, especially in the beginning when there's not much data.
                return progress_reward
            normalized_reward = (progress_reward - self.stats.mean) / (self.stats.standard_deviation + epsilon)
            return normalized_reward
        normalized_progress = normalize_progress(dist_since_previous_step)


        dist -= self.threshold - 1
        if dist <= 0:
            dist = 1
        normalized_distance = 1/(dist+1e-8)

        #reward += (50/(dist+1e-8)) + dist_since_previous_step
        w_distance = 0.2
        w_angle = 0.2
        w_progress = 0.50
        w_penalty = 0.1
        total_reward = (w_distance * normalized_distance +
                        w_angle * normalized_angle_reward +
                        w_progress * normalized_progress)
                        #w_penalty * self.get_step_penalty())


        

        # ======= PENALTY REWARD

        
        total_reward -= w_penalty# * self._ste)


#         print(f"""
# Normalized dist: {normalized_distance} 
# normalized angle reward:  {normalized_distance}
# Normalized progress: {normalized_progress}
# total_reward this step: {total_reward}
#               """
#               )
        
        #reward -= (self._step / self.length) * 10_000
        #reward -= 1/self._step
        # ==== apply all rewards
        reward += total_reward
        # ====== FINAL RWARD
        #max can get is 20
        #threshold = 15
        dist += self.threshold -1
        if dist < self.threshold:
            #print(" WE ARE NOW TERMINATING ",dist,  "  ", self.threshold)
            reward += 10_000
            terminated = True
        obs["is_terminal"] = terminated
    

        
        return obs, reward, terminated, info
    def step(self, action):
        #if self._step // 100:
        #    print("Action : ", action, " : ", self._action_to_direction[action])

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

        # ================= REWARD CALCULATIONS

        # set reward as dist to goal
        reward = 0  
        
        agent_pos = self._agent.get_translate()
        agent_old_pos = self._agent._last_translate
        goal_pos = self._goal_object.get_translate()
        #x_diff = abs(agent_pos[0]-goal_pos[0])
        #y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(goal_pos[:2]))
        old_dist = np.linalg.norm(np.asarray(agent_old_pos[:2]) - np.asarray(goal_pos[:2]))
        dist_since_previous_step = old_dist - dist
    
        # update the running stats
        self.stats.update(dist_since_previous_step)


        # dist = x_diff + y_diff

      

        # Define your points
        agent_point = self._agent.get_translate_vec()
        goal_point =self._goal_object.get_translate_vec()

        # Compute desired direction
        desired_direction = goal_point - agent_point
        desired_direction.Normalize()

        # Define agent orientation as a quaternion
        # For demonstration, this is a unit quaternion (no rotation).
        quat = self._agent.get_orientation_quat()

        # Extract forward direction from the quaternion
        # Assuming agent's initial forward direction is along z-axis (0, 0, 1)
        forward_direction = rotate_vector_by_quaternion(Gf.Vec3f(1, 0, 0), quat)

        # Convert Gf.Vec3f to numpy arrays for computation
        desired_direction_np = np.array([desired_direction[0], desired_direction[1], desired_direction[2]])
        forward_direction_np = np.array([forward_direction[0], forward_direction[1], forward_direction[2]])
        angle = abs(angle_between_vectors(desired_direction_np, forward_direction_np))
        #print(angle)
        reward = 0
        
        #         #max can get is 1
        # if angle < 45:
        #     angle_reward += (1/(angle+1e-8))/10# .1 if looking direct 

        # 0.0023
        #max can get is 1
        # if angle < 1:
        #     angle = 1
        # if angle < 45:
        #     angle_reward = (1/(angle+1e-8))/10# .1 if looking direct 
     
        #     self._angle_reward_steps += 1
           
        # else:
        #     self._angle_reward_steps = 0
        # if self._angle_reward_steps > 30:
        #     angle_reward /= (self._angle_reward_steps - 30)
        
        # ============== ANGLE REWARD
        angle_reward = 0#1e-20
        if angle < 45:
            angle_reward = (1/(angle+1e-8))#*10# .1 if looking direct 
        #reward += angle_reward
        normalized_angle_reward = np.cos(np.deg2rad(np.asarray(angle_reward)))

       
        # ======= DISTANCE REWARD
        #scaled dist to target
        # max 3.33 +-1
        # min 0.2 +- 1
        # if dist < 200:
        def normalize_progress(progress_reward):
            epsilon = 1e-10
            self.stats.update(progress_reward)
            if self.stats.standard_deviation < epsilon:
                # If the standard deviation is extremely small, just return the un-normalized reward.
                # This can happen, especially in the beginning when there's not much data.
                return progress_reward
            normalized_reward = (progress_reward - self.stats.mean) / (self.stats.standard_deviation + epsilon)
            return normalized_reward
        normalized_progress = normalize_progress(dist_since_previous_step)


        dist -= self.threshold - 1
        if dist <= 0:
            dist = 1
        normalized_distance = 1/(dist+1e-8)

        #reward += (50/(dist+1e-8)) + dist_since_previous_step
        w_distance = 0.2
        w_angle = 0.2
        w_progress = 0.50
        w_penalty = 0.1
        total_reward = (w_distance * normalized_distance +
                        w_angle * normalized_angle_reward +
                        w_progress * normalized_progress)
                        #w_penalty * self.get_step_penalty())


        

        # ======= PENALTY REWARD

        
        total_reward -= w_penalty# * self._ste)


#         print(f"""
# Normalized dist: {normalized_distance} 
# normalized angle reward:  {normalized_distance}
# Normalized progress: {normalized_progress}
# total_reward this step: {total_reward}
#               """
#               )
        
        #reward -= (self._step / self.length) * 10_000
        #reward -= 1/self._step
        # ==== apply all rewards
        reward += total_reward
        # ====== FINAL RWARD
        #max can get is 20
        #threshold = 15
        dist += self.threshold -1
        if dist < self.threshold:
            #print(" WE ARE NOW TERMINATING ",dist,  "  ", self.threshold)
            reward += 10_000
            terminated = True
        obs["is_terminal"] = terminated
    

        
        return obs, reward, terminated, info


    def get_valid_random_spawn(self, offset=0):
        range = 100
        valid_start = False
        agent_loc =[0,0,0]
        goal_loc = [0,0,0]
        while not valid_start:
            agent_loc = [np.random.uniform(0, range), np.random.uniform(0, range), 0]
            goal_loc = [np.random.uniform(0, range), np.random.uniform(0, range), 0]
            agent_loc[0] += offset
            goal_loc[0] += offset

            dist = np.linalg.norm(np.asarray(goal_loc[:2]) - np.asarray(agent_loc[:2]))
            if dist > (self.threshold + 10):
                valid_start = True
                break
        return agent_loc, goal_loc

    def reset(self):
        self._world.reset()
   
        self._step = 0
        agent_loc, goal_loc = self.get_valid_random_spawn()


        
        


        self._agent.change_start_and_reset(translate=agent_loc)
        self._goal_object.change_start_and_reset(translate=goal_loc)
        info = self._get_info()
        obs = self._get_obs()
        obs["is_terminal"] = self._step == 0 
        self._angle_reward_steps = 0
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
