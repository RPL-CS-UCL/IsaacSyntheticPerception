from omni.isaac.kit import SimulationApp

from omni.physx import get_physx_scene_query_interface

from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics
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
import random
from scipy.spatial.transform import Rotation
from pxr import UsdShade, Sdf
import cv2

def rotate_quaternion_by_angle(original_quaternion, angle_degrees, axis_of_rotation):
    angle_radians = np.deg2rad(angle_degrees)
    half_angle = angle_radians / 2.0
    axis_of_rotation /= np.linalg.norm(axis_of_rotation)
    rotation_quaternion = np.array(
        [
            np.cos(half_angle),
            axis_of_rotation[0] * np.sin(half_angle),
            axis_of_rotation[1] * np.sin(half_angle),
            axis_of_rotation[2] * np.sin(half_angle),
        ]
    )
    original_quaternion /= np.linalg.norm(original_quaternion)
    rotation_quaternion /= np.linalg.norm(rotation_quaternion)
    t = 0.5  # You can adjust this value to control the interpolation
    result_quaternion = slerp(original_quaternion, rotation_quaternion, t)
    return result_quaternion


def slerp(q1, q2, t):
    dot_product = np.dot(q1, q2)
    if dot_product < 0.0:
        q2 = -q2
        dot_product = -dot_product
    if dot_product > 0.95:
        result = q1 + t * (q2 - q1)
    else:
        angle = np.arccos(dot_product)
        sin_angle = np.sin(angle)
        result = (np.sin((1.0 - t) * angle) * q1 + np.sin(t * angle) * q2) / sin_angle
    return result


def angle_to_align_vectors(v1, v2):
    # Calculate the dot product of the two vectors
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    # Calculate the cross product of the two vectors
    cross_product = v1[0] * v2[1] - v1[1] * v2[0]
    # Calculate the signed angle using the arctan2 function
    signed_angle = math.atan2(cross_product, dot_product)
    if abs(signed_angle) > math.pi - 1e-6:
        signed_angle = math.pi - signed_angle
    return math.degrees(signed_angle)


def signed_angle_between_vectors(v1, v2):
    # Calculate the angle of each vector relative to the positive x-axis
    angle1 = math.atan2(v1[1], v1[0])
    angle2 = math.atan2(v2[1], v2[0])
    # Calculate the signed angle between the two vectors
    signed_angle = angle2 - angle1
    # Ensure the result is in the range [-pi, pi]
    if signed_angle > math.pi:
        signed_angle -= 2 * math.pi

    elif signed_angle <= -math.pi:
        signed_angle += 2 * math.pi

    return math.degrees(signed_angle)


def create_materials(mat_paths, scale, stage):
    mtl_shades = []
    mtl_created_list = []
    for mat_path in mat_paths:
        mat_name = mat_path.split("/")
        mat_name = mat_name[-1].split(".")[0]
        print(mat_name, " = ", mat_path)

        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name=mat_path,
            mtl_name=mat_name,
            mtl_created_list=mtl_created_list,
        )
        mtl_prim = stage.GetPrimAtPath(mtl_created_list[-1])
        omni.usd.create_material_input(
            mtl_prim,
            "project_uvw",
            True,
            Sdf.ValueTypeNames.Bool,
        )

        omni.usd.create_material_input(
            mtl_prim,
            "texture_scale",
            Gf.Vec2f(scale, scale),
            Sdf.ValueTypeNames.Float2,
        )
        mtl_shades.append(UsdShade.Material(mtl_prim))

    return mtl_shades


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

def vec3d_to_list(gfvec):
    return [gfvec[0], gfvec[1], gfvec[2]]


def list_to_vec3d(listvec):
    return Gf.Vec3d(float(listvec[0]), float(listvec[1]), float(listvec[2]))



class Environment(gym.Env):
    """
    Class that represents the world, agents, and, objects that can exist in an environment
    """
    def __init__(self, action_repeat=1, size=(64, 64), seed=0, id=0) -> None:
        print(" INIT GETING CALLED ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^6")
        # self._world = World()
        self._step = 0
        self._objects = []
        self._obstacles = []
        self._agents = []

        self.id = id
        self._agent = None
        self._goal = None
        self._step = 0
        self._length = 500  # 1000
        self._max_length = 2000
        self._angle_reward_steps = 0

        physics_dt = 1 / 60
        render_dt = 1 / 60
        # ### copied params
        self._size = size
        self._action_repeat = action_repeat
        self.reward_range = [-np.inf, np.inf]
        self._action_space = spaces.Discrete(6)
        self.threshold = 0.8

        self.env_id = id
        self.agent_alive = True
        self._collision_reset = False
        self._interacted_with_object = False
        self._goal_achieved = False

        # Values for learning
        self._first_time = True
        self._num_obstacles = 0
        self._num_obstacles_limit = 15
        self._random_starting_orientation = False
        self._size_of_map = 2
        self._map_limit = 44
        self._minimum_distance_between_objects = 1
        self._distance_between_objects_limit = 10
        self.min_dist_between_inter_and_goal = 0.5
        self._min_dist_between_inter_and_goal_limit = 10
        self.max_dist_between_inter_and_goal = 2
        self._max_dist_between_inter_and_goal_limit = 10
        self.min_dist_between_inter_and_agent = 0.5
        self._min_dist_between_inter_and_agent_limit = 10
        self.max_dist_between_inter_and_agent = 2
        self._max_dist_between_inter_and_agent_limit = 10

        velocity = 0.5 # 5
        self._velocity = velocity
        self._max_movement_per_step = self._velocity * 1 / 10
        diag_veloc = velocity * math.sqrt(2) / 2
        self._action_to_direction = {
            # linear
            0: np.array([velocity, 0, 0, 0, 0, 0]),  # forward
            1: np.array([-velocity, 0, 0, 0, 0, 0]),  # back
            2: np.array([0, velocity, 0, 0, 0, 0]),  # left
            3: np.array([0, -velocity, 0, 0, 0, 0]),  # right
            # 4: np.array([diag_veloc, diag_veloc, 0, 0, 0, 0]),  # forward
            # 5: np.array([-diag_veloc, -diag_veloc, 0, 0, 0, 0]),  # back
            # 6: np.array([diag_veloc, -diag_veloc, 0, 0, 0, 0]),  # left
            # 7: np.array([-diag_veloc, diag_veloc, 0, 0, 0, 0]),  # right
            # angular
            4: np.array([0, 0, 0, 0, 0, 1]),  # rotate right
            5: np.array([0, 0, 0, 0, 0, -1]),  # rotate left
        }
        self._world = None

    def set_velocity(self, velocity, episode_length):
        self._length = episode_length
        self._action_to_direction = {
                # linear
                0: np.array([velocity, 0, 0, 0, 0, 0]),  # forward
                1: np.array([-velocity, 0, 0, 0, 0, 0]),  # back
                2: np.array([0, velocity, 0, 0, 0, 0]),  # left
                3: np.array([0, -velocity, 0, 0, 0, 0]),  # right
                # 4: np.array([diag_veloc, diag_veloc, 0, 0, 0, 0]),  # forward
                # 5: np.array([-diag_veloc, -diag_veloc, 0, 0, 0, 0]),  # back
                # 6: np.array([diag_veloc, -diag_veloc, 0, 0, 0, 0]),  # left
                # 7: np.array([-diag_veloc, diag_veloc, 0, 0, 0, 0]),  # right
                # angular
                4: np.array([0, 0, 0, 0, 0, 1]),  # rotate right
                5: np.array([0, 0, 0, 0, 0, -1]),  # rotate left
        }
    def reset_velocity(self):
        self.set_velocity(self._velocity, 500)        

    def set_curriculum_values(
        self,
        map_size=None,
        random_starting_orientation=None,
        num_obstacles=None,
        min_dist_between_objs=None,
        episode_length=None,
        reward_first_time=None
        # min_dist_between_inter_and_goal=None,
        # max_dist_between_inter_and_goal=None,
        # min_dist_between_inter_and_agent=None,
        # max_dist_between_inter_and_agent=None
        
    ):
        if map_size is not None:
            self._size_of_map = map_size
        if random_starting_orientation is not None:
            self._random_starting_orientation = random_starting_orientation
        if num_obstacles is not None:
            self._num_obstacles = num_obstacles
        if min_dist_between_objs is not None:
            self._minimum_distance_between_objects = min_dist_between_objs
        if episode_length is not None:
            self._length = episode_length
        if reward_first_time is not None:
            self._first_time = reward_first_time
        # if min_dist_between_inter_and_goal is not None:
        #     self.min_dist_between_inter_and_goal = min_dist_between_inter_and_goal
        # if max_dist_between_inter_and_goal is not None:
        #     self.max_dist_between_inter_and_goal = max_dist_between_inter_and_goal
        # if min_dist_between_inter_and_agent is not None:
        #     self.min_dist_between_inter_and_agent = min_dist_between_inter_and_agent
        # if max_dist_between_inter_and_agent is not None:
        #     self.max_dist_between_inter_and_agent = max_dist_between_inter_and_agent

    def get_curriculum_values(self):
        curriculum_values = (self._size_of_map,
                            self._random_starting_orientation,
                            self._num_obstacles,
                            self._minimum_distance_between_objects,
                            self._length,
                            self._first_time
                            # self.min_dist_between_inter_and_goal,
                            # self.max_dist_between_inter_and_goal,
                            # self.min_dist_between_inter_and_agent,
                            # self.max_dist_between_inter_and_agent
        )    
        return curriculum_values 
   
    @property
    def map_limit(self):
        return self._map_limit
    
    @property
    def num_obstacles_limit(self):
        return self._num_obstacles_limit

    @property
    def distance_between_objects_limit(self):
        return self._distance_between_objects_limit
    
    @property
    def max_length(self):
        return self._max_length
    
    @property
    def reward_first_time(self):
        return self._first_time
    
    # @property
    # def min_dist_between_inter_and_goal_limit(self):
    #     return self._min_dist_between_inter_and_goal_limit
    
    # @property
    # def max_dist_between_inter_and_goal_limit(self):
    #     return self._max_dist_between_inter_and_goal_limit
    
    # @property
    # def min_dist_between_inter_and_agent_limit(self):
    #     return self._min_dist_between_inter_and_agent_limit
    
    # @property
    # def max_dist_between_inter_and_agent_limit(self):
    #     return self._max_dist_between_inter_and_agent_limit


    def setup_light(self, skybox_path=None):
        self._world.reset()

        stage = omni.usd.get_context().get_stage()
        if skybox_path:
            rotation = [0, 0, 0, 0]

            add_reference_to_stage(usd_path=skybox_path, prim_path="/World/sky")
        else:
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="DistantLight",
                prim_path=None,
                attributes={"angle": 1.0, "intensity": 3000},
                select_new_prim=True,
            )

    def curriculum_coords_check(self, n, d, s):
        """
        n = number of objects
        d = minimum dist
        s = size of map
        """
        if n > (s * s) // (d * d):
            return True 
        else:
            return False


    def generate_points(self, n, d, s, offset=0):
        """
        n = number of objects
        d = minimum dist
        s = size of map
        """
        if n > (s * s) // (d * d):
            raise ValueError("Cannot generate all points with the given constraints.")
        points = set()
        def generate_point():
                while True:
                    x = random.uniform(-s, s)
                    y = random.uniform(-s, s)
                    grid_x = int(x // d)
                    grid_y = int(y // d)
                    if (grid_x, grid_y) not in points:
                        points.add((grid_x, grid_y))
                        yield (x, y) 
        point_generator = generate_point()
        while len(points) < n:
            x, y = next(point_generator)
            points.add((int(x // d), int(y // d)))

        return [[(x * d)+offset, y * d, 0] for x, y in points]

    def setup_objects_agents_goals(
        self,
        world,
        id,
        cone_path=None,
        sensor_path=None,
        mat_path=None,
        table_path=None,
        obstacle_path=None,
        interaction_object=None,
        target_object=None
    ):
        print(" ========== we are spawning ", self._num_obstacles)
        # self._length = 1000
        self._world = world
        self.env_id = id
        self.id = id
        pos = [20, 0, 0]
        rotation = [0, 0, 0, 0]
        stage = omni.usd.get_context().get_stage()
        offset = self.env_id * 2500

        generate_points = self.generate_points(self._num_obstacles+3, self._minimum_distance_between_objects, self._size_of_map, offset=self.env_id * 2500)
        parent_path = f"/World/env_{self.env_id}"  # _{id}"

        omni.kit.commands.execute(
            "AddGroundPlaneCommand",
            stage=stage,
            planePath=f"{parent_path}/GroundPlane",
            axis="Z",
            size=500.0,
            position=Gf.Vec3f(offset, 0.0, 0.0),
            color=Gf.Vec3f(1.0, 1.0, 1.0),
        )

        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=Sdf.Path(f"{parent_path}/GroundPlane.xformOp:scale"),
            value=Gf.Vec3f(1.0, 1.0, 1.0),
            prev=Gf.Vec3f(1.0, 1.0, 1.0),
        )

        self._world.step(render=True)
        self._world.step(render=True)

        self.ground_prim = stage.GetPrimAtPath(f"{parent_path}/GroundPlane")
        self._materials = create_materials(mat_path, 0.1, stage)
        print(self._materials)
        mat_index = random.randint(0, len(self._materials) - 1)
        print(mat_index)
        UsdShade.MaterialBindingAPI(self.ground_prim).Bind(
            self._materials[mat_index], UsdShade.Tokens.strongerThanDescendants
        )

        # agent_loc, goal_loc = self.get_valid_random_spawn(offset=self.env_id * 2500)
        agent_loc = generate_points[0]
        goal_loc = generate_points[1]
        print("trying to get valid agent and goal loc", agent_loc, goal_loc)

        self._agent = Agent(
            sensor_path,  # "/home/jon/Documents/Isaac_dreamer/sensors.json",
            agent_loc,
            rotation,
            [0.3, 0.3, 0.7],
            "AGENT",
            parent_path,
            stage,
            disable_gravity=True,
            visibility="invisible",
        )

        self.target_image = target_object
        self.interaction_image = interaction_object
        

        usd_path = cone_path
        self._goal_object = Object(
            goal_loc,
            rotation,
            [0.01, 0.01, 0.01],
            "goal",
            parent_path,
            stage,
            usd_path=table_path,
            instanceable=False,
        )
        self.simulate_steps()
        # self._obstacles = []
        # self._locations_to_avoid = []
        # self._locations_to_avoid.append(goal_loc)
        # self._locations_to_avoid.append(agent_loc)
        # interaction_obj_loc = self.get_random_loc_interaction_object(goal_loc,agent_loc, offset=self.env_id * 2500
        #     )
        interaction_obj_loc = generate_points[2]
        print("interaction obj loc ", interaction_obj_loc)
        if interaction_obj_loc:
        
            self._interaction_obj= Object(
                interaction_obj_loc,
                rotation,
                [0.007, 0.007, 0.007],
                "interaction_obj",
                parent_path,
                stage,
                usd_path=obstacle_path,
                instanceable=True,
                disable_gravity=False
            )


        self.contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(
            self._agent._prim
        )
        self.contactReportAPI2 = PhysxSchema.PhysxContactReportAPI.Apply(
            self._interaction_obj._prim
        )

        self.contact_report_sub = (
            get_physx_simulation_interface().subscribe_contact_report_events(
                self.on_contact_report_event
            )
        )
        # self._obstacles = []
        # self._locations_to_avoid = []
        # self._locations_to_avoid.append(goal_loc)
        # self._locations_to_avoid.append(agent_loc)
        # self._locations_to_avoid.append(interaction_obj_loc)
        # print(goal_loc, agent_loc, interaction_obj_loc)
        self._obstacle_parent_path = parent_path
        self._stage = stage
        self._cone_path = cone_path
        if obstacle_path:
            for i in range(self._num_obstacles):
                # obs_loc = self.get_random_obstacle_loc(
                #     offset=self.env_id * 2500
                # )
                obs_loc = generate_points[3+i]
                if obs_loc is None:
                    continue
                #self._locations_to_avoid.append(obs_loc)
                self._obstacles.append(
                    Object(
                        obs_loc,
                        rotation,
                        [0.01, 0.01, 0.01],
                        f"obstacle_{i}",
                        parent_path,
                        stage,
                        usd_path=cone_path,
                        instanceable=False,
                    )
                )

    def on_contact_report_event(self, contact_headers, contact_data):
        for contact_header in contact_headers:
            act0_path = str(PhysicsSchemaTools.intToSdfPath(contact_header.actor0))
            act1_path = str(PhysicsSchemaTools.intToSdfPath(contact_header.actor1))
            if "AGENT"  in act0_path or "AGENT"  in act1_path: 

                cur_collider = str(
                    PhysicsSchemaTools.intToSdfPath(contact_header.collider0)
                )
                contact_data_offset = contact_header.contact_data_offset

                num_contact_data = contact_header.num_contact_data
                for index in range(
                    contact_data_offset, contact_data_offset + num_contact_data, 1
                ):
                    cur_contact = contact_data[index]
                    # print("collision")
                    env_str = f"env_{self.id}"
                    if env_str in act1_path and env_str in act0_path:
                        if "obs" in act0_path or "obs" in act1_path:
                            self._collision_reset = True
                        if "goal" in act0_path or "goal" in act1_path:
                            self._collision_reset = True
                        if "interaction" in act0_path or "interaction" in act1_path:
                            self._interacted_with_object = True
                            # print(env_str, " collided on step ", self._step)
            elif "AGENT" not in act0_path and "AGENT" not in act1_path:
                cur_collider = str(
                PhysicsSchemaTools.intToSdfPath(contact_header.collider0)
                )
                contact_data_offset = contact_header.contact_data_offset

                num_contact_data = contact_header.num_contact_data
                for index in range(
                    contact_data_offset, contact_data_offset + num_contact_data, 1
                ):
                    cur_contact = contact_data[index]
                    # print("collision")
                    env_str = f"env_{self.id}"
                    if env_str in act1_path and env_str in act0_path:
                        if "goal" in act0_path or "goal" in act1_path:
                            #print("interaction object colliding with goal")
                            # self._collision_reset = True
                            self._goal_achieved = True
                            print("act0_path", act0_path)
                            print("act1_path", act1_path)


    @property
    def action_space(self):
        space = self._action_space
        space.discrete = True
        return space

    @property
    def observation_space(self):
        spaces = {}
        spaces["image"] = gym.spaces.Box(0, 255, self._size + (3,), dtype=np.float32)
        spaces["target"] = gym.spaces.Box(0, 255, self._size + (3,), dtype=np.float32)
        spaces["interaction"] = gym.spaces.Box(0, 255, self._size + (3,), dtype=np.float32)
        spaces["dist_to_target"] = gym.spaces.Box(
            -np.inf, np.inf, (1,), dtype=np.float32
        )
        spaces["dist_to_object"] = gym.spaces.Box(
            -np.inf, np.inf, (1,), dtype=np.float32
        )

        spaces["depth"] = gym.spaces.Box(0.0,np.inf , self._size + (1,), dtype=np.float32)

        return gym.spaces.Dict(spaces)
    def _add_gaus_noise(self, arr, max_val):
        mean = 0
        stddev = 0.5  # You can adjust this value to control the noise level

 

        noise = np.random.normal(mean, stddev, arr.shape)
        noisy_array = arr + noise

 

        noisy_array = np.clip(noisy_array, 0, max_val)
        return noisy_array

 

    def _add_color_noise(self, image):
        mean = 0  # Mean for Gaussian noise
        stddev = 0.05  # Standard deviation for Gaussian noise
        speckle_intensity = 0.05  # Intensity of speckle noise

 

        # Generate Gaussian noise
        gaussian_noise = np.random.normal(mean, stddev, image.shape).astype(np.uint8)

 

        # Apply Gaussian noise to the image
        noisy_image = cv2.add(image, gaussian_noise)

 

        # # Generate speckle noise
        # speckle_noise = np.random.randn(*image.shape) * speckle_intensity
        #
        # # Apply speckle noise to the image
        # noisy_image = noisy_image + noisy_image * speckle_noise
        #
        # # Clip the values to ensure they are within a valid range (0-255)
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)
        return noisy_image

 

    def _get_obs(self,  dist,dist_to_obj):
        # print("here")

 

        obs, depth = self._agent.get_observations()[0]
        # print(obs)
        depth = depth.reshape(self._size + (1,))

 
        # if self._step > 500:
        #     print("HERE STOP NOW")
        #     np.save("/home/stuart/Desktop/microwave.npy", obs)
        # #
        depth[depth > 10] = 10  # 400#1500#np.finfo(np.float32).max
        depth = self._add_gaus_noise(depth, 10)
        # depth_min = np.min(depth)
        # depth_max = np.max(depth)
        # depth= (depth-depth_min)/(depth_max-depth_min)
        obs = self._add_color_noise(obs)
        # print(np.max(obs))

 

        # np.save("/home/jon/Desktop/isaac_depth.npy", image)
        if dist > 10:
            dist = -1
        if dist_to_obj > 10:
            dist_to_obj = -1
        if len(obs) == 0:
            return {
                "image": np.zeros((64, 64, 3), dtype=np.float32),
                "depth": np.ones((64, 64, 1), dtype=np.float32),
                "target": np.zeros((64, 64, 3), dtype=np.float32),
                "interaction": np.zeros((64, 64, 3), dtype=np.float32),
                "dist_to_target": np.float32([dist]),
                "dist_to_object": np.float32([dist_to_obj]),
            }

 

        obs = obs[:, :, :-1]

 

        obs = np.float32(obs)
        is_first = self._step == 0

        return {
            "image": obs,
            "depth" : depth,
            "target": np.float32(self.target_image),
            "interaction": np.float32(self.interaction_image),
            "dist_to_target": np.float32([dist]),
            "dist_to_object": np.float32([dist_to_obj]),
            "is_terminal": False,
            "is_first": is_first,
        }

    def _get_info(self):
        # agent_pos = self._agent.get_translate()
        # goal_pos = self._goal_object.get_translate()
        # x_diff = abs(agent_pos[0] - goal_pos[0])
        # y_diff = abs(agent_pos[1] - goal_pos[1])
        dist = 0#x_diff + y_diff

        return {"discount": np.float32(0.997), "dist_to_target": dist}

    def simulate_steps(self):
        # print("num total steps ", self._length)

        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)

    def reset_obstacles(self,generate_points):# agent_loc, goal_loc,interaction_obj_loc):
        # self._locations_to_avoid = []
        # self._locations_to_avoid.append(goal_loc)
        # self._locations_to_avoid.append(agent_loc)
        # self._locations_to_avoid.append(interaction_obj_loc)
        if len(self._obstacles) < self._num_obstacles:
              for i in range(len(self._obstacles), self._num_obstacles):
                self._obstacles.append(
                        Object(
                            [0,0,0],
                            [0,0,0,0],
                            [0.007, 0.007, 0.007],
                            f"obstacle_{i}",
                            self._obstacle_parent_path,
                            self._stage,
                            usd_path=self._cone_path,
                            instanceable=False,
                        ))
        for i,obs in enumerate(self._obstacles):
            obs_loc = generate_points[3+i]
            if obs_loc is None:
                continue
            # self._locations_to_avoid.append(obs_loc)
            obs.change_start_and_reset(translate=obs_loc)
    

    def temp_force_look(self,point_to_look_at, random_ori=True):
        agent_point = self._agent.get_translate_vec()
        goal_point = point_to_look_at#self._goal_object.get_translate_vec()
        # Compute desired direction
        desired_direction = goal_point - agent_point
        desired_direction.Normalize()

        quat = self._agent.get_orientation_quat()
        quat.Normalize()
        quat = Gf.Quatd(quat)
        forward = Gf.Vec3d(1, 0, 0)

        rotation = Gf.Rotation(quat)
        new_linear_veloc = rotation.TransformDir(
            forward
        )  # Gf.Vec3d(linear_veloc[0],linear_veloc[1],linear_veloc[2]))

        out = [
            new_linear_veloc[0],
            new_linear_veloc[1],
            new_linear_veloc[2],
        ]
        forward_direction_np = np.array([out[0], out[1], 0])

        desired_direction_np = np.array([desired_direction[0], desired_direction[1], 0])
        if random_ori:
            angle = random.uniform(0.0, 360.0)
        else:
            angle = angle_to_align_vectors(
                forward_direction_np[:2], desired_direction_np[:2]
            )
        random_rotation_angle = math.radians(angle)
        rotation_quaternion = Gf.Quatd(random_rotation_angle, Gf.Vec3d(0, 0, 1))
        resulting_quaternion = rotation_quaternion * quat

        resulting_quaternion.Normalize()
        quat = Gf.Quatf(resulting_quaternion)
        original_quaternion = self._agent.get_orientation()

        angle_degrees = angle  # 45.0  # Replace with your desired angle
        axis_of_rotation = [0, 0, 1]  # Replace with your desired axis

        # Convert the angle from degrees to radians
        angle_radians = np.deg2rad(angle_degrees)

        # Calculate the sin and cos components of half the angle
        half_angle = angle_radians / 2.0

        cos_half_angle = np.cos(half_angle)

        sin_half_angle = np.sin(half_angle)

        # Normalize the axis of rotation
        axis_of_rotation /= np.linalg.norm(axis_of_rotation)

        # Create a quaternion representing the desired rotation
        rotation_quaternion = np.array(
            [
                cos_half_angle,
                axis_of_rotation[0] * sin_half_angle,
                axis_of_rotation[1] * sin_half_angle,
                axis_of_rotation[2] * sin_half_angle,
            ]
        )

        # Ensure both quaternions are normalized
        original_quaternion /= np.linalg.norm(original_quaternion)
        rotation_quaternion /= np.linalg.norm(rotation_quaternion)

        # Quaternion multiplication (Hamilton product)
        result_quaternion = np.array(
            [
                original_quaternion[0] * rotation_quaternion[0]
                - original_quaternion[1] * rotation_quaternion[1]
                - original_quaternion[2] * rotation_quaternion[2]
                - original_quaternion[3] * rotation_quaternion[3],
                original_quaternion[0] * rotation_quaternion[1]
                + original_quaternion[1] * rotation_quaternion[0]
                + original_quaternion[2] * rotation_quaternion[3]
                - original_quaternion[3] * rotation_quaternion[2],
                original_quaternion[0] * rotation_quaternion[2]
                - original_quaternion[1] * rotation_quaternion[3]
                + original_quaternion[2] * rotation_quaternion[0]
                + original_quaternion[3] * rotation_quaternion[1],
                original_quaternion[0] * rotation_quaternion[3]
                + original_quaternion[1] * rotation_quaternion[2]
                - original_quaternion[2] * rotation_quaternion[1]
                + original_quaternion[3] * rotation_quaternion[0],
            ]
        )
        return result_quaternion

    def test_step(self, action):
        self.pre_step(action)
        self.simulate_steps()
        return self.post_step(action)

    def pre_step(self, action):
        # print(action)
        self._goal_pos = self._goal_object.get_translate()
        unpack_action = self._action_to_direction[action]

        linear_veloc = unpack_action[:3]
        angular_veloc = unpack_action[3:]
        linear_veloc_gf = list_to_vec3d(linear_veloc)
        rotation = Gf.Rotation(self._agent.get_orientation_quat())
        new_linear_veloc = rotation.TransformDir(
            linear_veloc_gf
        )  # Gf.Vec3d(linear_veloc[0],linear_veloc[1],linear_veloc[2]))
        new_linear_veloc = [
            new_linear_veloc[0],
            new_linear_veloc[1],
            # new_linear_veloc[2],
            0.0,
        ]
        self.agent_alive = self._agent.step(new_linear_veloc, angular_veloc)
        # self.temp_force_look()
    
    def get_angle_and_dist_to_ob(self, pos):
        agent_pos = self._agent.get_translate()
 
        goal_pos =pos
        dist = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(goal_pos[:2]))
      
        agent_point = list_to_vec3d(agent_pos)  # self._agent.get_translate_vec()
        goal_point = list_to_vec3d(goal_pos)  # self._goal_object.get_translate_vec()

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
        desired_direction_np = np.array(
            [desired_direction[0], desired_direction[1], desired_direction[2]]
        )
        forward_direction_np = np.array(
            [forward_direction[0], forward_direction[1], forward_direction[2]]
        )
        angle = abs(angle_between_vectors(desired_direction_np, forward_direction_np))
        return angle, dist

    def post_step(self, action):
        self._step += 1

        agent_alive = self.agent_alive
        self._done = (
            (not agent_alive)
            or (self._length and self._step >= self._length)
            or (self._collision_reset)
        )
        # ensure agent doesnt leave, if it does kill and reset
        # check if agent is at goal
        terminated = self._done  # not agent_alive
        info = self._get_info()

        # ================= REWARD CALCULATIONS

        # set reward as dist to goal
        reward = 0

        #agent_pos = self._agent.get_translate()
        #agent_old_pos = self._agent._last_translate
        goal_pos = self._goal_object.get_translate()
        """dist = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(goal_pos[:2]))
        old_dist = np.linalg.norm(
            np.asarray(agent_old_pos[:2]) - np.asarray(goal_pos[:2])
        )
        dist_since_previous_step = old_dist - dist
        agent_point = list_to_vec3d(agent_pos)  # self._agent.get_translate_vec()
        goal_point = list_to_vec3d(goal_pos)  # self._goal_object.get_translate_vec()

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
        desired_direction_np = np.array(
            [desired_direction[0], desired_direction[1], desired_direction[2]]
        )
        forward_direction_np = np.array(
            [forward_direction[0], forward_direction[1], forward_direction[2]]
        )
        angle = abs(angle_between_vectors(desired_direction_np, forward_direction_np))"""
        angle, dist = self.get_angle_and_dist_to_ob(goal_pos)
        
        # old angle
        dist_agent_to_goal = -1
        if angle < 45:
            dist_agent_to_goal = dist

        interaction_obj_pos = self._interaction_obj.get_translate()
        angle_to_interaction, dist_agent_to_obj = self.get_angle_and_dist_to_ob(interaction_obj_pos)
        
        #dist_agent_to_obj = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(interaction_obj_pos[:2]))
        dist_obj_to_goal = np.linalg.norm(np.asarray(goal_pos[:2]) - np.asarray(interaction_obj_pos[:2]))

        # dist from agent to ineraction 
        if angle_to_interaction > 45:
            dist_agent_to_obj = -1

        obs = self._get_obs(dist_agent_to_goal,dist_agent_to_obj)
        if self._collision_reset:
            reward -= 1
        # self.threshold = 10

        if dist_obj_to_goal < self.threshold or self._goal_achieved:
            print("giving reward of 10")
            reward += 10
            terminated = True

        if self._interacted_with_object:
            if self._first_time:
                reward+=0.5
                self._first_time = False
            reward+=0.001
        obs["is_terminal"] = terminated

        return obs, reward, terminated, info

    def get_valid_random_spawn(self, offset=0):
        range = self._size_of_map  # 25  # 50#200
        valid_start = False
        agent_loc = [0, 0, 0]
        goal_loc = [0, 0, 0]
        while not valid_start:
            agent_loc = [
                np.random.uniform(-range, range),
                np.random.uniform(-range, range),
                4,
            ]
            goal_loc = [
                np.random.uniform(-range, range),
                np.random.uniform(-range, range),
                2,
            ]
            agent_loc[0] += offset
            goal_loc[0] += offset

            dist = np.linalg.norm(np.asarray(goal_loc[:2]) - np.asarray(agent_loc[:2]))
            if dist > (self.threshold + 5):
                valid_start = True
                break
        return agent_loc, goal_loc

    def get_random_obstacle_loc(self, spawn_dist=None, offset=0):
        range = self._size_of_map  # 35  # 50#200
        valid_start = False
        attempts = 0
        
        if spawn_dist is None:
            spawn_dist = self._minimum_distance_between_objects


        while not valid_start:
            if attempts > 1_000_000:
                print(" too many attempts giving up")
                return None
            # print("in the while loop")
            obstacle_loc = [
                np.random.uniform(-range, range),
                np.random.uniform(-range, range),
                0,
            ]
            obstacle_loc[0] += offset
            away_from_all = True
            invalid_counter = 0
            for loc in self._locations_to_avoid:
                # 
                dist = np.linalg.norm(
                    np.asarray(loc[:2]) - np.asarray(obstacle_loc[:2])
                )
                # print(dist, " ", self.threshold + 15)
                # print("dist and spawn_dist: ", dist, spawn_dist)
                if dist < (spawn_dist):
                    valid_start = False
                    invalid_counter += 1
            attempts +=1
        return obstacle_loc

    # def get_random_obstacle_loc(self, ref_pos1, ref_pos2, offset=0):
    #     range = self._size_of_map
    #     attempts = 0

    #     while True:
    #         # print("=====we're here=====")
    #         if attempts > 1_000_000:
    #             print(" too many attempts giving up")
    #             return None

    #         obstacle_loc = [
    #             np.random.uniform(-range, range) + offset,
    #             np.random.uniform(-range, range),
    #             0,
    #         ]
            
    #         for obstacle in range(len(self._obstacles)):       
    #             dist_ref1 = np.linalg.norm(np.asarray(ref_pos1[:2]) - np.asarray(obstacle_loc[:2]))
    #             dist_ref2 = np.linalg.norm(np.asarray(ref_pos2[:2]) - np.asarray(obstacle_loc[:2]))
    #             if (self._minimum_distance_between_objects <= dist_ref1 <= self._distance_between_objects_limit) and (self._minimum_distance_between_objects <= dist_ref2 <= self._distance_between_objects_limit):
    #                 self._obstacles.append(obstacle_loc)
    #                 return obstacle_loc

    #         attempts += 1
            

    #     return None

    
    # def get_random_loc_interaction_object(self, goal_pos,agent_pos, offset=0):
    #     range = self._size_of_map  # 35  # 50#200
    #     valid_start = False
    #     attempts = 0
    #     print(range, self.min_dist_between_inter_and_goal, self.max_dist_between_inter_and_goal)

    #     while not valid_start:
    #         if attempts > 1_000_000:
    #             print("stuck here")
    #             print(" too many attempts giving up")
    #             return None
    #         # print("in the while loop")
    #         interaction_loc = [
    #             np.random.uniform(-range, range),
    #             np.random.uniform(-range, range),
    #             0,
    #         ]
    #         interaction_loc[0] += offset
    #         away_from_all = True
    #         invalid_counter = 0
            
    #         dist = np.linalg.norm(
    #                 np.asarray(goal_pos[:2]) - np.asarray(interaction_loc[:2])
    #             )
    #         agent_dist = np.linalg.norm(
    #                 np.asarray(agent_pos[:2]) - np.asarray(interaction_loc[:2])
    #             )

    #         if dist >= self.min_dist_between_inter_and_goal and dist <= self.max_dist_between_inter_and_goal:
    #             if agent_dist >= self.min_dist_between_inter_and_agent and agent_dist <= self.max_dist_between_inter_and_agent:
    #                 return interaction_loc
    #         attempts +=1
    #     return None

    def get_random_loc_interaction_object(self, goal_pos, agent_pos, offset=0):
        range = self._size_of_map
        valid_start = False
        attempts = 0
        
        # Adaptive increase in distance thresholds after every 'threshold_increase_interval' attempts
        threshold_increase_interval = 100_000
        adaptive_increase = 5  # Adjust this value based on how much you want to increase each time
        
        while not valid_start:
            if attempts > 1_000_000:
                print("stuck here")
                print(" too many attempts giving up")
                return None

            interaction_loc = np.array([
                np.random.uniform(-range, range) + offset,
                np.random.uniform(-range, range),
                0,
            ])

            dist_to_goal = np.linalg.norm(goal_pos[:2] - interaction_loc[:2])
            dist_to_agent = np.linalg.norm(agent_pos[:2] - interaction_loc[:2])

            # Adaptive adjustments
            current_min_dist_inter_goal = self.min_dist_between_inter_and_goal - (attempts // threshold_increase_interval) * adaptive_increase
            current_max_dist_inter_goal = self.max_dist_between_inter_and_goal + (attempts // threshold_increase_interval) * adaptive_increase
            current_min_dist_inter_agent = self.min_dist_between_inter_and_agent - (attempts // threshold_increase_interval) * adaptive_increase
            current_max_dist_inter_agent = self.max_dist_between_inter_and_agent + (attempts // threshold_increase_interval) * adaptive_increase

            if current_min_dist_inter_goal <= dist_to_goal <= current_max_dist_inter_goal and current_min_dist_inter_agent <= dist_to_agent <= current_max_dist_inter_agent:
                return interaction_loc.tolist()

            attempts += 1

        return None


    def reset(self):

        mat_index = random.randint(0, len(self._materials) - 1)
        # print(mat_index)
        UsdShade.MaterialBindingAPI(self.ground_prim).Bind(
            self._materials[mat_index], UsdShade.Tokens.strongerThanDescendants
        )
        self._collision_reset = False

        self.simulate_steps()
        self._step = 0
        generate_points = self.generate_points(self._num_obstacles+3, self._minimum_distance_between_objects, self._size_of_map, offset=self.env_id * 2500)
        agent_loc = generate_points[0]
        goal_loc= generate_points[1]
        interaction_obj_loc = generate_points[2]
        #agent_loc, goal_loc = self.get_valid_random_spawn(offset=self.env_id * 2500)
        agent_loc[2] = 0.5
        goal_loc[2] = 0.0

        self._agent.change_start_and_reset(translate=agent_loc)
        self._goal_object.change_start_and_reset(translate=goal_loc)
        # self._locations_to_avoid = []
        # self._locations_to_avoid.append(goal_loc)
        # self._locations_to_avoid.append(agent_loc)
        # interaction_obj_loc = self.get_random_loc_interaction_object(
        #            goal_loc, agent_loc,offset=self.env_id * 2500
        #         )
        self._interaction_obj.change_start_and_reset(translate=interaction_obj_loc)


        self.simulate_steps()
        quat = self.temp_force_look(self._interaction_obj.get_translate_vec(),random_ori=self._random_starting_orientation)
        quat = Gf.Quatf(quat[0], quat[1], quat[2], quat[3])
        self._agent.change_start_and_reset(orientation=quat)
        self.simulate_steps()
        info = self._get_info()
        obs = self._get_obs(-1,-1)
        obs["is_terminal"] = self._step == 0
        self._angle_reward_steps = 0

        dist = np.linalg.norm(np.asarray(agent_loc[:2]) - np.asarray(goal_loc[:2]))
        

        self.reset_obstacles(generate_points)

        # for loc in self._locations_to_avoid:
        #     dist = np.linalg.norm(
        #         np.asarray(loc[:2]) - np.asarray(agent_loc[:2])
        #     )
        #     # print(f" Agent {self.id} spawning at a dist of {dist}")

        self._collision_reset = False
        self._interacted_with_object = False
        self._goal_achieved = False
        self._first_time = True

        return obs

    def render(self, *args, **kwargs):
        return None


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

def create_material_and_bind(mat_name, mat_path, prim_path, scale, stage):
    obj_prim = stage.GetPrimAtPath(prim_path)
    mtl_created_list = []
    print(obj_prim)
    print(mat_path)

    omni.kit.commands.execute(
        "CreateAndBindMdlMaterialFromLibrary",
        mdl_name=mat_path,
        mtl_name=mat_name,
        mtl_created_list=mtl_created_list,
    )
    print(mtl_created_list)
    mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
    print(mtl_prim)
    omni.usd.create_material_input(
        mtl_prim,
        "project_uvw",
        True,
        Sdf.ValueTypeNames.Bool,
    )

    omni.usd.create_material_input(
        mtl_prim,
        "texture_scale",
        Gf.Vec2f(scale, scale),
        Sdf.ValueTypeNames.Float2,
    )
    cube_mat_shade = UsdShade.Material(mtl_prim)

    UsdShade.MaterialBindingAPI(obj_prim).Bind(
        cube_mat_shade, UsdShade.Tokens.strongerThanDescendants
    )

    def step(self, action):
        # print("inside step the action is ", action)
        # if self._step // 100:
        #    print("Action : ", action, " : ", self._action_to_direction[action])

        self._step += 1

        self._goal_pos = self._goal_object.get_translate()
        unpack_action = self._action_to_direction[action]
        linear_veloc = unpack_action[:3]
        angular_veloc = unpack_action[3:]

        # STEP AGENT HERE
        agent_alive = self._agent.step(linear_veloc, angular_veloc)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._world.step(render=True)
        self._done = (not agent_alive) or (self._length and self._step >= self._length)

        # ensure agent doesnt leave, if it does kill and reset
        # check if agent is at goal
        terminated = self._done  # not agent_alive
        obs = self._get_obs(-1)
        info = self._get_info()

        # ================= REWARD CALCULATIONS

        # set reward as dist to goal
        reward = 0

        agent_pos = self._agent.get_translate()
        agent_old_pos = self._agent._last_translate
        goal_pos = self._goal_object.get_translate()
        # x_diff = abs(agent_pos[0]-goal_pos[0])
        # y_diff= abs(agent_pos[1]-goal_pos[1])
        dist = np.linalg.norm(np.asarray(agent_pos[:2]) - np.asarray(goal_pos[:2]))
        old_dist = np.linalg.norm(
            np.asarray(agent_old_pos[:2]) - np.asarray(goal_pos[:2])
        )
        dist_since_previous_step = old_dist - dist

        # update the running stats
        self.stats.update(dist_since_previous_step)

        # dist = x_diff + y_diff

        # Define your points
        agent_point = self._agent.get_translate_vec()
        goal_point = self._goal_object.get_translate_vec()

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
        desired_direction_np = np.array(
            [desired_direction[0], desired_direction[1], desired_direction[2]]
        )
        forward_direction_np = np.array(
            [forward_direction[0], forward_direction[1], forward_direction[2]]
        )
        angle = abs(angle_between_vectors(desired_direction_np, forward_direction_np))
        # print(angle)
        reward = 0

        #         #max can get is 1
        # if angle < 45:
        #     angle_reward += (1/(angle+1e-8))/10# .1 if looking direct

        # 0.0023
        # max can get is 1
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
        angle_reward = 0  # 1e-20
        if angle < 45:
            angle_reward = 1 / (angle + 1e-8)  # *10# .1 if looking direct
        # reward += angle_reward
        normalized_angle_reward = np.cos(np.deg2rad(np.asarray(angle_reward)))

        # ======= DISTANCE REWARD
        # scaled dist to target
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
            normalized_reward = (progress_reward - self.stats.mean) / (
                self.stats.standard_deviation + epsilon
            )
            return normalized_reward

        normalized_progress = normalize_progress(dist_since_previous_step)

        dist -= self.threshold - 1
        if dist <= 0:
            dist = 1
        normalized_distance = 1 / (dist + 1e-8)

        # reward += (50/(dist+1e-8)) + dist_since_previous_step
        w_distance = 0.2
        w_angle = 0.2
        w_progress = 0.50
        w_penalty = 0.1
        total_reward = (
            w_distance * normalized_distance
            + w_angle * normalized_angle_reward
            + w_progress * normalized_progress
        )
        # w_penalty * self.get_step_penalty())

        # ======= PENALTY REWARD

        total_reward -= w_penalty  # * self._ste)
    """
