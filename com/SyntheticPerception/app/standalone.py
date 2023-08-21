import sys
import os
import multiprocessing
# sys.argv.insert(1, f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit')
#
# # Add paths to extensions
# sys.argv.append(f'--ext-folder')
# sys.argv.append(f'{os.path.abspath(os.environ["ISAAC_PATH"])}/exts')
# # Run headless
# sys.argv.append('--no-window')
#
# # Set some settings
# sys.argv.append('--/app/asyncRendering=False')
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({'headless': False})
from omni.isaac.core.utils.extensions import enable_extension

enable_extension('omni.kit.asset_converter')
simulation_app.update()
from omni.isaac.core import World
from omni.isaac.quadruped.robots import Anymal
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom
import time
import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb


import omni.kit.commands
from pxr import Gf, Sdf, Usd

from sensors import SensorRig
#import PCG.WorldGenerator as WG
from PCG.WorldGenerator import WorldManager

# sys.path.append('/home/stuart/Github/IsaacSyntheticPerception/com')
from dreamerv3_torch import envs
from dreamerv3_torch.dreamer import Dreamer


class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt, num_robots=1) -> None:
        """
        Summary

        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=render_dt,
        )
        self.num_robots = num_robots        
        self.step_total = 0
        self._world_size = 25.0

        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error('Could not find Isaac Sim assets folder')

        # spawn warehouse scene
        # prim = get_prim_at_path('/World/GroundPlane')
        # if not prim.IsValid():
        #     prim = define_prim('/World/GroundPlane', 'Xform')
        self.stage = omni.usd.get_context().get_stage()
        omni.kit.commands.execute('AddGroundPlaneCommand',
            stage=self.stage,
            planePath='/GroundPlane',
            axis='Z',
            size=self._world_size,
            position=Gf.Vec3f(0, 0, 0),
            color=Gf.Vec3f(0.5, 0.5, 0.5))
        mat_name = "Grass_Countryside"
        mat_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Grass_Countryside.mdl"
        prim_path = "/GroundPlane"
        self.world_manager = WorldManager()
        self.world_manager.create_material_and_bind(mat_name,mat_path,prim_path,1.0,self.stage)

            # asset_path = (
            #     assets_root_path
            #     + '/Isaac/Environments/Simple_Warehouse/warehouse.usd'
            # )
            # prim.GetReferences().AddReference(asset_path)

        objpath = "/home/stuart/Downloads/new_objects_save.json"
        wrldpath = "/home/stuart/Downloads/worlddata4.json"
        self.robots = []
        # self.mesh_gen = WG.create_world(objpath,wrldpath)

   
        np.random.seed(seed=None)
        #self.robots = self.spawn_robots(self.num_robots, self.start_locations(self.num_robots))
        # omni.kit.commands.execute('ChangeProperty',
        #     prop_path=Sdf.Path('/World/Anymal.xformOp:scale'),
        #     value=Gf.Vec3d(10.0, 10.0, 10.0),
        #     prev=Gf.Vec3d(1.0, 1.0, 1.0),
        # )
        print("============================================")
        #self.sensor_rig = SensorRig('SensorRig', '/World')
        self.path = '/home/stuart/Downloads/sensors.json'
        self.out_path = '/home/stuart/Downloads/out/'

        # Dreamer()


        # omni.kit.commands.execute('CreatePayloadCommand',
        #     # usd_context=<omni.usd._usd.UsdContext object at 0x7f9d38eb44f0>,
        #     path_to=Sdf.Path('/World/Black_Oak'),
        #     asset_path='/home/stuart/Documents/ov-vegetation3dpack-01-100.1.1/Trees/Black_Oak.usd',
        #     instanceable=False)

        
        self._rig_container = []
        self._num_rigs = 1
        if self._num_rigs > int(self._world_size/2):
            raise ValueError(f"too many rigs ({self._num_rigs}) for world size ({self._world_size})")
        self.start_locations(self._num_rigs, x_range=(-self._world_size,self._world_size))
        for rig in range(self._num_rigs):
            self._rig_container.append(SensorRig(f"SensorRig_{rig}","/World/Rigs"))
            # if self.locations[rig]
            self._rig_container[rig].create_rig_from_file(self.path, self.stage, self._world, self.locations[rig])
            self._rig_container[rig].setup_sensor_output_path(self.out_path)
            omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
                                        prim_type='Cube',
                                        prim_path=f'/World/Rigs/SensorRig_{rig}/Cube',
                                        select_new_prim=True,
                                        prepend_default_prim=False)

            omni.kit.commands.execute('SetStaticCollider',
                                        path=Sdf.Path(f'/World/Rigs/SensorRig_{rig}/Cube'),
                                        approximationShape='none')

            omni.kit.commands.execute('ChangeProperty',
                prop_path=Sdf.Path(f'/World/Rigs/SensorRig_{rig}.physxRigidBody:disableGravity'),
                value=False,
                prev=None,
                target_layer=Sdf.Find('anon:0x14c3e490:World0.usd')
                )

       

        self._world.reset()
        self._enter_toggled = 0
        self._base_command = np.zeros(3)

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            'NUMPAD_8': [1.0, 0.0, 0.0],
            'UP': [1.0, 0.0, 0.0],
            # back command
            'NUMPAD_2': [-1.0, 0.0, 0.0],
            'DOWN': [-1.0, 0.0, 0.0],
            # left command
            'NUMPAD_6': [0.0, -1.0, 0.0],
            'RIGHT': [0.0, -1.0, 0.0],
            # right command
            'NUMPAD_4': [0.0, 1.0, 0.0],
            'LEFT': [0.0, 1.0, 0.0],
            # yaw command (positive)
            'NUMPAD_7': [0.0, 0.0, 1.0],
            'N': [0.0, 0.0, 1.0],
            # yaw command (negative)
            'NUMPAD_9': [0.0, 0.0, -1.0],
            'M': [0.0, 0.0, -1.0],
        }
        self.needs_reset = False

    def start_locations(self, num_robots, x_range, z=0.7) -> None:
        y_range = x_range
        x_coords = np.random.uniform(x_range[0], x_range[1], num_robots)
        y_coords = np.random.uniform(y_range[0], y_range[1], num_robots)
        x_coords = np.around(x_coords, 1)
        y_coords = np.around(y_coords, 1)
        self.locations = np.column_stack((x_coords, y_coords, z*np.ones(num_robots)))

    def check_start_location(self, robot_location, terrain, max_slope_angle, relocation_distance, max_attempts):
        attempts = 0
        while attempts < max_attempts:
            location = robot_location
            gradient = self.compute_gradient(terrain, location)
            slope_angle = self.calculate_angle_to_plane(gradient)
            if slope_angle <= max_slope_angle:
                return location
            location = location + gradient * relocation_distance
            attempts += 1

        #raise Exception("Suitable spawn location not found after max attempts")
    
    # need coords from start location
    # samples coords around start location
    # use this to compute gradient (i.e., slope of terrain) between sample and start location
    # return gradient
    def compute_gradient(self, current_inde, location):
        delta = 0.1 # gradient step-size
        location_z = location[2]
        offset_x = [location[0] + delta, location[1]]   # offset terrain 
        offset_y = [location[0], location[1] + delta]
        gradient_x = (offset_x - location_z) / delta
        gradient_y = (offset_y - location_z) / delta
        gradient = {
            "x": gradient_x,
            "y": gradient_y
        }
        return gradient
    
    # need to calculate between gradient and horizontal plane
    # use this as slope angle
    # return angle 
    def calculate_angle_to_plane(self, gradient):
        # calc norm of gradients
        grad = gradient.items()
        norm = np.linalg.norm(grad)
        dot_product = 1 # dot_product between z_unt_vector (0,0,1) and gradient_vector is just 1 (since grad_z = 1)

        cos_theta = dot_product / norm # dot_product / magnitude
        angle = np.arccos(cos_theta)  # In radians

        return angle
    
    def get_height(self, x_coord, y_coord):
        # for 
        terrain = 1
        height = terrain[2]
        return height

    def spawn_robots(self, num_robots, start_locations):
        self._robot_list = []
        for robot in range(num_robots):
            print(f"[x,y,z] coord of Anymal_{robot}: {self.locations[robot]}")
            #self.locations[robot]
            x = int(round(self.locations[robot][0]))
            y = int(round(self.locations[robot][1]))
            point = self.mesh_gen._points[int(x+y*self.mesh_gen._size)][2]+1.0
            # print(f"mesh size: {self.mesh_gen._size}")
            # print(f"mesh height at position (x,y): {self.mesh_gen._points[(x+y*self.mesh_gen._size)]}")
            # for i in range(-5, 5, 1):
            #     print(f"points around start location: {self.mesh_gen._points[((x+i)+(y+i)*self.mesh_gen._size)]}")
            # print(f"length points vector: {len(self.mesh_gen._points)}")
            # print(f"FIRST 10 POINTS: {(self.mesh_gen._points[150:160])}")
            # print(f"MESH NORMALS: {self.mesh_gen.normals[0]}")
            normal_vec = self.mesh_gen.normals[int(x+y*self.mesh_gen._size)]
            # print(f"NORMAL VECTOR: {normal_vec}")
            up_vec = [0, 0, 1]
            diff_vec = np.asarray(abs(normal_vec - up_vec))
            diff_vec = np.rad2deg(diff_vec)
            # print(f"diff vec: {diff_vec}")

            # self._robot_list.append(self._world.scene.add(
            #     Anymal(
            #         prim_path=f'/World/Anymal_{robot}',
            #         name=f'Anymala_{robot}',
            #         usd_path=self.assets_root_path
            #         + '/Isaac/Robots/ANYbotics/anymal_c.usd',
            #         position=[self.locations[robot][0],self.locations[robot][1],point]#self.locations[robot],
            #     ))
            # )
            
        # print(dir(self._robot_list[0]))
        # print(self._robot_list[0])
        # print(self._robot_list[0].get_articulation_controller())
        self.time_since_spawn = time.time()
        return self._robot_list

    def setup(self) -> None:
        """
        [Summary]

        Set up keyboard listener and add physics callback

        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )
        self._world.add_physics_callback(
            'anymal_advance', callback_fn=self.on_physics_step
        )

        self.stage = (
            omni.usd.get_context().get_stage()
        )  # Used to access Geometry
        # self.sensor_rig.create_rig_from_file(self.path, self.stage, self._world)
        # self.sensor_rig.setup_sensor_output_path(self.out_path)

    def test(self, r, a, b):
        r.advance(a,b)

    def on_physics_step(self, step_size) -> None:
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        self.step_total += step_size
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
        #self._anymal.advance(step_size, self._base_command)
        for rig in self._rig_container:
            # THe following is a list of outputs per rig
            # e.g. 1 camera, 1 imu
            list_of_sensor_out = rig.sample_sensors_return()
            # dreamer.addobs(list_of_sensor_out[0])
            # linear_veloc,angular_veloc = dreamer.output()
            # rig.apply_veloc(linear_veloc,angular_veloc)
            #print(list_of_sensor_out)
            rig.move(self.step_total)
        for robot in self.robots:
            # print("joint position: ", robot._compute_observation(self._base_command)[12:24])
            time_elapsed = abs(time.time()-robot._time_since_spawned)
            # robot.advance(step_size, self._base_command)
            # print(robot.get_joint_positions())

            if time_elapsed > 5:
                robot.advance(step_size, self._base_command)

           
            else:
                robot.set_joint_positions(np.array([0,0,-0,-0,
                                                -3,3,-3,3,
                                            -1.7,1.7,-1.7,1.7,]))
                # JOINTS
                #  FL BR FR BL
                # robot.set_joint_positions(np.array([0.0, 0.6, -1.0, 
                # 0.0, -0.6, 1.0,
                #  -0.0, 0.6, 1.0,
                #  -0.0, -0.6, 1.0]))

        # do reset stuff in self.run(), performance takes a hit if done here
                #self.sensor_rig.sample_sensors(step_size)

    def run(self) -> None:
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            render = False
            if self.step_total > 10:
                self.step_total = 0
                # self.robot_reset()
            render = True

            self._world.step(render=render)
            if not self._world.is_simulating():
                self.needs_reset = True
        return

    def robot_reset(self) -> None:
        print("============================================\n RESET \n============================================")
        
        for robot in self.robots:
            robot.post_reset()

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        [Summary];

        Keyboard subscriber callback to when kit is updated.

        """
        # reset event
        self._event_flag = False

        self._base_command[0:3] += np.array(self._input_keyboard_mapping['UP'])
        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] += np.array(
                    self._input_keyboard_mapping[event.input.name]
                )

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] -= np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        return True


def main():
    """
    [Summary]

    Parse arguments and instantiate the ANYmal runner

    """
    physics_dt = 1 / 60.0
    render_dt = 1 / 60.0

    # envs.IsaacSim()

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt,num_robots=1)
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()

    # simulation_app.pause()
   
    # simulation_app.pause()
    runner.run()
    simulation_app.close()


if __name__ == '__main__':
    main()


# omni.kit.commands.execute('CreateMeshPrimWithDefaultXform',
# 	prim_type='Cube',
# 	prim_path=f'/World/Rigs/SensorRig_{i}/Cube',
# 	select_new_prim=True,
# 	prepend_default_prim=False)

# omni.kit.commands.execute('SetStaticCollider',
# 	path=Sdf.Path(f'/World/Rigs/SensorRig_{i}/Cube'),
# 	approximationShape='none')


