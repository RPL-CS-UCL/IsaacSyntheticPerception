import sys
import os

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

import omni.appwindow  # Contains handle to keyboard
import numpy as np
import carb

from sensors import SensorRig
import PCG.WorldGenerator as WG


class Anymal_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
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
        self.step_total = 0

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error('Could not find Isaac Sim assets folder')

        # spawn warehouse scene
        prim = get_prim_at_path('/World/GroundPlane')
        if not prim.IsValid():
            prim = define_prim('/World/GroundPlane', 'Xform')
            asset_path = (
                assets_root_path
                + '/Isaac/Environments/Simple_Warehouse/warehouse.usd'
            )
            prim.GetReferences().AddReference(asset_path)

        self._anymal = self._world.scene.add(
            Anymal(
                prim_path='/World/Anymal',
                name='Anymal',
                usd_path=assets_root_path
                + '/Isaac/Robots/ANYbotics/anymal_c.usd',
                position=np.array([0, 0, 0.70]),
            )
        )
        self.sensor_rig = SensorRig('SensorRig', '/World/Anymal')
        self.path = '/home/jon/Downloads/sensors.json'
        self.out_path = '/home/jon/Downloads/out/'

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
        self.sensor_rig.create_rig_from_file(self.path, self.stage)
        self.sensor_rig.setup_sensor_output_path(self.out_path)

    def on_physics_step(self, step_size) -> None:
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        self.step_total += step_size
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
        self._anymal.advance(step_size, self._base_command)
        self.sensor_rig.sample_sensors(step_size)

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
            render = True

            self._world.step(render=render)
            if not self._world.is_simulating():
                self.needs_reset = True
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        [Summary]

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
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()

    # simulation_app.pause()
    objpath = "/home/jon/Downloads/new_objects_save.json"
    wrldpath = "/home/jon/Downloads/worlddata3.json"
    WG.create_world(objpath,wrldpath)
    # simulation_app.pause()
    runner.run()
    simulation_app.close()


if __name__ == '__main__':
    main()
