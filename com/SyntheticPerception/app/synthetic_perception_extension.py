"""
This class and functions handles the UI/UX side of the extension. 
All UX is collected and managed here and call their respective functions in the main 
SyntheticPerception sample.



"""
import time
import random
from omni.isaac.core.prims import XFormPrim, RigidPrim
from omni.physx import acquire_physx_interface
import os
from pxr import Usd, Gf, Ar, Pcp, Sdf, UsdRi, UsdGeom, UsdPhysics
from pxr import UsdShade, Sdf
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.kit.window.popup_dialog import FormDialog
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import (
    btn_builder,
    int_builder,
    float_builder,
    dropdown_builder,
    combo_floatfield_slider_builder,
    str_builder,
    xyz_builder,
)  # , str_builder
from omni.isaac.ui import (
    FloatField,
    CheckBox,
    StateButton,
    DropDown,
    StringField,
    Button,
    CheckBox,
)
from omni.isaac.core import SimulationContext
from .PCG.WorldGenerator import WorldManager 


from .synthetic_perception import SyntheticPerception
import omni
import json

from omni.isaac.core.utils.stage import (
    update_stage,
    add_reference_to_stage,
    is_stage_loading,
    update_stage_async,
)
from omni.isaac.core.utils.prims import define_prim, delete_prim

from .PCG.MeshGenerator import MeshGen
# from .Utils.EnvDataTool.EnvCreator import EnvTool
import open3d as o3d
import os
from perlin_numpy import generate_perlin_noise_2d, generate_fractal_noise_2d
from sklearn.preprocessing import normalize
from perlin_noise import PerlinNoise
import matplotlib.pyplot as plt
import cv2
import omni.kit.asset_converter
import carb
from omni.kit.window.popup_dialog.dialog import PopupDialog


class SelectedPrim:
    def __init__(self) -> None:
        self.prim = None
        self.prim_path = None
        self.object_scale = 1
        self.object_scale_delta = 0
        self.allow_y_rot = False
        self.unique_id = ''
        self.usd_path = ''
        self.class_name = ''
        self.posson_size = 1

    def get_y_rot_state(self):
        if self.allow_y_rot:
            return 'Enabled'
        return 'Disabled'

    def __str__(self) -> str:
        return f'prim: {self.prim} \n prim_path: {self.prim_path}\n Object Scale: {self.object_scale}\n \
                object scale delta: {self.object_scale_delta}\n allow y rot: {self.allow_y_rot}\n usdpath: {self.usd_path}\n unique_id: {self.unique_id}'


class SyntheticPerceptionExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name='ExtensionName',
            submenu_name='',
            name='Synthetic perception',
            title='Synthetic Perception',
            doc_link='https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html',
            overview="This extension provides tools to both generate procedural environments within Isaac as well as capturing and saving sensor data. This also includes waypoint movement and other such movement types.",
            sample=SyntheticPerception(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=12,
            window_width=700,
        )
        self.object_data_save_path = ''
        self.task_ui_elements = {}
        self.world_gen_ui_elements = {}
        self.usd_context = omni.usd.get_context()
        self.selected_prim = SelectedPrim()
        self.selected_prim_dict = {}
        self._object_selector = False
        self.prim = None

        self._object_path = ''
        self._world_path = ''
        self.mm = False
        self.OBJECT_EDITING_ALLOWED = False
        # frame = self.get_frame(index=0)
        # self.build_task_controls_ui(frame)

        # frame = self.get_frame(index=0)
        # self.build_sensor_ui(frame)

        # frame = self.get_frame(index=1)
        # self.build_worldgen_ui(frame)

        self._window.visible = True
        frame = self.get_frame(index=0)
        self.build_pcg_env_ui(frame)

        self._window.visible = True

        frame = self.get_frame(index=1)
        self.build_sensor_rig_ui(frame)

        frame = self.get_frame(index=2)
        self.setup_worldgen_ui(frame)
        self.events = self.usd_context.get_stage_event_stream()

        self.stage_event_delegate = self.events.create_subscription_to_pop(
            self._get_obj_details, name='Object Info Selection Update'
        )

    def on_stage_event(self, event):

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):

            prim_path = (
                self.usd_context.get_selection().get_selected_prim_paths()
            )

            if not prim_path:

                return

    def shutdown_cleanup(self):
        self.sample.remove_all_objects()

    def build_popup(self, errors):

        message_out = ''.join([str + '\n' for str in errors])
        dialog = FormDialog(
            title='ERROR',
            message=message_out,
            ok_handler=lambda dialog: print(
                f"Form accepted: '{dialog.get_values()}'"
            ),
        )

    def build_sensor_rig_ui(self, frame):
        self.build_sensor_rig_ui_values = {}
        self.build_sensor_rig_ui_values['RigPath'] = ''
        self.build_sensor_rig_ui_values['WaypointPath'] = ''
        self.build_sensor_rig_ui_values['MovementType'] = None

        self.build_sensor_rig_ui_values['OutputSavePath'] = ''
        self.sample.setup_scene()

        def has_missing_inputs_init_rig():
            errors = []

            if self.build_sensor_rig_ui_values['RigPath'] == '':
                errors.append('No Sensor Rig path supplied')

            if self.build_sensor_rig_ui_values['OutputSavePath'] == '':
                errors.append('No output path supplied')

            # Check if both files exist
            if not self._check_file_exists(
                self.build_sensor_rig_ui_values['RigPath']
            ):
                errors.append(
                    'Sensor rig parameter file invalid or does not exist.'
                )

            if len(errors) != 0:
                self.build_popup(errors)
                return True
            return False

        def has_missing_inputs_wp():
            errors = []

            if self.build_sensor_rig_ui_values['WaypointPath'] == '':
                errors.append('No waypoint path supplied')

            # Check if both files exist
            if not self._check_file_exists(
                self.build_sensor_rig_ui_values['WaypointPath']
            ):
                errors.append('Waypoint file is not valid or does not exist.')

            if len(errors) != 0:
                self.build_popup(errors)
                return True
            return False

        async def init_rig_and_waypoints():
            # await asyncio.ensure_future(self.sample.init_world())
            self.sample.init_sensor_rig_from_file(
                self.build_sensor_rig_ui_values['RigPath'],
                self.build_sensor_rig_ui_values['OutputSavePath'],
            )

        def load_sensor_rig_from_path():

            if has_missing_inputs_init_rig():
                return
            asyncio.ensure_future(init_rig_and_waypoints())

            stage = omni.usd.get_context().get_stage()
            parent = stage.GetPrimAtPath('/_WAYPOINTS_')

            if not parent:

                # parent = XFormPrim(
                #     name="_WAYPOINTS_",
                #     prim_path = "/"
                # )
                parent = define_prim('/_WAYPOINTS_', 'Xform')
            cube_prim = stage.GetPrimAtPath('/_WAYPOINTS_/w_01')
            if not cube_prim:
                cube_prim = stage.DefinePrim('/_WAYPOINTS_/w_01', 'Cube')
            UsdGeom.Xformable(cube_prim).AddTranslateOp().Set((0.0, 0.0, 0.0))

        def update_sensor_rig_path(val):

            self.build_sensor_rig_ui_values['RigPath'] = val

        def update_rig_movement_type(val):

            self.build_sensor_rig_ui_values['MovementType'] = val

        def update_waypoint_path(val):
            self.build_sensor_rig_ui_values['WaypointPath'] = val

        def load_waypoints_intermediate():
            asyncio.ensure_future(load_waypoints())

        async def load_waypoints():
            if has_missing_inputs_wp():
                return
            # self.sample.force_reload()
            await asyncio.ensure_future(self.sample._on_load_world_async())
            # await asyncio.ensure_future(self.sample.init_world())
            # print(self.sample._world.GetAttributes())
            # print(self.sample._world.__dir__())
            stage = omni.usd.get_context().get_stage()

            # Add a physics scene prim to stage

            scene = UsdPhysics.Scene.Define(
                stage, Sdf.Path('/World/physicsScene')
            )
            stage = omni.usd.get_context().get_stage()
            if not self.build_sensor_rig_ui_values['WaypointPath']:
                dialog = FormDialog(
                    title='ERROR No path',
                    message='No waypoint file was given. Not saving - please input a save path.',
                )
                return

            with open(
                self.build_sensor_rig_ui_values['WaypointPath'], 'r'
            ) as fh:
                json_data = json.load(fh)
                # print('Trying to load waypoints')
                # print(json_data)

                initial_prim_path = '/_WAYPOINTS_'
                prim_check= stage.GetPrimAtPath(initial_prim_path)
                parent = prim_check
                if not prim_check:
                    parent = define_prim('/_WAYPOINTS_', 'Xform')

                initial_prim_wp = '/_WAYPOINTS_/w_01'
                prim_check = stage.GetPrimAtPath(initial_prim_wp)
                if prim_check:
                    delete_prim(initial_prim_path)

                for i, c in enumerate(json_data):
                    # parent = define_prim('/_WAYPOINTS_', 'Xform')
                    cube_prim = stage.DefinePrim(
                        '/_WAYPOINTS_/w_{:02d}'.format(i + 1), 'Cube'
                    )
                    UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(
                        Gf.Vec3d(c)
                    )
                self.sample.sr.initialize_waypoints_preloaded(json_data,stage.GetPrimAtPath("/_WAYPOINTS_"))

            self.sample._world.add_physics_callback(
                'sim_step', callback_fn=self.sample.sr.move
            )

            self.sample.attach_sensor_sample_callback()

        def update_output_save_path(val):
            self.build_sensor_rig_ui_values['OutputSavePath'] = val

        def save_waypoints():
            def __n():
                print('')

            if not self.build_sensor_rig_ui_values['WaypointPath']:
                dialog = FormDialog(
                    title='ERROR No path',
                    message='No waypoint file was given. Not saving - please input a save path.',
                )
                return

            stage = omni.usd.get_context().get_stage()
            waypoints = []
            for prim_ref in stage.Traverse():
                prim_ref_name = str(prim_ref.GetPrimPath())
                if '_WAYPOINTS_' in prim_ref_name:
                    for i in range(len(prim_ref.GetChildren())):
                        prim_child = prim_ref.GetChildren()[i]
                        translate = prim_child.GetAttribute(
                            'xformOp:translate'
                        ).Get()
                        waypoints.append(
                            [translate[0], translate[1], translate[2]]
                        )
            with open(
                self.build_sensor_rig_ui_values['WaypointPath'], 'w'
            ) as fh:
                json.dump(waypoints, fh, indent=1)

        def run():
            def run_step_temp(n):
                curr_time = time.time() - self.minhan_timer_start
                # print(curr_time)
                if curr_time > 2:
                    self.minhan_timer_start = time.time()
                    scale = random.uniform(1.0,10.0)
                    self.plane_prim.GetAttribute('xformOp:scale').Set(Gf.Vec3d(scale,scale,scale))

            # self.sample.sr.hide_waypoints_an_rig()
            self.minhan_timer_start = 0
            stage = omni.usd.get_context().get_stage()
            self.plane_prim = stage.GetPrimAtPath("/World/Plane")
            # print(plane_prim)
            # print(plane_prim.GetAttributes())
            self.plane_prim.GetAttribute('physics:angularVelocity').Set(Gf.Vec3d(5.0,5.0,5.0))

            self.sample._world.add_physics_callback(
                'demo_step', callback_fn=run_step_temp
            )
        def sample():
            print("trying to sample")
            self.sample.sr.sample_all_sensors()
        self._sensor_rig_ui_inputs = {}
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Sensor Rig'
                frame.visible = True

                self._sensor_rig_ui_inputs['RigPath'] = StringField(
                    'Sensor Rig settings path',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=update_sensor_rig_path,
                )

                self._sensor_rig_ui_inputs['OutputSavePath'] = StringField(
                    'Output path',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=update_output_save_path,
                )
                self._sensor_rig_ui_inputs['LoadRig'] = Button(
                    'Load sensor rig',
                    'Load',
                    on_click_fn=load_sensor_rig_from_path,
                )

                self._sensor_rig_ui_inputs['MovementType'] = DropDown(
                    'Movement Type: ', on_selection_fn=update_rig_movement_type
                )
                self._sensor_rig_ui_inputs['MovementType'].set_items(
                    ['WAYPOINT', 'KEYBOARD']
                )

                self._sensor_rig_ui_inputs['WaypointPath'] = StringField(
                    'Waypoints path',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=update_waypoint_path,
                )

                self._sensor_rig_ui_inputs['LoadWaypoints'] = Button(
                    'Load & attach waypoints',
                    'Load',
                    on_click_fn=load_waypoints_intermediate,
                )
                self._sensor_rig_ui_inputs['SaveWaypoints'] = Button(
                    'Save waypoints',
                    'Save',
                    on_click_fn=save_waypoints,
                )

                # self._sensor_rig_ui_inputs['Run Simulation'] = Button(
                #     'Run Simulation',
                #     'Run',
                #     on_click_fn=run_sim,
                # )

                self._sensor_rig_ui_inputs['run'] = Button(
                    'Minghan run',
                    'run',
                    on_click_fn=run,
                )

                self._sensor_rig_ui_inputs['sample'] = Button(
                    'Sample sensors',
                    'sampe',
                    on_click_fn=sample,
                )

    def init_semantics_in_scene(self):
        self.sample.init_semantics_in_scene()

    def _rebuild_update(self, e):
        if str(e) == 'Manual':
            self.mm = True

        if str(e) == 'Waypoints':
            self.mm = False

        print(self.mm)
        return e

    def update_scale(self, val):
        if self.prim and val > 0:
            _ = self.prim.GetAttribute('xformOp:scale').Set(
                Gf.Vec3d([val, val, val])
            )
            self.selected_prim.object_scale = val
            self.selected_prim_dict[self.current_path].object_scale = val

    def update_scale_delta(self, val):
        if self.prim and val > 0:
            # update the local info
            # _ = self.prim.GetAttribute('xformOp:scale').Set(Gf.Vec3d([val,val,val]))
            self.selected_prim.object_scale_delta = val

            self.selected_prim_dict[self.current_path].object_scale_delta = val

    def update_poisson_size(self, val):
        if self.prim and val > 0:
            # update the local info
            # _ = self.prim.GetAttribute('xformOp:scale').Set(Gf.Vec3d([val,val,val]))

            self.selected_prim_dict[self.current_path].posson_size = val

    def update_yrot(self, val):
        if self.prim and val != 'Not Selected':
            enable_y_rot = False
            if (
                val == 'Enabled'
            ):  # self.world_gen_ui_elements["AllowYRot"].get_selected() == "Enabled":
                enable_y_rot = True
            self.selected_prim.allow_y_rot = enable_y_rot

            self.selected_prim_dict[
                self.current_path
            ].allow_y_rot = enable_y_rot

    def prim_name_update(self, val):
        if self.prim and val != '':
            self.selected_prim_dict[self.current_path].unique_id = val

    def class_name_update(self, val):
        if self.prim and val != '':
            self.selected_prim_dict[self.current_path].class_name = val

    def update_usd_path(self, val):
        if self.prim and val != '':
            self.selected_prim_dict[self.current_path].usd_path = val

    def save_path_update(self, val):
        self.object_data_save_path = val

    def _true(self, val):
        return True

    def save_object_data_to_file(self):
        def where_json(file_name):
            return os.path.exists(file_name)

        # Guard statement to catch no save path
        if self.object_data_save_path == '':
            dialog = FormDialog(
                title='ERROR No path',
                message='No save file was given. Not saving - please input a save path.',
                ok_handler=lambda dialog: print(
                    f"Form accepted: '{dialog.get_values()}'"
                ),
            )
            return

        if '.json' not in self.object_data_save_path:
            dialog = FormDialog(
                title='ERROR no specific file',
                message='No save file was given. Not saving - please input a save path with a filename and the .json extension.',
                ok_handler=lambda dialog: print(
                    f"Form accepted: '{dialog.get_values()}'"
                ),
            )
            return

        if self.selected_prim_dict[self.current_path].usd_path == '':
            dialog = FormDialog(
                title='ERROR no usd path',
                message='No USD path was specified. This is required and must exist!',
                ok_handler=lambda dialog: print(
                    f"Form accepted: '{dialog.get_values()}'"
                ),
            )
            return

        # Check if file exists at path
        # print('Attempting to edit or create the save obj file')
        data = {}
        with open(self.object_data_save_path, 'r+') as infile:
            try:
                data = json.load(infile)
            except:
                pass
            selected = self.selected_prim_dict[self.current_path]
        with open(self.object_data_save_path, 'w+') as outfile:
            specific_data = {
                'object_scale': selected.object_scale,
                'object_scale_delta': selected.object_scale_delta,
                'poisson_size': selected.posson_size,
                'allow_y_rot': selected.allow_y_rot,
                'class_name': selected.class_name,
                'usd_path': selected.usd_path,
            }
            data[selected.unique_id] = specific_data
            # data[local_selected.unique_id]=
            json.dump(data, outfile)

    def setup_worldgen_ui(self, frame):
        def test_check(val):
            self.OBJECT_EDITING_ALLOWED = val

        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Object set up'
                frame.visible = True
                self.world_gen_ui_elements['toggle'] = CheckBox(
                    'Object setup mode', on_click_fn=test_check
                )
                # print(self.world_gen_ui_elements["toggle"].__dir__())

                self.world_gen_ui_elements['SavePath'] = StringField(
                    'SavePath',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=self.save_path_update,
                )
                self.world_gen_ui_elements['PrimName'] = StringField(
                    'Unique Name',
                    'None',
                    read_only=False,
                    on_value_changed_fn=self.prim_name_update,
                )
                self.world_gen_ui_elements['PrimName'].set_value('None')

                self.world_gen_ui_elements['ClassName'] = StringField(
                    'Class Name',
                    'None',
                    read_only=False,
                    on_value_changed_fn=self.class_name_update,
                )
                self.world_gen_ui_elements['SelectedObjScale'] = FloatField(
                    'Object Scale',
                    default_value=1.0,
                    on_value_changed_fn=self.update_scale,
                )

                self.world_gen_ui_elements[
                    'SelectedObjScaleDelta'
                ] = FloatField(
                    'Object Scale Delta +/-',
                    on_value_changed_fn=self.update_scale_delta,
                )

                self.world_gen_ui_elements['PoissonSize'] = FloatField(
                    'Poisson Point Size',
                    default_value=1.0,
                    on_value_changed_fn=self.update_poisson_size,
                )
                # self.world_gen_ui_elements["AllowYRot"] = CheckBox("Allow Y-axis rotation", default_value = False, on_click_fn=self.update_yrot)
                self.world_gen_ui_elements['AllowYRot'] = DropDown(
                    'Allow Y-axis rotation', on_selection_fn=self.update_yrot
                )
                self.world_gen_ui_elements['AllowYRot'].set_items(
                    ['Not Selected', 'Enabled', 'Disabled']
                )
                self.world_gen_ui_elements['USDPath'] = StringField(
                    'USD Path',
                    use_folder_picker=True,
                    on_value_changed_fn=self.update_usd_path,
                )
                self.world_gen_ui_elements['SAVE'] = Button(
                    'Save this object to file',
                    'SAVE',
                    on_click_fn=self.save_object_data_to_file,
                )

        self.prim = None

        self.position = [0, 0, 0]

    def _get_obj_details(self, event):
        # if not self._object_selector:
        #     return
        if not self.OBJECT_EDITING_ALLOWED:
            return
        prim_path = self.usd_context.get_selection().get_selected_prim_paths()

        self.world_gen_ui_elements['SavePath'] = self.object_data_save_path
        if not prim_path:
            for key in self.world_gen_ui_elements:
                if type(self.world_gen_ui_elements[key]) == FloatField:
                    self.world_gen_ui_elements[key].set_value(0)

                if type(self.world_gen_ui_elements[key]) == DropDown:

                    self.world_gen_ui_elements[key].set_selection(
                        'Not Selected'
                    )
                if type(self.world_gen_ui_elements[key]) == StringField:

                    self.world_gen_ui_elements[key].set_value('')

            return

        stage = self.usd_context.get_stage()

        prim = stage.GetPrimAtPath(prim_path[0])

        self.prim = prim

        self.current_path = prim_path[0]

        # Check if the prim exists in our current dictionary
        if self.current_path not in self.selected_prim_dict:
            # Create the entry
            self.selected_prim_dict[self.current_path] = SelectedPrim()

        # This entry should now exist so we can use it.

        self.selected_prim.prim = prim
        self.selected_prim.prim_path = prim_path[0]

        # print('prim: ' + str(prim), " ", self.prim.GetAttributes())
        obj_scale = self.prim.GetAttribute('xformOp:scale').Get()
        # self.selected_prim.object_scale = sum(obj_scale) / len(obj_scale)
        self.selected_prim_dict[self.current_path].object_scale = sum(
            obj_scale
        ) / len(obj_scale)

        if (
            self.selected_prim_dict[self.current_path].unique_id == ''
            or self.selected_prim_dict[self.current_path].unique_id == 'None'
        ):
            self.selected_prim_dict[
                self.current_path
            ].unique_id = self.current_path.split('/')[-1]
        self.world_gen_ui_elements['PrimName'].set_value(
            self.selected_prim_dict[self.current_path].unique_id
        )
        self.world_gen_ui_elements['ClassName'].set_value(
            self.selected_prim_dict[self.current_path].class_name
        )
        self.world_gen_ui_elements['SelectedObjScale'].set_value(
            self.selected_prim_dict[self.current_path].object_scale
        )
        self.world_gen_ui_elements['SelectedObjScaleDelta'].set_value(
            self.selected_prim_dict[self.current_path].object_scale_delta
        )

        self.world_gen_ui_elements['PoissonSize'].set_value(
            self.selected_prim_dict[self.current_path].posson_size
        )
        self.world_gen_ui_elements['AllowYRot'].set_selection(
            self.selected_prim_dict[self.current_path].get_y_rot_state()
        )
        self.world_gen_ui_elements['USDPath'].set_value(
            self.selected_prim_dict[self.current_path].usd_path
        )

    def _update_object_path(self, val):
        if val != '':
            self._object_path = val

    def _update_world_path(self, val):
        if val != '':
            self._world_path = val

    def _check_file_exists(self, path):
        try:
            with open(path, 'r+') as infile:
                return True
        except:
            return False

    def _run_world_creation(self):
        # (
        #     obs_to_spawn,
        #     object_dict,
        #     height_map,
        # ) = self.sample.generate_world_generator(
        #     'C:\\Users\\jonem\\Desktop\\worlddata2.json',
        #     'C:\\Users\\jonem\\Desktop\\new_objects_save.json',
        # )
        #
        # asyncio.ensure_future(self.sample._on_load_world_async())
        # asyncio.ensure_future(
        #     self.sample.spawn_all(obs_to_spawn, object_dict, height_map)
        # )
        # return
        errors = []

        if self._object_path == '':
            errors.append('No Object path specified.')
        if '.json' not in self._object_path:
            errors.append('Object path does not contain .json extension.')
        if self._world_path == '':
            errors.append('No world path environment file was specified.')
        if '.json' not in self._world_path:
            errors.append('World path does not contain .json exntension.')

        # Check if both files exist
        if not self._check_file_exists(self._object_path):
            errors.append('Object path file specified does not exist.')

        if not self._check_file_exists(self._world_path):
            errors.append('World path file specified does not exist.')

        if len(errors) != 0:
            message_out = ''.join([str + '\n' for str in errors])
            dialog = FormDialog(
                title='ERROR',
                message=message_out,
                ok_handler=lambda dialog: print(
                    f"Form accepted: '{dialog.get_values()}'"
                ),
            )
            return
        print("Starting world gen")
        WG = WorldManager()
        WG.create_world(self._world_path, self._object_path)
        print("world creation finished")

        return

        # (
        #     obs_to_spawn,
        #     object_dict,
        #     height_map,
        #    normals 
        # ) = self.sample.generate_world_generator(
        #     self._world_path, self._object_path
        # )
        # print("Starting obj spawn")
        # asyncio.ensure_future(self.sample._on_load_world_async())
        # asyncio.ensure_future(
        #     self.sample.spawn_all(obs_to_spawn, object_dict, height_map, normals)
        # )

    def build_pcg_env_ui(self, frame):
        def open_world_creator():
            pass

        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Generate World Set up'
                frame.visible = True

                self.world_gen_ui_elements['RunCreateTool'] = Button(
                    'Open the world creator tool',
                    'Open',
                    on_click_fn=open_world_creator,
                )

                self.world_gen_ui_elements['ObjectsPath'] = StringField(
                    'Objects Path',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=self._update_object_path,
                )

                self.world_gen_ui_elements['WorldPath'] = StringField(
                    'World Path',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=self._update_world_path,
                )
                self.world_gen_ui_elements['SAVE'] = Button(
                    'Initialize world generation',
                    'Create World',
                    on_click_fn=self._run_world_creation,
                )

                self.world_gen_ui_elements['InitSemantics'] = Button(
                    'Initialize semantics and physics (must do)',
                    'Initialize',
                    on_click_fn=self.init_semantics_in_scene,
                )

    # async def material_test(self):
    #
    #     shape = (256, 256)
    #     threshold = 0.5
    #     region_value = 1
    #     # Convert to pymeshlab mesh
    #     l = shape[0] * 10   # 2560
    #     data = generate_perlin_noise_2d(shape, (8, 8))
    #     data = (data - np.min(data)) / (np.max(data) - np.min(data))
    #     data[data < threshold] = 0
    #     data[data >= threshold] = region_value
    #     mGen = MeshGen(
    #         256,
    #         10,
    #         data,
    #         'C:/Users/jonem/Documents/Kit/apps/Isaac-Sim/exts/IsaacSyntheticPerception/com/SyntheticPerception/app/PCG',
    #     )
    #     mGen.generate_terrain_mesh()
    #     return
    #     # asyncio.ensure_future(self.sample.init_world())
    #     print(' =========================== ')
    #     mat_path = 'http://omniverse-content-production.s3-us-west-2.amazonaws.com/Materials/Base/Natural/Dirt.mdl'
    #     prim_path = '/World/mesh_1'
    #     mat = '/World/Looks/Dirt'
    #
    #     stage = omni.usd.get_context().get_stage()
    #     obj_prim = stage.GetPrimAtPath(prim_path)
    #     mat_name = 'Dirt'
    #     # omni.kit.commands.execute(
    #     #     'CreateMdlMaterialPrimCommand',
    #     #     mtl_url=mat_path,
    #     #     mtl_name=f'{mat_name}',
    #     #     mtl_path=f'/World/Looks/{mat_name}',
    #     # )
    #
    #     # omni.kit.commands.execute(
    #     #     'CreateMdlMaterialPrimCommand',
    #     #     mtl_url=mat_path,
    #     #     mtl_name=f'{mat_name}',
    #     #     mtl_path=f'/World/Looks/{mat_name}',
    #     # )
    #     #
    #     # # update_stage()
    #     # _ = omni.kit.commands.execute(
    #     #     'BindMaterialCommand',
    #     #     prim_path=prim_path,
    #     #     material_path=f'/World/Looks/{mat_name}',
    #     # )
    #     mtl_created_list = []
    #
    #     omni.kit.commands.execute(
    #         'CreateAndBindMdlMaterialFromLibrary',
    #         mdl_name=mat_path,
    #         mtl_name=mat_name,
    #         mtl_created_list=mtl_created_list,
    #     )
    #
    #     mtl_prim = stage.GetPrimAtPath(mtl_created_list[0])
    #
    #     omni.usd.create_material_input(
    #         mtl_prim,
    #         'project_uvw',
    #         True,
    #         Sdf.ValueTypeNames.Bool,
    #     )
    #
    #     omni.usd.create_material_input(
    #         mtl_prim,
    #         'texture_scale',
    #         Gf.Vec2f(0.001, 0.001),
    #         Sdf.ValueTypeNames.Float2,
    #     )
    #     cube_mat_shade = UsdShade.Material(mtl_prim)
    #
    #     UsdShade.MaterialBindingAPI(obj_prim).Bind(
    #         cube_mat_shade, UsdShade.Tokens.strongerThanDescendants
    #     )
    #     return
    #
    #     # Set material inputs, these can be determined by looking at the .mdl file
    #
    #     # or by selecting the Shader attached to the Material in the stage window and looking at the details panel
    #
    #     print('wait')
    #     await update_stage_async()
    #     print('continue')
    #     update_stage()
    #     while is_stage_loading():
    #         await update_stage_async()
    #
    #     stage = omni.usd.get_context().get_stage()
    #     p = stage.GetPrimAtPath(f'{mat}/Shader')
    #     not_set = False
    #
    #     omni.kit.commands.execute(
    #         'SelectPrims',
    #         old_selected_paths=['/World'],
    #         new_selected_paths=['/World/Looks/Dirt'],
    #         expand_in_stage=True,
    #     )
    #
    #     omni.kit.commands.execute(
    #         'SelectPrims',
    #         old_selected_paths=['/World'],
    #         new_selected_paths=['/World/Looks/Dirt'],
    #         expand_in_stage=True,
    #     )
    #
    #     print('wait')
    #     await update_stage_async()
    #     print('continue')
    #     update_stage()
    #     while is_stage_loading():
    #         await update_stage_async()
    #     # while not not_set:
    #     #     try:
    #     #         material_attributes = p.GetAttributes()
    #     #         p.GetAttribute('inputs:project_uvw').Set(True)
    #     #         not_set = True
    #     #         print("success: ", _)
    #     #     except:
    #     #
    #     #         print("failure: ", _)
    #     #         await update_stage_async()
    #     #
    #
    #     material_attributes = p.GetAttributes()
    #     p.GetAttribute('inputs:project_uvw').Set(True)
    #     p.GetAttribute('inputs:texture_scale').Set((0.001, 0.001))
    #
    #     omni.kit.commands.execute(
    #         'SelectPrims',
    #         old_selected_paths=['/World'],
    #         new_selected_paths=['/World/Looks/Dirt'],
    #         expand_in_stage=True,
    #     )
    #
    #     omni.kit.commands.execute(
    #         'SelectPrims',
    #         old_selected_paths=['/World'],
    #         new_selected_paths=['/World/Looks/Dirt'],
    #         expand_in_stage=True,
    #     )

    # def build_task_controls_ui(self, frame):
    #     with frame:
    #         with ui.VStack(spacing=5):
    #             # Update the Frame Title
    #             frame.title = 'Sensor Controls'
    #             frame.visible = True
    #
    #             self.add_button_title(
    #                 'Attach Sys To Scene', 'Attach', self._loadtest
    #             )
    #             self.add_button_title(
    #                 'Init waypoints & attach', 'Attach', self._testRigWaypoint
    #             )
    #
    #             # self.add_button('veloc', self._save_lidar_info_event)
    #             # self.task_ui_elements['veloc'].enabled = True
    #
    #             self.add_button('sample sensors', self._on_sample_sensors)
    #             self.task_ui_elements['sample sensors'].enabled = True
    #             # self.add_string_field("test", self._empty_func)
    #
    #             self.add_button('init_world', self.ui_init_world)
    #             self.task_ui_elements['init_world'].enabled = True
    #
    #             self.add_button(
    #                 'load_sensors', self.test_load_sensors_from_file
    #             )
    #             self.task_ui_elements['load_sensors'].enabled = True
    #             # OTHER UI NEEDED
    #             # load sensor rig
    #             # ^ let the above handle waypoints and everything
    #
    #             # self.add_button('init_semantics', self.ui_init_semantics)
    #             # self.task_ui_elements['init_semantics'].enabled = True
    #             # self.add_button('area gen test', self._empty_func)
    #             # self.task_ui_elements['area gen test'].enabled = True

    # def build_sensor_ui(self, frame):
    #     with frame:
    #         with ui.VStack(spacing=5):
    #             # Update the Frame Title
    #             frame.title = 'Sensors'
    #             frame.visible = True
    #             self.task_ui_elements['movement_mode'] = dropdown_builder(
    #                 items=['Waypoints', 'Manual', 'Linear'],
    #                 on_clicked_fn=self._rebuild_update,
    #             )
    #             self.task_ui_elements['movement_speed'] = int_builder(
    #                 'move speed'
    #             )

    # def add_button(self, label, on_clicked_fn):
    #     """Adds a button to the task frame"""
    #     dict = {
    #         'label': label,
    #         'type': 'button',
    #         'text': label,
    #         'tooltip': label,
    #         'on_clicked_fn': on_clicked_fn,
    #     }
    #
    #     self.task_ui_elements[label] = btn_builder(**dict)
    #     self.task_ui_elements[label].enabled = False

    # async def ini(self):
    #     await asyncio.ensure_future(self.sample.init_world())
    #     self.sample.init_sensor_rig_from_file()
    #
    #     stage = omni.usd.get_context().get_stage()
    #     self.sample.sr.initialize_waypoints('', stage)
    #     print('Attach move to callback')
    #     self.sample.attach_sensor_waypoint_callback(self.sample.sr)

    # def _add_to_scene_event(self):
    #     self.sample.init_sensor_and_semantics()
    #
    # def _on_load_scene_button_event(self):
    #     self._add_to_scene_event()

    # def build_worldgen_ui(self, frame):
    #     with frame:
    #         with ui.VStack(spacing=5):
    #             # Update the Frame Title
    #             frame.title = 'World Gen'
    #             frame.visible = True
    #             self.add_button('init_world', self.ui_init_world)
    #             self.task_ui_elements['init_world'].enabled = True
    #
    #             self.add_button('init_semantics', self.ui_init_semantics)
    #             self.task_ui_elements['init_semantics'].enabled = True
    #             # self.add_button('area gen test', self._empty_func)
    #             # self.task_ui_elements['area gen test'].enabled = True
