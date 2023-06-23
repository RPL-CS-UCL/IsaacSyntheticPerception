# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# from guppy import hpy
import os

from pxr import Usd, Gf, Ar, Pcp, Sdf, UsdRi
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
)
from .synthetic_perception import SyntheticPerception
from .sensors import SensorRig
import numpy as np
import omni
from .PCG import AreaMaskGenerator
from omni.isaac.core.objects import DynamicCuboid

# This file is for UI control. It build on sample extension
from omni.isaac.core import World
import json

import os


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
            name='Extension Name',
            title='ExtensionName Task',
            doc_link='https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html',
            overview="This Example shows how to follow a target using Franka robot in Isaac Sim.\n\nPress the 'Open in IDE' button to view the source code.",
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

        self.mm = False
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        frame = self.get_frame(index=1)
        self.build_sensor_ui(frame)

        frame = self.get_frame(index=1)
        self.build_worldgen_ui(frame)

        frame = self.get_frame(index=2)
        self.setup_worldgen_ui(frame)
        self._window.visible = True

        self.events = self.usd_context.get_stage_event_stream()

        self.stage_event_delegate = self.events.create_subscription_to_pop(
            self._get_obj_details, name='Object Info Selection Update'
        )

    def on_stage_event(self, event):

        # NEW: if statement to only check when selection changed

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            print('selection changed')

            prim_path = (
                self.usd_context.get_selection().get_selected_prim_paths()
            )

            if not prim_path:

                return

    def shutdown_cleanup(self):
        self.sample.remove_all_objects()

    def add_button(self, label, on_clicked_fn):
        """Adds a button to the task frame"""
        dict = {
            'label': label,
            'type': 'button',
            'text': label,
            'tooltip': label,
            'on_clicked_fn': on_clicked_fn,
        }

        self.task_ui_elements[label] = btn_builder(**dict)
        self.task_ui_elements[label].enabled = False

    def talk(self):
        print('talk talk talk talk')

    def add_float(self, label, dict_to_use):
        """Adds a button to the task frame"""
        dict = {
            'label': label,
            'type': 'button',
            'tooltip': label,
        }

        dict_to_use[label] = float_builder(**dict)
        # dict_to_use[label].enabled = True
        # dict_to_use[label].on_value_changed_fn(self.talk)

    def add_xyz(self, label, dict_to_use):
        """Adds a button to the task frame"""
        dict = {
            'label': label,
        }

        dict_to_use[label] = xyz_builder(**dict)
        # self.task_ui_elements[label].enabled = False

    def add_string_field(self, label, on_clicked_fn):
        """Adds a string to the task frame ()"""
        dict = {
            'label': label,
            'use_folder_picker': True,
            'on_clicked_fn': on_clicked_fn,
        }
        self.task_ui_elements[label] = str_builder(**dict)

    # def add_int_field(self, label, on_clicked_fn):
    #     "Adds a string to the task frame ()"
    #     dict = {
    #         "label": label,
    #         "use_folder_picker": True,
    #         "on_clicked_fn": on_clicked_fn,
    #     }
    #     self.task_ui_elements[label] = str_builder(**dict)
    def add_button_title(self, label, title, on_clicked_fn):
        """Adds a button"""
        dict = {
            'label': label,
            'type': 'button',
            'text': title,
            'tooltip': label,
            'on_clicked_fn': on_clicked_fn,
        }

        self.task_ui_elements[label] = btn_builder(**dict)
        self.task_ui_elements[label].enabled = True

    def add_slider(self, label, on_clicked_fn):
        dict = {
            'label': label,
        }
        _, self.task_ui_elements[label] = combo_floatfield_slider_builder(
            **dict
        )
        print(self.task_ui_elements[label].__dict__)
        self.task_ui_elements[label].enabled = False

    def _add_to_scene_event(self):
        self.sample.init_sensor_and_semantics()

    def _camera_seg_event(self):
        asyncio.ensure_future(self.sample.final_fn())

    def _test_event(self):
        self.sample.test(
            self.task_ui_elements['speed_slider'].get_value_as_float()
        )

    def _on_sample_sensors(self):
        self.sample.sample_sensors()

    def _save_lidar_info_event(self):
        # asyncio.ensure_future(self.sample.save_lidar_data())
        # self.sample_sensor_rig.apply_veloc()
        # h = hpy()
        # h.heap()
        # print(h.heap())
        # self.sample.save_lidar_data()
        return

    def _on_load_scene_button_event(self):
        self._add_to_scene_event()

    def _testRigWaypoint(self):
        print('init waypoints')
        stage = omni.usd.get_context().get_stage()
        self.sample_sensor_rig.initialize_waypoints('', stage)
        print('Attach move to callback')
        self.sample.temp_passthrough(self.sample_sensor_rig)
        # self_sensor_rig.move()

    def _loadtest(self):
        # Get the save directory

        asyncio.ensure_future(self.sample.load_sample())

    def _empty_func(self):
        print('Area gen test, passing to sample')
        self.sample.test_areagen()

    def ui_init_world(self):
        asyncio.ensure_future(self.sample.init_world())

    def ui_init_semantics(self):
        self.sample.init_semantics_in_scene()

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Task Controls'
                frame.visible = True

                self.add_button_title(
                    'Attach Sys To Scene', 'Attach', self._loadtest
                )
                self.add_button_title(
                    'Init waypoints & attach', 'Attach', self._testRigWaypoint
                )

                self.add_button('veloc', self._save_lidar_info_event)
                self.task_ui_elements['veloc'].enabled = True

                self.add_button('sample sensors', self._on_sample_sensors)
                self.task_ui_elements['sample sensors'].enabled = True
                # self.add_string_field("test", self._empty_func)

                self.add_button('init_world', self.ui_init_world)
                self.task_ui_elements['init_world'].enabled = True

                self.add_button('init_semantics', self.ui_init_semantics)
                self.task_ui_elements['init_semantics'].enabled = True
                self.add_button('area gen test', self._empty_func)
                self.task_ui_elements['area gen test'].enabled = True

    def _rebuild_update(self, e):
        print('request rebuild', e)
        if str(e) == 'Manual':
            self.mm = True

        if str(e) == 'Waypoints':
            self.mm = False

        print(self.mm)
        return e

    def build_sensor_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Sensors'
                frame.visible = True
                self.task_ui_elements['movement_mode'] = dropdown_builder(
                    items=['Waypoints', 'Manual', 'Linear'],
                    on_clicked_fn=self._rebuild_update,
                )
                self.task_ui_elements['movement_speed'] = int_builder(
                    'move speed'
                )

    def build_worldgen_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'World Gen'
                frame.visible = True
                self.add_button('init_world', self.ui_init_world)
                self.task_ui_elements['init_world'].enabled = True

                self.add_button('init_semantics', self.ui_init_semantics)
                self.task_ui_elements['init_semantics'].enabled = True
                self.add_button('area gen test', self._empty_func)
                self.task_ui_elements['area gen test'].enabled = True

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

    def update_yrot(self, val):
        if self.prim and val != 'Not Selected':
            print('Updating y rot')
            # update the local info
            enable_y_rot = False
            if (
                val == 'Enabled'
            ):  # self.world_gen_ui_elements["AllowYRot"].get_selected() == "Enabled":
                enable_y_rot = True
            print('setting y rot to ', enable_y_rot, '   ', val)
            self.selected_prim.allow_y_rot = enable_y_rot

            self.selected_prim_dict[
                self.current_path
            ].allow_y_rot = enable_y_rot

    def prim_name_update(self, val):
        if self.prim and val != '':
            self.selected_prim_dict[self.current_path].unique_id = val
            print('!! Updating prim name to ', val)
            # print(self.selected_prim.unique_id)
            # print(self.selected_prim)

    def class_name_update(self, val):
        if self.prim and val != '':
            self.selected_prim_dict[self.current_path].class_name = val
            print('!! Updating clss name to ', val)
            # print(self.selected_prim.unique_id)
            # print(self.selected_prim)

    def update_usd_path(self, val):
        if self.prim and val != '':
            print('setting usd path to ', val)
            self.selected_prim_dict[self.current_path].usd_path = val
            print(self.selected_prim_dict[self.current_path].usd_path)

    def save_path_update(self, val):
        self.object_data_save_path = val

    def _true(self, val):
        return True

    # def save_object_data_to_file(self):
    #     if self.object_data_save_path == '':
    #         return

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
        print('Attempting to edit or create the save obj file')
        data = {}
        with open(self.object_data_save_path, 'r+') as infile:
            try:
                data = json.load(infile)
                print('old file loaded')
            except:
                print('couldnt load the file')
                pass
            selected = self.selected_prim_dict[self.current_path]
        with open(self.object_data_save_path, 'w+') as outfile:
            specific_data = {
                'object_scale': selected.object_scale,
                'object_scale_delta': selected.object_scale_delta,
                'allow_y_rot': selected.allow_y_rot,
                'class_name': selected.class_name,
                'usd_path': selected.usd_path,
            }
            data[selected.unique_id] = specific_data
            # data[local_selected.unique_id]=
            json.dump(data, outfile)

    def setup_worldgen_ui(self, frame):

        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = 'Object set up'
                frame.visible = True

                self.world_gen_ui_elements['SavePath'] = StringField(
                    'SavePath',
                    'None',
                    read_only=False,
                    use_folder_picker=True,
                    item_filter_fn=self._true,
                    on_value_changed_fn=self.save_path_update,
                )
                # self.add_button('get_selected', self._get_obj_details)
                # self.task_ui_elements['get_selected'].enabled = True
                # self.add_xyz('xyz',self.world_gen_ui_elements)
                # self.add_float('Scale', self.world_gen_ui_elements)
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

                # self.task_ui_elements['xyz'].enabled = True

                # self.add_button('area gen test2', self._empty_func)
                # self.task_ui_elements['area gen test2'].enabled = True
        self.prim = None

        self.position = [0, 0, 0]

        # Save the UsdContext name (we currently only work with a single Context)

        # Track selection changes

    def _get_obj_details(self, event):
        print('==== Selection occured ====')
        prim_path = self.usd_context.get_selection().get_selected_prim_paths()

        self.world_gen_ui_elements['SavePath'] = self.object_data_save_path
        if not prim_path:
            print('unselecting')
            for key in self.world_gen_ui_elements:
                print(type(self.world_gen_ui_elements[key]))
                if type(self.world_gen_ui_elements[key]) == FloatField:
                    self.world_gen_ui_elements[key].set_value(0)

                if type(self.world_gen_ui_elements[key]) == DropDown:
                    print('resetting dropdown')

                    self.world_gen_ui_elements[key].set_selection(
                        'Not Selected'
                    )
                if type(self.world_gen_ui_elements[key]) == StringField:

                    self.world_gen_ui_elements[key].set_value('')

                    # self.world_gen_ui_elements[key].set_value(False)
                # if self.world_gen_ui_elements[key] is
            # print("")
            # self.world_gen_ui_elements['SelectedObjScale'].set_value(0)
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
            print('setting unique id')
            self.selected_prim_dict[
                self.current_path
            ].unique_id = self.current_path
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

        self.world_gen_ui_elements['AllowYRot'].set_selection(
            self.selected_prim_dict[self.current_path].get_y_rot_state()
        )
        print(
            '===== usd path ',
            self.selected_prim_dict[self.current_path].usd_path,
        )
        self.world_gen_ui_elements['USDPath'].set_value(
            self.selected_prim_dict[self.current_path].usd_path
        )
