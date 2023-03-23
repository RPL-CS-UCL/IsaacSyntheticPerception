# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# from guppy import hpy
import os
from omni.isaac.examples.base_sample import BaseSampleExtension
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import (
    btn_builder,
    int_builder,
    dropdown_builder,
    combo_floatfield_slider_builder,
    str_builder,
)  # , str_builder
from .synthetic_perception import SyntheticPerception
from .sensors import SensorRig
import numpy as np
import omni

# This file is for UI control. It build on sample extension

class SyntheticPerceptionExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="ExtensionName",
            submenu_name="",
            name="Extension Name",
            title="ExtensionName Task",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html",
            overview="This Example shows how to follow a target using Franka robot in Isaac Sim.\n\nPress the 'Open in IDE' button to view the source code.",
            sample=SyntheticPerception(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=2,
            window_width=700,
        )

        self.mm = False
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        frame = self.get_frame(index=1)
        self.build_sensor_ui(frame)
        self._window.visible = True
        return

    def shutdown_cleanup(self):
        self.sample.remove_all_objects()
        return

    def add_button(self, label, on_clicked_fn):
        """Adds a button to the task frame"""
        dict = {
            "label": label,
            "type": "button",
            "text": label,
            "tooltip": label,
            "on_clicked_fn": on_clicked_fn,
        }

        self.task_ui_elements[label] = btn_builder(**dict)
        self.task_ui_elements[label].enabled = False

    def add_string_field(self, label, on_clicked_fn):
        "Adds a string to the task frame ()"
        dict = {
            "label": label,
            "use_folder_picker": True,
            "on_clicked_fn": on_clicked_fn,
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
            "label": label,
            "type": "button",
            "text": title,
            "tooltip": label,
            "on_clicked_fn": on_clicked_fn,
        }

        self.task_ui_elements[label] = btn_builder(**dict)
        self.task_ui_elements[label].enabled = True

    def add_slider(self, label, on_clicked_fn):
        dict = {
            "label": label,
        }
        _, self.task_ui_elements[label] = combo_floatfield_slider_builder(**dict)
        print(self.task_ui_elements[label].__dict__)
        self.task_ui_elements[label].enabled = False

    def _add_to_scene_event(self):
        self.sample.init_sensor_and_semantics()

    def _camera_seg_event(self):
        asyncio.ensure_future(self.sample.final_fn())

    def _test_event(self):
        self.sample.test(self.task_ui_elements["speed_slider"].get_value_as_float())

    def _on_sample_sensors(self):
        self.sample.sample_sensors()

    def _save_lidar_info_event(self):
        # asyncio.ensure_future(self.sample.save_lidar_data())
        # self.sample.sr.apply_veloc()
        # h = hpy()
        # h.heap()
        # print(h.heap())
        # self.sample.save_lidar_data()
        return

    def _on_load_scene_button_event(self):
        self._add_to_scene_event()

    def _testRigWaypoint(self):
        print("init waypoints")
        stage = omni.usd.get_context().get_stage()
        # self.sr.create_rig(np.array([0,0,0]),np.asarray([1,1,1,1]),stage)
        # self.sr.initialize_waypoints("",stage)
        self.sample.sr.initialize_waypoints("", stage)
        print("Attach move to callback")
        self.sample.temp_passthrough(self.sample.sr)
        # self.sr.move()

    def _loadtest(self):
        # Get the save directory

        asyncio.ensure_future(self.sample.load_sample())

    def _empty_func(self):
        print("Calling empty onclick")
        pass
    
    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True

                self.add_button_title("Attach Sys To Scene", "Attach", self._loadtest)
                self.add_button_title(
                    "Init waypoints & attach", "Attach", self._testRigWaypoint
                )

                self.add_button("veloc", self._save_lidar_info_event)
                self.task_ui_elements["veloc"].enabled = True

                self.add_button("sample sensors", self._on_sample_sensors)
                self.task_ui_elements["sample sensors"].enabled = True
                self.add_string_field("test", self._empty_func)


    def _rebuild_update(self, e):
        print("request rebuild", e)
        if str(e) == "Manual":
            self.mm=True

            # self.task_ui_elements["movement_speed"].enabled = True
        if str(e) == "Waypoints":
            self.mm = False

            # self.task_ui_elements["movement_speed"].enabled = False
        print(self.mm)
        # frame = self.get_frame(index=1)
        # self.build_sensor_ui(frame)
        return e
    def build_sensor_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Sensors"
                frame.visible = True
                self.task_ui_elements["movement_mode"] = dropdown_builder(items=["Waypoints","Manual","Linear"],on_clicked_fn=self._rebuild_update)
                self.task_ui_elements["movement_speed"] = int_builder("move speed")

                # self.task_ui_elements["movement_speed"].enabled = False
                # self.add_button_title("Attach Sys To Scene", "Attach", self._loadtest)
                # self.add_button_title(
                #     "Init waypoints & attach", "Attach", self._testRigWaypoint
                # )
                #
                # self.add_button("veloc", self._save_lidar_info_event)
                # self.task_ui_elements["veloc"].enabled = True
                #
                # self.add_button("sample sensors", self._on_sample_sensors)
                # self.task_ui_elements["sample sensors"].enabled = True
                # self.add_string_field("test", self._empty_func)
                # print(self.task_ui_elements["test"].get_value_as_float())
