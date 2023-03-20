# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.examples.base_sample import BaseSampleExtension
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import (
    btn_builder,
    dropdown_builder,
    combo_floatfield_slider_builder,
)  # , str_builder
from .synthetic_perception import SyntheticPerception
from .sensors import SensorRig

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
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        frame = self.get_frame(index=1)
        self._window.visible = True
        return

    def post_reset_button_event(self):
        # self.reset_imitation_buttons()
        pass

    def post_load_button_event(self):
        pass
        # self.reset_imitation_buttons()

    def post_clear_button_event(self):
        # self.reset_imitation_buttons()
        pass

    def shutdown_cleanup(self):
        self.sample.remove_all_objects()
        return

    def add_button(self, label, on_clicked_fn):
        """Adds a button"""
        dict = {
            "label": label,
            "type": "button",
            "text": label,
            "tooltip": label,
            "on_clicked_fn": on_clicked_fn,
        }

        self.task_ui_elements[label] = btn_builder(**dict)
        self.task_ui_elements[label].enabled = False

        return

    def add_slider(self, label, on_clicked_fn):
        dict = {
            "label": label,
        }
        _, self.task_ui_elements[label] = combo_floatfield_slider_builder(**dict)
        print("here =-- --=-=-=-=-=----- ", _)
        print(self.task_ui_elements[label].__dict__)
        self.task_ui_elements[label].enabled = False

    def _add_to_scene_event(self):
        self.sample.init_sensor_and_semantics()
        return

    def _camera_seg_event(self):
        asyncio.ensure_future(self.sample.final_fn())
        return

    def _test_event(self):
        # asyncio.ensure_future(self.sample.test())
        # print("pressing but")
        # print(self.task_ui_elements["speed_slider"].get_value_as_float())
        self.sample.test(self.task_ui_elements["speed_slider"].get_value_as_float())
    
    def _on_sample_sensors(self):
        # asyncio.ensure_future(self.sample.test())
        # print("pressing but")
        # print(self.task_ui_elements["speed_slider"].get_value_as_float())
        # self.sample.test(self.task_ui_elements["speed_slider"].get_value_as_float())
        self.sample.sample_sensors()
    def _save_lidar_info_event(self):
        # asyncio.ensure_future(self.sample.save_lidar_data())
        self.sample.sr.apply_veloc()
        # self.sample.save_lidar_data()
        return

    def _on_load_scene_button_event(self):
        self._add_to_scene_event()

    def _on_value_changed(self):
        print(" ======================= done =====================")
    def _testFunc(self):
        sr = SensorRig(",","")
        stage = omni.usd.get_context().get_stage()
        sr.initialize_waypoints("",stage)

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True

                self.add_button("Waypointtest", self._testFunc)

                self.task_ui_elements["Waypointtest"].enabled = True

                self.add_button("Load Scene", self._on_load_scene_button_event)
                self.task_ui_elements["Load Scene"].enabled = True

                self.add_button("veloc", self._save_lidar_info_event)
                self.task_ui_elements["veloc"].enabled = True

                self.add_button("sample sensors", self._on_sample_sensors)
                self.task_ui_elements["sample sensors"].enabled = True

                # self.add_button("apply velocity", self._test_event)
                # self.task_ui_elements["apply velocity"].enabled = True
                args = {
                    "label": "Gripper Speed (UP)",
                    "default_val": 0,
                    "min": 0,
                    "max": 100,
                    "step": 1,
                    "tooltip": ["Speed in ()", "Speed in ()"],
                }

                self.task_ui_elements["speed_slider"], slider = combo_floatfield_slider_builder(
                    **args)
                # )
                #
                args = {
                    "label": "Set Speed",
                    "type": "button",
                    "text": "APPLY",
                    "tooltip": "Apply Cone Velocity in the Z-Axis",
                    "on_clicked_fn": self._test_event,
                }
                self.task_ui_elements["speed_button"] = btn_builder(**args)
                # self.add_slider("velocity", self._test_event)
                self.task_ui_elements["speed_button"].enabled = True
                # print(self.task_ui_elements["velocity"])

    def _on_record_data_event(self):
        self.task_ui_elements["Record"].enabled = False
        self.task_ui_elements["Stop Record"].enabled = True
        self.sample.toggle_data_recording(True)
        return

    def _on_stop_record_data_event(self):
        self.task_ui_elements["Record"].enabled = True
        self.task_ui_elements["Stop Record"].enabled = False
        self.sample.toggle_data_recording(False)
        return
