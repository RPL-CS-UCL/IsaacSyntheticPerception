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
from omni.isaac.ui.ui_utils import btn_builder, dropdown_builder  # , str_builder
from .synthetic_perception import SyntheticPerception

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

    def _add_to_scene_event(self):
        self.sample.init_sensor_and_semantics()
        return
    def _camera_seg_event(self):
        asyncio.ensure_future(self.sample.final_fn())
        return

    def _save_lidar_info_event(self):
        asyncio.ensure_future(self.sample.save_lidar_data())
        # self.sample.save_lidar_data()
        return

    def _on_load_scene_button_event(self):
        self._add_to_scene_event()

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                self.add_button("Load Scene", self._on_load_scene_button_event)
                self.task_ui_elements["Load Scene"].enabled = True

                self.add_button("Save LiDAR", self._save_lidar_info_event)

                self.task_ui_elements["Save LiDAR"].enabled = True
                self.add_button("camera test", self._camera_seg_event)


                self.task_ui_elements["camera test"].enabled = True

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
