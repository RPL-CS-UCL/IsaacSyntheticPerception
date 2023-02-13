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
from omni.isaac.ui.ui_utils import btn_builder,dropdown_builder #, str_builder
from .extensionname import ExtensionName

# This file is for UI control. It build on sample extension
# It calls FollowTarget, which is the sample, building on BaseSample
# - it builds the world defines task behaviours
# such as robot controller and waypoints, and passes on commands to the
# Task  

class ExtensionNameExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="ExtensionName",
            submenu_name="",
            name="Extension Name",
            title="ExtensionName Task",
            doc_link="https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html",
            overview="This Example shows how to follow a target using Franka robot in Isaac Sim.\n\nPress the 'Open in IDE' button to view the source code.",
            sample=ExtensionName(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=2,
            window_width=700,
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        frame = self.get_frame(index=1)
        self.build_data_logging_ui(frame)
        self._window.visible = True
        return

    def _on_follow_target_button_event(self):
        #asyncio.ensure_future(self.sample._on_follow_target_event_async())
        ##self.task_ui_elements["Follow Target"].enabled = False
        #self.task_ui_elements["Record Pose"].enabled = True
        #self.task_ui_elements["Record Grasp - Close"].enabled = True
        #self.task_ui_elements["Record Grasp - Open"].enabled = True
        return

    def reset_imitation_buttons(self):
        #self.task_ui_elements["Follow Target"].enabled = True
        #self.task_ui_elements["Mimic Trajectory"].enabled = True
        #self.task_ui_elements["Randomize Target"].enabled = True
        # self.task_ui_elements["Tool Task Scene"].enabled = True
        # self.task_ui_elements["Brush-Shoe Task Scene"].enabled = True
        #if len(self.sample.waypoints)>0: 
        #   self.task_ui_elements["Follow Trajectory"].enabled = True
        return


    def post_reset_button_event(self):
        #self.reset_imitation_buttons()
        pass

    def post_load_button_event(self):
        pass
        #self.reset_imitation_buttons()

    def post_clear_button_event(self):
        #self.reset_imitation_buttons()
        pass
	
    def shutdown_cleanup(self):
        self.sample.remove_all_objects()
        return

    def add_button(self, label, on_clicked_fn):
        """ Adds a button """
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
        self.sample.add_to_scene()
        return

    def _save_lidar_info_event(self):
        asyncio.ensure_future(self.sample.save_lidar_data())
        # self.sample.save_lidar_data()
        return

    def _on_load_scene_button_event(self):
        selection = self.task_ui_elements["Scene dropdown"].get_item_value_model().as_int
        selected_scene = self.selectable_scenes[selection]
        self._add_to_scene_event()
        # if selected_scene == "Tool Task Scene":
        #     self._on_generate_tool_task_button_event()
        # elif selected_scene == "Brush-Shoe Task Scene":
        #     self._on_generate_brush_shoe_task_button_event()
        # elif selected_scene == "Wine Scene":
        #     self.sample.generate_wine_scene()
        # elif selected_scene == "Screwdriver Scene":
        #     self.sample.generate_screwdriver_scene()
        # else:
        #     print (f"Unrecognized scene {selected_scene}!")
        return

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                self.selectable_scenes = ["Wine Scene", "Tool Task Scene", 
                                          "Brush-Shoe Task Scene", "Screwdriver Scene"]
                self.task_ui_elements["Scene dropdown"] = dropdown_builder(label="Choose Scene",
                                                                items=self.selectable_scenes)
                self.add_button("Load Scene", 
                           self._on_load_scene_button_event)
                self.task_ui_elements["Load Scene"].enabled = True

                self.add_button("Save LiDAR", self._save_lidar_info_event)

                self.task_ui_elements["Save LiDAR"].enabled = True

                # self.add_button("Mimic Trajectory", 
                #            self._on_imitate_trajectory_button_event)
                # self.add_button("Randomize Target", 
                #            self._on_randomize_target_button_event)
                # self.add_button("Tool Task Scene", 
                #            self._on_generate_tool_task_button_event)
                # self.add_button("Brush-Shoe Task Scene", 
                #            self._on_generate_brush_shoe_task_button_event)

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
        
    def build_data_logging_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Demonstration Controls"
                frame.visible = True

                # self.add_button("Follow Target", 
                #            self._on_follow_target_button_event)
                #
                # self.add_button("Follow Trajectory", 
                #            self._on_follow_trajectory_button_event)
                #
                # self.add_button("Record", 
                #            self._on_record_data_event)
                # self.add_button("Stop Record", 
                #            self._on_stop_record_data_event)
                #
                # self.add_button("Record Pose", 
                #            self._on_record_position_button_event)
                # self.add_button("Record Grasp - Close", 
                #            self._on_record_grasp_close_button_event)
                # self.add_button("Record Grasp - Open", 
                #            self._on_record_grasp_open_button_event)
                # self.task_ui_elements["Stop Record"].enabled = self.sample.is_data_recording
                # self.task_ui_elements["Record"].enabled = not self.sample.is_data_recording
        return
