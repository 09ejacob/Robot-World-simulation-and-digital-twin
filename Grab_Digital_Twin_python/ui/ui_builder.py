# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.timeline
import omni.ui as ui
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import create_new_stage, get_current_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from pxr import Sdf, UsdLux
from ..scenes.setup_scene import setup_scene
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
)
from ..scenarios.pick_boxes_scenario import PickBoxesScenario
from ..scenarios.stack_box_scenario import StackBoxScenario
from ..scenarios.udp_scenario import UDPScenario
from ..robot.robot_controller import RobotController
from ..camera_capture import CameraCapture


class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._robot_controller = RobotController()
        self._camera_capture = CameraCapture()
        self._scenarios = {
            "UDP": UDPScenario,
            "Pick Boxes": PickBoxesScenario,
            "Stack Box": StackBoxScenario,
        }
        self._current_scenario_name = "UDP"  # Default scenario

        self._scenario = None

        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)"""
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()

            # Only disable if the scenario is actually finished
            if self._scenario and self._scenario._did_run:
                self._scenario_state_btn.enabled = False

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._load_btn = LoadButton(
                    "Load Button",
                    "LOAD",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._setup_scenario,
                )
                self._load_btn.set_world_settings(
                    physics_dt=1 / 60.0, rendering_dt=1 / 20.0
                )
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button",
                    "RESET",
                    pre_reset_fn=None,
                    post_reset_fn=self._on_post_reset_btn,
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)

        scenario_frame = CollapsableFrame("Scenario", collapsed=False)
        with scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label("Select Scenario:")

                self._scenario_dropdown = ui.ComboBox(
                    0,
                    *list(self._scenarios.keys()),
                )

                ui.Button(
                    "Select Scenario",
                    clicked_fn=self._select_scenario,
                )

                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self.start_scenario,
                    on_b_click_fn=self.stop_scenario,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

        robot_controls_frame = CollapsableFrame("Robot Controls", collapsed=False)
        with robot_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Button(
                    "Open Gripper", clicked_fn=self._robot_controller.open_gripper
                )
                ui.Button(
                    "Close Gripper", clicked_fn=self._robot_controller.close_gripper
                )

                ui.Label("Set Axis1 position:")
                self._angular_drive_input_axis1 = ui.FloatField()
                ui.Button(
                    "Set position (0, 360)",
                    clicked_fn=lambda: self._robot_controller.set_angular_drive_target(
                        AXIS1_JOINT_PATH,
                        self._angular_drive_input_axis1.model.get_value_as_float(),
                    ),
                )

                ui.Label("Set Axis2 position:")
                self._prismatic_drive_input_axis2 = ui.FloatField()
                ui.Button(
                    "Set position (0, 3.0)",
                    clicked_fn=lambda: self._robot_controller.set_prismatic_joint_position(
                        AXIS2_JOINT_PATH,
                        self._prismatic_drive_input_axis2.model.get_value_as_float(),
                    ),
                )

                ui.Label("Set Axis3 position:")
                self._prismatic_drive_input_axis3 = ui.FloatField()
                ui.Button(
                    "Set position (0, 2.0)",
                    clicked_fn=lambda: self._robot_controller.set_prismatic_joint_position(
                        AXIS3_JOINT_PATH,
                        self._prismatic_drive_input_axis3.model.get_value_as_float(),
                    ),
                )

                ui.Label("Set Axis4 position:")
                self._angular_drive_input_axis4 = ui.FloatField()
                ui.Button(
                    "Set position (0, 360)",
                    clicked_fn=lambda: self._robot_controller.set_angular_drive_target(
                        AXIS4_JOINT_PATH,
                        self._angular_drive_input_axis4.model.get_value_as_float(),
                    ),
                )

        camera_controls_frame = CollapsableFrame("Camera Controls", collapsed=False)
        with camera_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # Button to refresh camera list
                ui.Button("Refresh Camera List", clicked_fn=self._refresh_camera_list)

                ui.Label("Available Cameras:(After refresh) : ")

                self._capture_button_container = ui.VStack()

                ui.Label("Capture from ALL Cameras:")

                ui.Button(
                    "Capture from All Cameras",
                    clicked_fn=self._capture_from_all_cameras,
                )

        self.frames.append(camera_controls_frame)
        self.frames.append(world_controls_frame)
        self.frames.append(scenario_frame)
        self.frames.append(robot_controls_frame)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        self._scenario = UDPScenario(robot_controller=self._robot_controller)

    def _select_scenario(self):
        """
        Selects the scenario from the dropdown when the "Select Scenario" button is clicked.
        """
        value_model = self._scenario_dropdown.model.get_item_value_model()
        selected_index = value_model.as_int

        scenario_names = list(self._scenarios.keys())

        if selected_index < 0 or selected_index >= len(scenario_names):
            print(f"⚠️ Error: Invalid scenario index {selected_index}")
            return

        selected_scenario = scenario_names[selected_index]
        print(f"Switching to scenario: {selected_scenario}")

        # Reset the previous scenario
        if self._scenario is not None:
            self._scenario.reset()

        scenario_cls = self._scenarios[selected_scenario]
        self._scenario = scenario_cls(robot_controller=self._robot_controller)
        self._setup_scenario()

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        create_new_stage()
        setup_scene()

        timeline_iface = omni.timeline.get_timeline_interface()
        timeline_iface.set_auto_update(False)

        print("Scene setup complete.")

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.
        """
        if self._scenario:
            self._scenario.setup()
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True
            self._reset_btn.enabled = True

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        if self._scenario:
            self._scenario.reset()
            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        This function will repeatedly advance the script in scenario.py until it is finished.

        Args:
            step (float): The dt of the current physics step
        """
        if self._scenario and self._scenario.update(step):
            self._scenario_state_btn.enabled = False

    def start_scenario(self):
        """Starts the selected scenario."""
        if self._scenario is None:
            print("No scenario selected! Click 'Select Scenario' first.")
            return

        print(f"Running scenario: {self._current_scenario_name}")
        self._timeline.play()

    def stop_scenario(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._scenario_state_btn.reset()
        self._scenario_state_btn.enabled = False
        self._reset_btn.enabled = False

    def _capture_from_camera(self, camera_id):
        """Capture an image from the specified camera."""
        image_path = self._robot_controller.capture_from_camera(camera_id)
        print(f"Captured image from {camera_id}: {image_path}")

    def _capture_from_all_cameras(self):
        """Capture images from all registered cameras."""
        image_paths = self._robot_controller.capture_from_all_cameras()
        print(f"Captured images: {image_paths}")

    def _capture_all_images(self):
        """Capture images from all registered cameras."""
        image_paths = self._robot_controller.capture_from_all_cameras()
        if image_paths:
            print("Captured images from all cameras:", image_paths)
        else:
            print("Failed to capture images from all cameras.")

    def _refresh_camera_list(self):
        """Update camera dropdown with the latest registered cameras and add capture button."""
        try:
            # Get the current list of registered cameras
            cameras = self._camera_capture.get_registered_cameras()
            print(f"Refresh found cameras: {cameras}")

            self._capture_button_container.clear()

            for camera_id in cameras:
                self._add_capture_button(camera_id)

        except Exception as e:
            print(f"Error refreshing camera list: {e}")

    def _add_capture_button(self, camera_id):
        """Adds a capture button for the specified camera."""
        with self._capture_button_container:
            self._capture_button = ui.Button(
                f"Capture from {camera_id}",
                clicked_fn=lambda: self._capture_from_camera(camera_id),
            )
            print(f"Added button for camera: {camera_id}")

    def _remove_capture_button(self):
        """Removes the capture button if it exists."""
        if hasattr(self, "_capture_button"):
            self._capture_button.destroy()
            del self._capture_button
            print("Removed capture button")
