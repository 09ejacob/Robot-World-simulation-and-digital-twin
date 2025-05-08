# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import carb
import omni.timeline
import omni.ui as ui
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.gui.components.element_wrappers import CollapsableFrame, StateButton
from isaacsim.gui.components.ui_utils import get_style
from ..scenes.setup_scene import setup_scene
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    ROBOT_PATH,
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

        # Lists elements so they can be enabled/disabled or shown/hidden based on simulation state
        self._robot_controls_widgets = []
        self._camera_controls_widgets = []
        self._dynamic_camera_buttons = []

        self._grab_usd_options = ["Grab.usd", "Grab-bottlegripper.usd"]

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._robot_controller = RobotController()
        self._camera_capture = CameraCapture()
        self._scenarios = {
            "UDP": UDPScenario,
            "Pick Boxes": PickBoxesScenario,
            "Stack Box": StackBoxScenario,
        }
        self._scenario_options = ["-- Select Scenario --"] + list(
            self._scenarios.keys()
        )
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
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._set_robot_camera_controls_enabled(True)
        elif event.type in (
            int(omni.timeline.TimelineEventType.STOP),
            int(omni.timeline.TimelineEventType.PAUSE),
        ):
            self._set_robot_camera_controls_enabled(False)

        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._scenario_state_btn.reset()
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
                self._enable_cameras_model = ui.SimpleBoolModel(True)
                self._enable_3d_model = ui.SimpleBoolModel(False)

                with ui.HStack():
                    ui.Label("Enable Cameras")
                    ui.CheckBox(model=self._enable_cameras_model)

                with ui.HStack():
                    ui.Label("Enable 3D Features")
                    self._3d_checkbox = ui.CheckBox(model=self._enable_3d_model)

                def _update_3d_checkbox_enabled_state(model=None):
                    self._3d_checkbox.enabled = (
                        self._enable_cameras_model.get_value_as_bool()
                    )

                # Initialize state
                _update_3d_checkbox_enabled_state()

                # React to changes in the camera checkbox
                self._enable_cameras_model.add_value_changed_fn(
                    _update_3d_checkbox_enabled_state
                )

                ui.Label("Select Grab USD:")
                self._grab_dropdown = ui.ComboBox(0, *self._grab_usd_options)

                ui.Label("Physics Rate (Hz):")
                self._physics_rate_model = ui.SimpleFloatModel(60.0)
                ui.FloatField(model=self._physics_rate_model)

                ui.Label("Rendering Rate (Hz):")
                self._rendering_rate_model = ui.SimpleFloatModel(60.0)
                ui.FloatField(model=self._rendering_rate_model)

                self._load_btn = LoadButton(
                    "Load Grab USD",
                    "LOAD",
                    setup_scene_fn=self._setup_scene,
                    setup_post_load_fn=self._setup_scenario,
                )

                def _on_rate_changed(model):
                    hz_phys = max(0.1, self._physics_rate_model.get_value_as_float())
                    hz_rend = max(0.1, self._rendering_rate_model.get_value_as_float())
                    self._load_btn.set_world_settings(
                        physics_dt=1.0 / hz_phys,
                        rendering_dt=1.0 / hz_rend,
                    )

                # subscribe to both models:
                self._physics_rate_model.add_value_changed_fn(_on_rate_changed)
                self._rendering_rate_model.add_value_changed_fn(_on_rate_changed)

                # set initial values immediately
                _on_rate_changed(None)

                self.wrapped_ui_elements.append(self._load_btn)

        scenario_frame = CollapsableFrame("Scenario", collapsed=False)
        with scenario_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                ui.Label("Select Scenario:")

                self._scenario_dropdown = ui.ComboBox(
                    0,
                    *self._scenario_options,
                )

                self._load_scenario_button = ui.Button(
                    "Load Scenario",
                    clicked_fn=self._select_scenario,
                )

                self._scenario_state_btn = StateButton(
                    "Run Scenario",
                    "RUN",
                    "STOP",
                    on_a_click_fn=self._start_scenario,
                    on_b_click_fn=self._stop_scenario,
                    physics_callback_fn=self._update_scenario,
                )
                self._scenario_state_btn.enabled = False
                self.wrapped_ui_elements.append(self._scenario_state_btn)

                self._unload_btn = ResetButton(
                    "Unload Scenario",
                    "UNLOAD SCENARIO",
                    pre_reset_fn=None,
                    post_reset_fn=self._on_post_unload_btn,
                )
                self._unload_btn.enabled = False
                self.wrapped_ui_elements.append(self._unload_btn)

        robot_controls_frame = CollapsableFrame("Robot Controls", collapsed=False)
        with robot_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                openGripperButton = ui.Button(
                    "Open Gripper", clicked_fn=self._robot_controller.open_gripper
                )
                self._robot_controls_widgets.append(openGripperButton)

                closeGripperButton = ui.Button(
                    "Close Gripper", clicked_fn=self._robot_controller.close_gripper
                )
                self._robot_controls_widgets.append(closeGripperButton)

                bottleGripperIdleButton = ui.Button(
                    "Set bottlegripper to idle",
                    clicked_fn=self._robot_controller.set_bottlegripper_to_idle_pos,
                )
                self._robot_controls_widgets.append(bottleGripperIdleButton)

                ui.Label("Set Axis1 position (-360, 360):")
                self._angular_drive_input_axis1 = ui.FloatField()
                axis1Button = ui.Button(
                    "Set position",
                    clicked_fn=lambda: self._robot_controller.set_angular_drive_target(
                        AXIS1_JOINT_PATH,
                        self._angular_drive_input_axis1.model.get_value_as_float(),
                    ),
                )
                self._robot_controls_widgets.append(axis1Button)

                ui.Label("Set Axis2 position (0, 3.0):")
                self._prismatic_drive_input_axis2 = ui.FloatField()
                axis2Button = ui.Button(
                    "Set position",
                    clicked_fn=lambda: self._robot_controller.set_prismatic_joint_position(
                        AXIS2_JOINT_PATH,
                        self._prismatic_drive_input_axis2.model.get_value_as_float(),
                    ),
                )
                self._robot_controls_widgets.append(axis2Button)

                ui.Label("Set Axis3 position (0, 2.0):")
                self._prismatic_drive_input_axis3 = ui.FloatField()
                axis3Button = ui.Button(
                    "Set position",
                    clicked_fn=lambda: self._robot_controller.set_prismatic_joint_position(
                        AXIS3_JOINT_PATH,
                        self._prismatic_drive_input_axis3.model.get_value_as_float(),
                    ),
                )
                self._robot_controls_widgets.append(axis3Button)

                ui.Label("Set Axis4 position (-360, 360):")
                self._angular_drive_input_axis4 = ui.FloatField()
                axis4Button = ui.Button(
                    "Set position",
                    clicked_fn=lambda: self._robot_controller.set_angular_drive_target(
                        AXIS4_JOINT_PATH,
                        self._angular_drive_input_axis4.model.get_value_as_float(),
                    ),
                )
                self._robot_controls_widgets.append(axis4Button)

        camera_controls_frame = CollapsableFrame("Camera Controls", collapsed=False)
        with camera_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # Button to refresh camera list
                refreshCameraButton = ui.Button(
                    "Refresh Camera List", clicked_fn=self._refresh_camera_list
                )
                self._camera_controls_widgets.append(refreshCameraButton)

                ui.Label("Available Cameras:(After refresh) : ")

                self._capture_button_container = ui.VStack()

                ui.Label("Capture from ALL Cameras:")

                captureAllCamerasButton = ui.Button(
                    "Capture from All Cameras",
                    clicked_fn=self._capture_from_all_cameras,
                )
                self._camera_controls_widgets.append(captureAllCamerasButton)

                ui.Separator()
                ui.Label("Stereo Camera Controls:")
                capturePointCloudButton = ui.Button(
                    "Capture pointclouds", clicked_fn=self._capture_pointclouds
                )
                self._camera_controls_widgets.append(capturePointCloudButton)

        self.frames.append(camera_controls_frame)
        self.frames.append(world_controls_frame)
        self.frames.append(scenario_frame)
        self.frames.append(robot_controls_frame)

        is_running = self._timeline.is_playing()
        self._set_robot_camera_controls_enabled(is_running)

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        """Reset UIBuilders internal state."""
        self._scenario = None

    def _select_scenario(self):
        """
        Selects the scenario from the dropdown when the "Load Scenario" button is clicked.
        """
        idx = self._scenario_dropdown.model.get_item_value_model().as_int

        if idx <= 0 or idx >= len(self._scenario_options):
            carb.log_warn("Please select a valid scenario from the dropdown.")
            return

        selected_scenario = self._scenario_options[idx]
        print(f"[MAIN] Switching to scenario: {selected_scenario}")
        self._current_scenario_name = selected_scenario

        if self._scenario is not None:
            self._scenario.unload()

        scenario_cls = self._scenarios[selected_scenario]
        enable_cameras = self._enable_cameras_model.get_value_as_bool()
        self._scenario = scenario_cls(
            robot_controller=self._robot_controller,
            allow_udp_capture=enable_cameras,
        )
        self._setup_scenario()

    def _setup_scene(self):
        """
        This function is attached to the Load Button as the setup_scene_fn callback.
        On pressing the Load Button, a new instance of World() is created and then this function is called.
        The user should now load their assets onto the stage and add them to the World Scene.
        """
        usd_idx = self._grab_dropdown.model.get_item_value_model().as_int
        usd_file = (
            self._grab_usd_options[usd_idx]
            if 0 <= usd_idx < len(self._grab_usd_options)
            else self._grab_usd_options[0]
        )

        self._camera_capture.initialize()
        create_new_stage()
        setup_scene(
            grab_usd=usd_file,
            enable_cameras=self._enable_cameras_model.get_value_as_bool(),
            enable_3d_features=self._enable_3d_model.get_value_as_bool(),
            enable_overview_camera=False,
        )

        omni.timeline.get_timeline_interface().set_auto_update(False)
        print(f"[MAIN] Scene setup complete with grab asset: {usd_file}")

    def _setup_scenario(self):
        """
        This function is attached to the Load Button as the setup_post_load_fn callback.
        The user may assume that their assets have been loaded by their setup_scene_fn callback, that
        their objects are properly initialized, and that the timeline is paused on timestep 0.
        """
        if self._scenario:
            self._scenario.setup()

            self._robot_controller._ensure_contact_sensor()

            self._scenario_state_btn.reset()
            self._scenario_state_btn.enabled = True
            self._unload_btn.enabled = True

    def _on_post_unload_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """
        if self._scenario:
            self._scenario.unload()
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

    def _start_scenario(self):
        """Starts the selected scenario."""
        if self._scenario is None:
            carb.log_warn("No scenario selected. Click 'Select Scenario' first.")
            return

        print(f"[MAIN] Running scenario: {self._current_scenario_name}")
        self._timeline.play()

    def _stop_scenario(self):
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
        self._unload_btn.enabled = False

    def _is_robot_loaded(self) -> bool:
        stage = omni.usd.get_context().get_stage()
        return stage.GetPrimAtPath(ROBOT_PATH).IsValid()

    def _set_robot_camera_controls_enabled(self, sim_running: bool):
        robot_loaded = self._is_robot_loaded()
        visible = sim_running and robot_loaded

        for widget in self._robot_controls_widgets + self._camera_controls_widgets:
            widget.enabled = visible

        for btn in self._dynamic_camera_buttons:
            btn.visible = visible

        if hasattr(self, "_load_scenario_button"):
            self._load_scenario_button.enabled = robot_loaded

    def _capture_from_camera(self, camera_id):
        """Capture an image from the specified camera."""
        image_path = self._robot_controller.capture_cameras(camera_id)
        print(f"[MAIN] Captured image from {camera_id}: {image_path}")

    def _capture_3d_from_camera(self, camera_id):
        """Capture an image from the specified camera."""
        image_path = self._camera_capture.capture_pointcloud(camera_id)
        print(f"[MAIN] Captured 3D image from {camera_id}: {image_path}")

    def _capture_from_all_cameras(self):
        """Capture images from all registered cameras."""
        image_paths = self._robot_controller.capture_cameras()
        print(f"[MAIN] Captured images from all cameras: {image_paths}")

    def _capture_all_images(self):
        """Capture images from all registered cameras."""
        image_paths = self._robot_controller.capture_cameras()
        if image_paths:
            print(f"[MAIN] Captured images from all cameras: {image_paths}")
        else:
            carb.log_warn("No images were captured from any camera.")

    def _refresh_camera_list(self):
        """Update camera dropdown with the latest registered cameras and add capture button."""
        try:
            # Get the current list of registered cameras
            cameras = self._camera_capture.get_registered_cameras()
            print(f"[DEBUG] Refresh found cameras: {cameras}")

            self._capture_button_container.clear()
            self._dynamic_camera_buttons.clear()

            for camera_id in cameras:
                self._add_capture_button(camera_id)

        except Exception as e:
            carb.log_error(f"Error refreshing camera list: {e}")

    def _add_capture_button(self, camera_id):
        """Adds a capture button for the specified camera."""
        with self._capture_button_container:
            btn = ui.Button(
                f"Capture from {camera_id}",
                clicked_fn=lambda: self._capture_from_camera(camera_id),
            )
            self._dynamic_camera_buttons.append(btn)
            print(f"[DEBUG] Added capture button for camera: {camera_id}")

    def _remove_capture_button(self):
        """Removes the capture button if it exists."""
        if hasattr(self, "_capture_button"):
            self._capture_button.destroy()
            del self._capture_button
            print("[DEBUG] Removed capture button")

    def _capture_pointclouds(self):
        """Test the stereo camera setup."""
        self._robot_controller.capture_stereo_pointcloud()
