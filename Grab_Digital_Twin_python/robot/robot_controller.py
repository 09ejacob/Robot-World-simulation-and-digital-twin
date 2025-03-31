import numpy as np
from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
import omni.graph as og2
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control
from ..camera_capture import CameraCapture
import time


from ..global_variables import GRIPPER_CLOSE_PATH, GRIPPER_OPEN_PATH, ROBOT_PATH


class RobotController:
    def __init__(self):
        self.camera_capture = CameraCapture()
        self.stage = omni.usd.get_context().get_stage()

        if self.stage is None:
            print("[RobotController] WARNING: Stage is None")
        else:
            print(
                f"[RobotController] Stage Root Layer: {self.stage.GetRootLayer().identifier}"
            )

        self.dc_interface = _dynamic_control.acquire_dynamic_control_interface()

        if self.dc_interface is None:
            print(
                "[RobotController] ERROR: Failed to acquire Dynamic Control interface"
            )

        try:
            self.articulation = self.dc_interface.get_articulation(ROBOT_PATH)
            if self.articulation is None:
                print("[RobotController] WARNING: Initial articulation handle is None")
        except Exception as e:
            print(f"[RobotController] Exception getting articulation: {e}")

    def refresh_handles(self):
        self.articulation = self.dc_interface.get_articulation(ROBOT_PATH)

    def open_gripper(self):
        node = og2.core.get_node_by_path(GRIPPER_OPEN_PATH)
        if node is None:
            print(f"[open_gripper] Node not found at: {GRIPPER_OPEN_PATH}")
            return
        attr = node.get_attribute("state:enableImpulse")
        if attr is None:
            print(f"[open_gripper] Attribute 'state:enableImpulse' not found on node.")
            return
        attr.set(1)
        node.request_compute()

    def close_gripper(self):
        node = og2.core.get_node_by_path(GRIPPER_CLOSE_PATH)
        if node is None:
            print(f"[close_gripper] Node not found at: {GRIPPER_CLOSE_PATH}")
            return
        attr = node.get_attribute("state:enableImpulse")
        if attr is None:
            print(f"[close_gripper] Attribute 'state:enableImpulse' not found on node.")
            return
        attr.set(1)
        node.request_compute()

    def set_angular_drive_target(self, joint_prim_path, target_position):
        stage = get_current_stage()
        joint_prim = stage.GetPrimAtPath(joint_prim_path)

        if not joint_prim.IsValid():
            print(f"Joint prim at path {joint_prim_path} is not valid.")
            return

        # Ensure the joint has the PhysicsAngularDrive API applied
        drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if not drive_api:
            print(f"Angular drive API not found on joint at {joint_prim_path}.")
            return

        # Set the target position
        drive_api.GetTargetPositionAttr().Set(target_position)
        # print(
        #     f"Target position set to {target_position} degrees for joint at {joint_prim_path}."
        # )

    def set_prismatic_joint_position(self, joint_prim_path, position):
        stage = get_current_stage()
        joint_prim = stage.GetPrimAtPath(joint_prim_path)
        if not joint_prim.IsValid():
            print(f"Joint prim at path {joint_prim_path} is not valid.")
            return

        lower_limit_attr = joint_prim.GetAttribute("physics:lowerLimit")
        upper_limit_attr = joint_prim.GetAttribute("physics:upperLimit")
        if not lower_limit_attr or not upper_limit_attr:
            print(
                f"Prismatic joint at {joint_prim_path} does not have limit attributes."
            )
            return

        drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
        if not drive_api:
            print(
                f"No linear drive is applied to the prismatic joint at {joint_prim_path}."
            )
            return

        lower_limit = lower_limit_attr.Get()
        upper_limit = upper_limit_attr.Get()
        clamped_position = max(lower_limit, min(position, upper_limit))
        drive_api.GetTargetPositionAttr().Set(clamped_position)

    def read_force_sensor_value(self):
        dof_states = self.dc_interface.get_articulation_dof_states(
            self.articulation, _dynamic_control.STATE_ALL
        )
        sensor_dof_index = 2
        force_value = dof_states["effort"][sensor_dof_index]
        print("Effort sensor reading:", force_value)
        return force_value

    def get_dof_index_for_joint(self, joint_prim_path) -> int:
        joint_count = self.dc_interface.get_articulation_joint_count(self.articulation)
        for j in range(joint_count):
            joint_handle = self.dc_interface.get_articulation_joint(
                self.articulation, j
            )
            path = self.dc_interface.get_joint_path(joint_handle)
            if path == joint_prim_path:
                dof_handle = self.dc_interface.get_joint_dof(joint_handle, 0)
                dof_count = self.dc_interface.get_articulation_dof_count(
                    self.articulation
                )
                for dof_index in range(dof_count):
                    candidate_handle = self.dc_interface.get_articulation_dof(
                        self.articulation, dof_index
                    )
                    if candidate_handle == dof_handle:
                        return dof_index
        return -1

    def wait_for_joint_position(
        self,
        dof_index,
        target_position,
        pos_threshold=0.01,
        max_frames=1000,
        is_angular=False,
    ):
        frames = 0
        if is_angular:
            target_position = np.deg2rad(target_position)
            pos_threshold = np.deg2rad(pos_threshold)

        while True:
            dof_states = self.dc_interface.get_articulation_dof_states(
                self.articulation, _dynamic_control.STATE_POS
            )
            current_pos = dof_states["pos"][dof_index]

            # For debugging
            # if frames % 10 == 0:
            #     unit = "rad" if is_angular else "m"
            #     print(
            #         f"Frame {frames}: DOF {dof_index} position = {currenta_pos} {unit}, Target = {target_position} {unit}"
            #     )

            if is_angular:
                target_position = (target_position + np.pi) % (2 * np.pi) - np.pi
                current_pos = (current_pos + np.pi) % (2 * np.pi) - np.pi

            if abs(current_pos - target_position) < pos_threshold:
                break

            frames += 1
            if frames >= max_frames:
                break

            yield

    def capture_from_camera(self, camera_id):
        """
        Capture an image from a specific camera

        Args:
            camera_id (str): ID of the camera to capture from

        Returns:
            str: Path to the saved image file, or None if capture failed
        """
        # Make sure timeline is playing to update frames

        # Capture the image
        result = self.camera_capture.capture_image(camera_id)
        return result

    def capture_from_all_cameras(self):
        """
        Capture images from all registered cameras

        Returns:
            dict: Map of camera IDs to saved image paths
        """
        # Make sure timeline is playing to update frames

        # Capture from all cameras
        results = self.camera_capture.capture_all_cameras()

        return results

    def get_registered_cameras(self):
        """
        Get list of registered camera IDs

        Returns:
            list: Camera IDs
        """
        return self.camera_capture.get_registered_cameras()

    def print_joint_position_by_index(self, dof_index, is_angular=False):
        if not self.articulation:
            print("Articulation handle is invalid.")
            return

        dof_states = self.dc_interface.get_articulation_dof_states(
            self.articulation, _dynamic_control.STATE_POS
        )

        if dof_index < 0 or dof_index >= len(dof_states["pos"]):
            print(f"Invalid DOF index: {dof_index}")
            return

        current_pos = dof_states["pos"][dof_index]
        if is_angular:
            current_pos_deg = np.rad2deg(current_pos)
            print(f"[DOF {dof_index}] Angular Position: {current_pos_deg:.3f} degrees")
        else:
            print(f"[DOF {dof_index}] Linear Position: {current_pos:.4f} meters")
