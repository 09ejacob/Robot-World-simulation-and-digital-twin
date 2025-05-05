import numpy as np
from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
import omni.graph as og2
import omni.usd
from omni.isaac.dynamic_control import _dynamic_control
from pxr import UsdGeom
from pxr import Gf
from omni.isaac.core.articulations import ArticulationView
from isaacsim.sensors.physics import ContactSensor
from omni.physx import get_physx_simulation_interface
from omni.physx.bindings._physx import IPhysxSimulation
from omni.physx.scripts.physicsUtils import PhysicsSchemaTools
import omni.kit.commands

from ..camera_capture import CameraCapture

from ..global_variables import (
    GRIPPER_CLOSE_PATH,
    GRIPPER_OPEN_PATH,
    ROBOT_PATH,
    BOTTLEGRIPPER_JOINT_PATH_RIGHT,
    BOTTLEGRIPPER_JOINT_PATH_LEFT,
    BOTTLEGRIPPER_OPEN,
    BOTTLEGRIPPER_CLOSE,
    BOTTLEGRIPPER_IDLE,
    GRIPPER_PATH,
    ENVIRONMENT_PATH,
    PALLET_STACK_PATH,
)


class RobotController:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.camera_capture = CameraCapture()
        self.dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        self.articulation = self.dc_interface.get_articulation(ROBOT_PATH)

        self._contact_sensor = None

    def _ensure_contact_sensor(self):
        """
        Ensure that the ContactSensor exists on the gripper prim.

        Returns:
            bool: True if the sensor is ready, False if not.
        """
        if self._contact_sensor is None:
            try:
                self._contact_sensor = ContactSensor(
                    prim_path=f"{GRIPPER_PATH}/Contact_Sensor",
                    name="Contact_Sensor",
                    frequency=1,
                    translation=np.zeros(3),
                    radius=-1.0,
                    min_threshold=0.0,
                    max_threshold=1e6,
                )
            except Exception as e:
                print(f"[WARN] couldn’t create ContactSensor yet: {e}")
                return False
        return True

    def refresh_handles(self):
        """Refresh the articulation handle."""
        self.articulation = self.dc_interface.get_articulation(ROBOT_PATH)

    def open_gripper(self):
        """Open the gripper or bottlegripper."""
        stage = get_current_stage()
        left = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_LEFT)
        right = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_RIGHT)
        if left.IsValid() and right.IsValid():
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_RIGHT,
                BOTTLEGRIPPER_OPEN,
            )
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_LEFT,
                BOTTLEGRIPPER_OPEN * (-1),
            )
        else:
            node = og2.core.get_node_by_path(GRIPPER_OPEN_PATH)
            attr = node.get_attribute("state:enableImpulse")
            attr.set(1)
            node.request_compute()

    def close_gripper(self):
        """Close the gripper or bottlegripper."""
        stage = get_current_stage()
        left = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_LEFT)
        right = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_RIGHT)
        if left.IsValid() and right.IsValid():
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_RIGHT,
                BOTTLEGRIPPER_CLOSE,
            )
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_LEFT,
                BOTTLEGRIPPER_CLOSE * (-1),
            )
        else:
            node = og2.core.get_node_by_path(GRIPPER_CLOSE_PATH)
            attr = node.get_attribute("state:enableImpulse")
            attr.set(1)
            node.request_compute()

    def set_bottlegripper_to_idle_pos(self):
        """Move the bottlegripper to its idle position."""
        stage = get_current_stage()
        left = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_LEFT)
        right = stage.GetPrimAtPath(BOTTLEGRIPPER_JOINT_PATH_RIGHT)
        if left.IsValid() and right.IsValid():
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_RIGHT,
                BOTTLEGRIPPER_IDLE,
            )
            self.set_prismatic_joint_position(
                BOTTLEGRIPPER_JOINT_PATH_LEFT,
                BOTTLEGRIPPER_IDLE,
            )
        else:
            print("Bottlegripper is not active")

    def set_angular_drive_target(self, joint_prim_path, target_position):
        """
        Drive an angular joint to a target angle (degrees).

        Args:
            joint_prim_path (str): USD path of the joint.
            target_position (float): desired angle in degrees.
        """
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
        """
        Drive a prismatic joint to a target linear position.

        Args:
            joint_prim_path (str): USD path of the joint.
            position (float): desired position in meters.
        """
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

    def get_dof_index_for_joint(self, joint_prim_path) -> int:
        """
        Look up the DOF index in dynamic control for a given joint path.

        Returns:
            int: the DOF index, or -1 if it was not found.
        """
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

    def get_joint_position_by_index(self, dof_index, is_angular=False):
        """
        Read the current joint position from dynamic control.

        Args:
            dof_index (int): the DOF index.
            is_angular (bool): if True, return degrees instead of meters.

        Returns:
            float or None: current position of the joint or None if invalid.
        """
        if not self.articulation:
            return None

        dof_states = self.dc_interface.get_articulation_dof_states(
            self.articulation, _dynamic_control.STATE_POS
        )

        if dof_index < 0 or dof_index >= len(dof_states["pos"]):
            return None

        current_pos = dof_states["pos"][dof_index]
        if is_angular:
            return np.rad2deg(current_pos)
        return current_pos

    def print_joint_position_by_index(self, dof_index, is_angular=False):
        """Print the current joint position."""
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

    def wait_for_joint_position(
        self,
        dof_index,
        target_position,
        pos_threshold=0.01,
        max_frames=1000,
        is_angular=False,
    ):
        """
        Yields until the joint reaches the target position within the threshold.
        """
        frames = 0
        if is_angular:
            target_position = np.deg2rad(target_position)
            pos_threshold = np.deg2rad(pos_threshold)

        while True:
            dof_states = self.dc_interface.get_articulation_dof_states(
                self.articulation, _dynamic_control.STATE_POS
            )
            current_pos = dof_states["pos"][dof_index]

            if is_angular:
                target_position = (target_position + np.pi) % (2 * np.pi) - np.pi
                current_pos = (current_pos + np.pi) % (2 * np.pi) - np.pi

            if abs(current_pos - target_position) < pos_threshold:
                break

            frames += 1
            if frames >= max_frames:
                break

            yield

    def teleport_robot(self, position):
        """
        Teleport the robot to a given position.

        Args:
            position (tuple): (x, y, z) position for where to move the robot.
        """
        stage = omni.usd.get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
        if not robot_prim.IsValid():
            print("Robot prim not found at /World/Robot")
            return

        xformable = UsdGeom.Xformable(robot_prim)
        xformable.ClearXformOpOrder()
        translate_op = xformable.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*position))

    def get_force_sensor_data(self):
        """
        Returns the latest contact-sensor frame.
        """
        if not self._ensure_contact_sensor():
            return
        data = self._contact_sensor.get_current_frame()

        return data

    def _get_colliding_prim(self) -> list[str]:
        """
        Query PhysX for all prims which are currently colliding with the gripper.

        Returns:
            list[str]: USD paths of colliding prims.
        """
        sim = get_physx_simulation_interface()
        contact_headers, _ = sim.get_contact_report()

        collided = set()
        for hdr in contact_headers:
            primA = PhysicsSchemaTools.intToSdfPath(hdr.actor0).pathString
            primB = PhysicsSchemaTools.intToSdfPath(hdr.actor1).pathString

            if primA == GRIPPER_PATH and not primB.startswith(GRIPPER_PATH):
                collided.add(primB)
            elif primB == GRIPPER_PATH and not primA.startswith(GRIPPER_PATH):
                collided.add(primA)

        return list(collided)

    def add_colliding_item(self):
        """For each item which is colliding with the gripper, move it into the under the pallet prim of the robot."""
        for p in self._get_colliding_prim():
            if not p.startswith(ENVIRONMENT_PATH) or not p.rsplit("/", 1)[
                -1
            ].startswith("box_"):
                continue

            leaf = p.rsplit("/", 1)[-1]
            dest = f"{PALLET_STACK_PATH}/{leaf}"

            omni.kit.commands.execute(
                "MovePrims", paths_to_move={p: dest}, keep_world_transform=True
            )

    def capture_cameras(
        self, cameras=None, udp_controller=None, host=None, port=None, stream=False
    ):
        """
        Capture images from specified cameras, with optional UDP streaming.

        Args:
            cameras (list[str], optional): Camera IDs to capture from. Defaults to all registered.
            udp_controller (UDPController, optional): Used for streaming
            host (str, optional): Target host for UDP
            port (int, optional): Target port for UDP
            stream (bool): Whether to stream over UDP

        Returns:
            dict: Map of camera ID to saved image path (or None if failed)
        """
        if cameras is None:
            cameras = self.camera_capture.get_registered_cameras()
        elif isinstance(cameras, str):
            cameras = [cameras]

        results = {}

        for cam_id in cameras:
            print(f"[DEBUG] Capturing from camera: {cam_id}")
            if stream and udp_controller and host and port:
                result = self.camera_capture.capture_and_stream(
                    cam_id, udp_controller, host, port
                )
            else:
                result = self.camera_capture.capture_image(cam_id)

            results[cam_id] = result

        return results

    
    
    def capture_stereo_pointcloud(self,stereo_pair="main_stereo"):
        """
        Capture a stereo point cloud from the camera pair

        Args:
            stereo_pair (str): ID of the stereo camera pair to capture from

        Returns:
            str: Path to the saved point cloud file, or None if capture failed
        """
        # Capture the point cloud
        result = self.camera_capture.save_stereo_pointcloud_pair(stereo_pair)
        return result


    def generate_video(self, fps):
        """Convert previously captured frames into a video at a given framerate."""
        self.camera_capture.convert_video_from_images(fps)
