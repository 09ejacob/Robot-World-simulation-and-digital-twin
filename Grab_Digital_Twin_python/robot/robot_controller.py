import numpy as np
from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
import omni.graph as og
from pxr import Usd, UsdGeom
import omni.usd
import omni.graph as og2
from omni.isaac.dynamic_control import _dynamic_control

from ..global_variables import GRIPPER_CLOSE_PATH, GRIPPER_OPEN_PATH

stage = omni.usd.get_context().get_stage()


def open_gripper():
    ogn1 = og2.core.get_node_by_path(GRIPPER_OPEN_PATH)

    attr = ogn1.get_attribute("state:enableImpulse")
    attr.set(1)
    ogn1.request_compute()


def close_gripper():
    ogn2 = og2.core.get_node_by_path(GRIPPER_CLOSE_PATH)

    attr = ogn2.get_attribute("state:enableImpulse")
    attr.set(1)
    ogn2.request_compute()


def set_angular_drive_target(joint_prim_path, target_position):
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


def set_prismatic_joint_position(joint_prim_path, position):
    stage = get_current_stage()
    joint_prim = stage.GetPrimAtPath(joint_prim_path)
    if not joint_prim.IsValid():
        print(f"Joint prim at path {joint_prim_path} is not valid.")
        return

    # Still clamp using the lower and upper limit attributes:
    lower_limit_attr = joint_prim.GetAttribute("physics:lowerLimit")
    upper_limit_attr = joint_prim.GetAttribute("physics:upperLimit")
    if not lower_limit_attr or not upper_limit_attr:
        print(f"Prismatic joint at {joint_prim_path} does not have limit attributes.")
        return

    # Retrieve the DriveAPI directly
    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
    if not drive_api:
        print(
            f"No linear drive is applied to the prismatic joint at {joint_prim_path}."
        )
        return

    lower_limit = lower_limit_attr.Get()
    upper_limit = upper_limit_attr.Get()
    clamped_position = max(lower_limit, min(position, upper_limit))

    # Now set the drive API's target position:
    drive_api.GetTargetPositionAttr().Set(clamped_position)

    # print(
    #     f"Prismatic joint at {joint_prim_path} -> targetPosition = {clamped_position}"
    # )


def read_force_sensor_value():
    dc_interface = _dynamic_control.acquire_dynamic_control_interface()

    articulation = dc_interface.get_articulation("/World/Robot")
    dof_states = dc_interface.get_articulation_dof_states(
        articulation, _dynamic_control.STATE_ALL
    )

    sensor_dof_index = 2
    force_value = dof_states["effort"][sensor_dof_index]
    print("Effort sensor reading:", force_value)


def get_dof_index_for_joint_prim_path(
    dc_interface, articulation, joint_prim_path
) -> int:
    """
    Find which DOF index in 'articulation' corresponds to the single-DOF joint
    at USD path 'joint_prim_path'.

    Returns an integer DOF index, or -1 if not found.
    """
    joint_count = dc_interface.get_articulation_joint_count(articulation)
    for j in range(joint_count):
        joint_handle = dc_interface.get_articulation_joint(articulation, j)
        path = dc_interface.get_joint_path(joint_handle)
        if path == joint_prim_path:
            dof_handle = dc_interface.get_joint_dof(joint_handle, 0)

            dof_count = dc_interface.get_articulation_dof_count(articulation)
            for dof_index in range(dof_count):
                candidate_handle = dc_interface.get_articulation_dof(
                    articulation, dof_index
                )
                if candidate_handle == dof_handle:
                    return dof_index
    return -1


def wait_for_joint_position(
    dc_interface,
    articulation,
    dof_index,
    target_position,
    pos_threshold=0.01,
    max_frames=1000,
    is_angular=False,
):
    """
    Waits until a joint reaches the target position or times out.

    Parameters:
    dc_interface : DynamicControl - Interface for articulation states.
    articulation : int - Handle for the robot.
    dof_index : int - Joint DOF index.
    target_position : float - Target (degrees for revolute, meters for prismatic).
    pos_threshold : float - Allowed error (default: 0.01).
    max_frames : int - Max simulation steps before timeout (default: 500).
    is_angular : bool - Convert degrees to radians if True (default: False).

    Example:
    ```python
    yield from wait_for_joint_position(dc_interface, articulation, axis1_dof_index, 180, 1.0, 1000, True)
    ```
    """

    frames = 0

    # If angular, convert degrees to radians
    if is_angular:
        target_position = np.deg2rad(target_position)
        pos_threshold = np.deg2rad(pos_threshold)

    while True:
        # Get the current joint position
        dof_states = dc_interface.get_articulation_dof_states(
            articulation, _dynamic_control.STATE_POS
        )
        current_pos = dof_states["pos"][dof_index]

        # For debugging
        if frames % 10 == 0:
            unit = "rad" if is_angular else "m"
        #    print(
        #        f"Frame {frames}: DOF {dof_index} position = {current_pos} {unit}, Target = {target_position} {unit}"
        #    )

        # If angular, normalize the angles to be within [-π, π] range
        if is_angular:
            target_position = (target_position + np.pi) % (2 * np.pi) - np.pi
            current_pos = (current_pos + np.pi) % (2 * np.pi) - np.pi

        # Check if position is close enough
        if abs(current_pos - target_position) < pos_threshold:
            break

        frames += 1
        if frames >= max_frames:
            # print(f"Timeout waiting for DOF {dof_index} to reach {target_position}")

            break  # Timeout

        yield
