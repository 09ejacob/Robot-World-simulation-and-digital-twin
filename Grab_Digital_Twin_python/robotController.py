from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
import omni.graph as og
from pxr import Usd, UsdGeom
import omni.usd
import omni.graph as og2

stage = omni.usd.get_context().get_stage()


def open_gripper():
    ogn1 = og2.core.get_node_by_path(
        "/World/Robot/Tower/Axis2/gripper/SurfaceGripperActionGraph/open"
    )

    attr = ogn1.get_attribute("state:enableImpulse")
    attr.set(1)
    ogn1.request_compute()


def close_gripper():
    ogn2 = og2.core.get_node_by_path(
        "/World/Robot/Tower/Axis2/gripper/SurfaceGripperActionGraph/close"
    )

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
    print(
        f"Target position set to {target_position} degrees for joint at {joint_prim_path}."
    )


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

    print(
        f"Prismatic joint at {joint_prim_path} -> targetPosition = {clamped_position}"
    )
