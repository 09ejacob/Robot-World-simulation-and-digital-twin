from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage
from SetupScene import surface_gripper
from omni.isaac.core.simulation_context import SimulationContext


def open_gripper():
    if surface_gripper is not None:
        surface_gripper.open()
        print("Gripper opened (deactivated).")
    else:
        print("No surface gripper found.")


def close_gripper():
    if surface_gripper is not None:
        surface_gripper.close()
        print("Gripper closed (activated).")
    else:
        print("No surface gripper found.")


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


set_angular_drive_target("/World/Robot/Joints/RevoluteJointAxis1", target_position=180)
set_prismatic_joint_position(
    "/World/Robot/Joints/PrismaticJointAxis2", position=-1.5
)  # From -1.5 to 0.8
