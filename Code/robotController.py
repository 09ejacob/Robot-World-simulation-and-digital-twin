from pxr import UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage


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
        f"Target position set to {target_position} radians for joint at {joint_prim_path}."
    )


set_angular_drive_target("/World/Robot/Joints/RevoluteJointAxis1", target_position=180)
