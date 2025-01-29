import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.surface_gripper import SurfaceGripper
import omni.usd
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom, UsdPhysics, Sdf, Gf

_surface_gripper = None


def get_gripper():
    if _surface_gripper is None:
        raise RuntimeError("SurfaceGripper is not initialized. Run setup_scene first.")
    return _surface_gripper


def create_ground_plane(path):
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))
    print("Created ground plane")


def create_xform(path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xform.Define(stage, path)

    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))

    print("Created Xform")


def enable_linear_drive(
    joint_prim_path, stiffness=10.0, damping=5.0, max_force=30.0, target_position=0.0
):
    stage = get_current_stage()
    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    if not joint_prim.IsValid():
        print(f"Joint prim at path {joint_prim_path} is not valid.")
        return

    # Ensure the joint is of type PhysicsPrismaticJoint
    if joint_prim.GetTypeName() != "PhysicsPrismaticJoint":
        print(f"Joint at path {joint_prim_path} is not a PhysicsPrismaticJoint.")
        return

    # Add the PhysicsDrive API with tag "linear"
    UsdPhysics.DriveAPI.Apply(joint_prim, "linear")

    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
    # Set drive parameters
    drive_api.GetStiffnessAttr().Set(stiffness)
    drive_api.GetDampingAttr().Set(damping)
    drive_api.GetMaxForceAttr().Set(max_force)
    # Set initial target
    drive_api.GetTargetPositionAttr().Set(target_position)

    print(f"Linear drive enabled for joint at {joint_prim_path}.")


def enable_angular_drive(
    joint_prim_path,
    stiffness=10.0,
    damping=5.0,
    max_force=30.0,
    target_position=0.0,
):
    stage = get_current_stage()
    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    if not joint_prim.IsValid():
        print(f"Joint prim at path {joint_prim_path} is not valid.")
        return

    # Ensure the joint is of type PhysicsRevoluteJoint
    if joint_prim.GetTypeName() != "PhysicsRevoluteJoint":
        print(f"Joint at path {joint_prim_path} is not a PhysicsRevoluteJoint.")
        return

    # Add the PhysicsAngularDrive API to the joint
    UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

    # Set drive parameters: stiffness, damping, max force
    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    drive_api.GetStiffnessAttr().Set(stiffness)
    drive_api.GetDampingAttr().Set(damping)
    drive_api.GetMaxForceAttr().Set(max_force)

    # Set initial drive target position
    drive_api.GetTargetPositionAttr().Set(target_position)

    print(f"Angular drive enabled for joint at {joint_prim_path}.")


def set_prismatic_joint_limits(joint_prim_path, lower_limit=None, upper_limit=None):
    stage = get_current_stage()
    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    if not joint_prim.IsValid():
        print(f"Joint prim at path {joint_prim_path} is not valid.")
        return

    if joint_prim.GetTypeName() != "PhysicsPrismaticJoint":
        print(f"Joint at path {joint_prim_path} is not a PhysicsPrismaticJoint.")
        return

    if lower_limit is not None:
        joint_prim.GetAttribute("physics:lowerLimit").Set(lower_limit)
    if upper_limit is not None:
        joint_prim.GetAttribute("physics:upperLimit").Set(upper_limit)

    print(f"Linear limits set for prismatic joint at {joint_prim_path}.")


def create_joint(
    joint_prim_path,
    object1_path,
    object2_path,
    joint_type,
    hinge_axis,
):
    stage = get_current_stage()

    create_prim(
        prim_path=joint_prim_path,
        prim_type=joint_type,
        attributes={"physics:axis": hinge_axis},
    )

    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    joint_prim.GetRelationship("physics:body0").SetTargets([Sdf.Path(object1_path)])
    joint_prim.GetRelationship("physics:body1").SetTargets([Sdf.Path(object2_path)])


def setup_robot(
    prim_path1,
    prim_path2,
    prim_path3,
    joint_prim_path1,
    joint_prim_path2,
    joint_prim_path3,
    position1=(0, 0, 0),
    scale1=(1, 1, 1),
    color1=(0, 0, 0),  # gripper
    position2=(0, 0, 0),
    scale2=(1, 1, 1),
    color2=(0, 0, 0),  # tower
    position3=(0, 0, 0),
    scale3=(1, 1, 1),
    color3=(0, 0, 0),
):  # snake
    # Shapes
    gripper = DynamicCuboid(
        prim_path=prim_path1,
        position=np.array(position1),
        scale=np.array(scale1),
        color=np.array(color1),
    )
    print("Created gripper")

    tower = DynamicCuboid(
        prim_path=prim_path2,
        position=np.array(position2),
        scale=np.array(scale2),
        color=np.array(color2),
    )
    print("Created tower")

    snake = DynamicCuboid(
        prim_path=prim_path3,
        position=np.array(position3),
        scale=np.array(scale3),
        color=np.array(color3),
    )
    print("Created snake")

    # Joints
    create_joint(
        joint_prim_path1,
        "/World/Robot/Tower/Axis2/snake",
        "/World/Robot/Tower/Axis2/gripper",
        "PhysicsRevoluteJoint",
        "Z",
    )  # Axis4 joint
    create_joint(
        joint_prim_path2,
        "/World/Robot/Tower/tower",
        "/World/Robot/Tower/Axis2/snake",
        "PhysicsPrismaticJoint",
        "Z",
    )  # Axis2 joint
    set_prismatic_joint_limits(joint_prim_path2, -1.5, 0.8)
    enable_linear_drive(
        joint_prim_path2, stiffness=100, damping=10, max_force=100, target_position=0.0
    )

    create_joint(
        joint_prim_path3,
        "/World/Robot/Tower/tower",
        "/World/groundPlane",
        "PhysicsRevoluteJoint",
        "Z",
    )  # Axis1 joint
    enable_angular_drive(joint_prim_path3)

    _surface_gripper = SurfaceGripper(
        usd_path=None,  # No external USD provided
        translate=-0.1,  # Offset in the gripper's local Z direction
        direction="z",  # Gripper's direction
        grip_threshold=0.02,  # Distance threshold for grasping
        force_limit=100.0,  # Maximum gripping force
        torque_limit=1000.0,  # Maximum gripping torque
        kp=1.0e4,  # Stiffness of the joint
        kd=1.0e3,  # Damping of the joint
        disable_gravity=True,  # Disable gravity for the gripper
    )
    _surface_gripper.initialize(root_prim_path=prim_path1)
    print("Surface Gripper initialized and attached to the gripper.")


def create_pick_box(prim_path, position=(0, 0, 0), scale=(1, 1, 1), color=(4, 4, 4)):
    pickBox = DynamicCuboid(
        prim_path=prim_path,
        position=np.array(position),
        scale=np.array(scale),
        color=np.array(color),
    )
    print("Created pick-box")


def setup_scene():
    print("Setting up scene...")

    create_ground_plane("/World/groundPlane")

    create_xform(
        "/World/Robot", translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    )
    create_xform(
        "/World/Robot/Tower", translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    )
    create_xform(
        "/World/Robot/Tower/Axis2",
        translate=(0, 0, 0),
        rotation=(0, 0, 0),
        scale=(1, 1, 1),
    )

    setup_robot(
        "/World/Robot/Tower/Axis2/gripper",
        "/World/Robot/Tower/tower",
        "/World/Robot/Tower/Axis2/snake",
        "/World/Robot/Joints/RevoluteJointAxis4",
        "/World/Robot/Joints/PrismaticJointAxis2",
        "/World/Robot/Joints/RevoluteJointAxis1",
        position1=(0.0, 2.25, 1.87),
        scale1=(0.6, 0.3, 0.1),
        color1=(0.2, 0.5, 0.7),  # gripper
        position2=(0.0, 0, 1.5),
        scale2=(0.8, 0.5, 3),
        color2=(0.7, 0.3, 0.5),  # tower
        position3=(0.0, 1.25, 2),
        scale3=(0.15, 2, 0.15),
        color3=(0.2, 0.5, 0.3),  # snake
    )

    create_pick_box(
        "/World/Environment/pickBox",
        position=(0, 2.3, 0.3),
        scale=(1, 1, 0.5),
        color=(2, 2, 2),
    )  # pick-box

    print("Scene setup complete.")
