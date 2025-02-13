import numpy as np
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import DynamicCylinder
from omni.isaac.core.objects.ground_plane import GroundPlane
from isaacsim.robot.surface_gripper import SurfaceGripper
import omni.usd
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
import omni.graph.core as og
from pxr import Usd, UsdGeom
from .camera import setup_camera
import asyncio
import omni.kit.app
import numpy as np

from .global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS2_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    FIXED_JOINT_BASE_GROUND,
    FIXED_JOINT_FORCE_SENSOR,
    FORCE_SENSOR_PATH,
    GRIPPER_ACTION_GRAPH_PATH,
    GRIPPER_OFFSET_PATH,
    GRIPPER_PATH,
    GROUND_PLANE_PATH,
    PHYSICS_SCENE_PATH,
    ROBOT_BASE_CUBE_PATH,
    ROBOT_BASE_GROUP_PATH,
    ROBOT_PATH,
    SNAKE_BASE_PATH,
    SNAKE_PATH,
    TOWER_CUBOID_PATH,
    TOWER_PATH,
)


def create_ground_plane(path):
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))
    # world.scene.add_default_ground_plane()
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

    if hinge_axis != None:
        create_prim(
            prim_path=joint_prim_path,
            prim_type=joint_type,
            attributes={"physics:axis": hinge_axis},
        )
    else:
        create_prim(
            prim_path=joint_prim_path,
            prim_type=joint_type,
        )

    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    joint_prim.GetRelationship("physics:body0").SetTargets([Sdf.Path(object1_path)])
    joint_prim.GetRelationship("physics:body1").SetTargets([Sdf.Path(object2_path)])


def setup_robot(
    prim_path1,
    prim_path2,
    prim_path3,
    prim_path4,
    prim_path5,
    joint_prim_path1,
    joint_prim_path2,
    joint_prim_path3,
    joint_prim_path4,
    joint_prim_path5,
    position1=(0, 0, 0),
    scale1=(1, 1, 1),
    color1=(0, 0, 0),  # gripper
    position2=(0, 0, 0),
    scale2=(1, 1, 1),
    color2=(0, 0, 0),  # tower
    position3=(0, 0, 0),
    scale3=(1, 1, 1),
    color3=(0, 0, 0),
    position4=(0, 0, 0),
    scale4=(1, 1, 1),
    color4=(0, 0, 0),
    position5=(0, 0, 0),
    scale5=(1, 1, 1),
    color5=(0, 0, 0),
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

    base = DynamicCuboid(
        prim_path=prim_path4,
        position=np.array(position4),
        scale=np.array(scale4),
        color=np.array(color4),
    )

    snake_base = DynamicCuboid(
        prim_path=prim_path5,
        position=np.array(position5),
        scale=np.array(scale5),
        color=np.array(color5),
    )
    print("Created snake base")

    stage = get_current_stage()
    collisionAPI = UsdPhysics.CollisionAPI.Get(stage, prim_path5)
    collisionAPI.GetCollisionEnabledAttr().Set(False)

    stage = get_current_stage()
    snake_base_prim = stage.GetPrimAtPath(prim_path5)
    # Create the primvar if it doesn't exist, then set it to True.
    snake_base_prim.CreateAttribute("primvars:isVolume", Sdf.ValueTypeNames.Bool).Set(
        True
    )

    create_force_sensor(FORCE_SENSOR_PATH, sensor_offset=(0.0, 2.25, 2.39))

    setup_camera(
        prim_path="/World/Robot/Tower/Axis2/gripper/cameraSensor",
        position=[1, 2, 1],
        orientation=[100, 0, 1],
        resolution=(1920, 1080),
    )

    # Joints
    create_joint(
        joint_prim_path1,
        FORCE_SENSOR_PATH,
        GRIPPER_PATH,
        "PhysicsRevoluteJoint",
        "Z",
    )  # Axis4 joint
    enable_angular_drive(joint_prim_path1)

    create_joint(
        joint_prim_path2,
        SNAKE_PATH,
        SNAKE_BASE_PATH,
        "PhysicsPrismaticJoint",
        "Y",
    )  # In out joint
    set_prismatic_joint_limits(joint_prim_path2, -1.0, 1.5)
    enable_linear_drive(
        joint_prim_path2, stiffness=100, damping=100, max_force=100, target_position=0.0
    )

    create_joint(
        joint_prim_path3,
        TOWER_CUBOID_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsRevoluteJoint",
        "Z",
    )  # Axis1 joint
    enable_angular_drive(joint_prim_path3)

    create_joint(
        joint_prim_path4,
        GROUND_PLANE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )  # Base and groundplane joint

    create_joint(
        joint_prim_path5,
        TOWER_CUBOID_PATH,
        SNAKE_BASE_PATH,
        "PhysicsPrismaticJoint",
        "Z",
    )  # Axis2 joint
    set_prismatic_joint_limits(joint_prim_path5, -1.5, 0.8)
    enable_linear_drive(
        joint_prim_path5,
        stiffness=1000,
        damping=100,
        max_force=100,
        target_position=0.0,
    )

    create_surface_gripper(
        GRIPPER_ACTION_GRAPH_PATH,
        GRIPPER_OFFSET_PATH,
        GRIPPER_PATH,
    )


def create_force_sensor(sensor_prim_path, sensor_offset=(0.0, 0.0, 0.0)):
    force_sensor = DynamicCuboid(
        prim_path=sensor_prim_path,
        position=np.array(sensor_offset),
        scale=np.array([0.2, 0.2, 0.05]),
        color=np.array([1.0, 0.0, 0.0]),
    )

    create_joint(
        FIXED_JOINT_FORCE_SENSOR,
        FORCE_SENSOR_PATH,
        SNAKE_PATH,
        "PhysicsFixedJoint",
        None,
    )


def create_surface_gripper(graph_path, grip_position_path, parent_rigidBody_path):
    keys = og.Controller.Keys
    (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("surface_gripper", "isaacsim.robot.surface_gripper.SurfaceGripper"),
                ("close", "omni.graph.action.OnImpulseEvent"),
                ("impulse_monitor", "omni.graph.action.OnImpulseEvent"),
                ("open", "omni.graph.action.OnImpulseEvent"),
            ],
            keys.SET_VALUES: [
                ("surface_gripper.inputs:GripPosition", grip_position_path),
                ("surface_gripper.inputs:ParentRigidBody", parent_rigidBody_path),
            ],
            keys.CONNECT: [
                ("impulse_monitor.outputs:execOut", "surface_gripper.inputs:onStep"),
                ("open.outputs:execOut", "surface_gripper.inputs:Open"),  # open
                ("close.outputs:execOut", "surface_gripper.inputs:Close"),  # close
            ],
        },
    )
    print("Created surface gripper action graph")


def setup_scene():
    stage = get_current_stage()
    UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    print("Setting up scene...")

    create_ground_plane(GROUND_PLANE_PATH)

    create_xform(ROBOT_PATH, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))
    create_xform(TOWER_PATH, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))
    create_xform(
        AXIS2_PATH,
        translate=(0, 0, 0),
        rotation=(0, 0, 0),
        scale=(1, 1, 1),
    )

    create_xform(
        ROBOT_BASE_GROUP_PATH,
        translate=(0, 0, 0),
        rotation=(0, 0, 0),
        scale=(1, 1, 1),
    )

    setup_robot(
        GRIPPER_PATH,
        TOWER_CUBOID_PATH,
        SNAKE_PATH,
        ROBOT_BASE_CUBE_PATH,
        SNAKE_BASE_PATH,
        AXIS4_JOINT_PATH,
        AXIS3_JOINT_PATH,
        AXIS1_JOINT_PATH,
        FIXED_JOINT_BASE_GROUND,
        AXIS2_JOINT_PATH,
        position1=(0.0, 2.25, 2.3),
        scale1=(0.6, 0.3, 0.1),
        color1=(0.2, 0.5, 0.7),
        position2=(0.0, 0, 2),
        scale2=(0.8, 0.5, 3),
        color2=(0.7, 0.3, 0.5),
        position3=(0.0, 2.2, 2.5),
        scale3=(0.15, 0.4, 0.15),
        color3=(0.2, 0.5, 0.3),
        position4=(0, 0, 0.25),
        scale4=(2, 6, 0.5),
        color4=(0.6, 0.2, 0.2),
        position5=(0, 0, 2.5),
        scale5=(0.5, 7, 0.3),
        color5=(0.1, 0.2, 0.2),
    )

    create_xform(
        GRIPPER_OFFSET_PATH,
        translate=(0, 0, -0.500997),
        rotation=(0, 0, 0),
    scale=(1, 1, 1) 
)


