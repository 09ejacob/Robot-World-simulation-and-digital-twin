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
from isaacsim.sensors.camera import Camera
from .camera import setup_camera
import asyncio
import omni.kit.app

from .global_variables import (
    JOINTS_PATH,
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS2_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    FIXED_JOINT_BASE_GROUND,
    PRISMATIC_JOINT_FORCE_SENSOR,
    ROBOT_BASE_JOINT_PATH,
    AXIS2_TOWER_JOINT_PATH,
    PALLET_BASE_JOINT_PATH,
    CABINET_BASE_JOINT_PATH,
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
    AXIS2_BASE_PATH,
    TOWER_PATH,
    ROBOT_BASE_PATH,
    AXIS2_TOWER_PATH,
    PALLET_BASE_PATH,
    CABINET_PATH,
    CAMERA_SENSOR_PATH,
    BOXCAMERA_PATH,
    CAMERA_SNAKE_JOINT_PATH,
)


def create_ground_plane(path):
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))


def create_xform(path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xform.Define(stage, path)

    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def enable_linear_drive(
    joint_prim_path, stiffness=10.0, damping=5.0, max_force=30.0, target_position=0.0
):
    stage = get_current_stage()
    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    if not joint_prim.IsValid():
        print(f"Joint prim at path {joint_prim_path} is not valid.")
        return

    if joint_prim.GetTypeName() != "PhysicsPrismaticJoint":
        print(f"Joint at path {joint_prim_path} is not a PhysicsPrismaticJoint.")
        return

    UsdPhysics.DriveAPI.Apply(joint_prim, "linear")

    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
    drive_api.GetStiffnessAttr().Set(stiffness)
    drive_api.GetDampingAttr().Set(damping)
    drive_api.GetMaxForceAttr().Set(max_force)
    drive_api.GetTargetPositionAttr().Set(target_position)


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

    if joint_prim.GetTypeName() != "PhysicsRevoluteJoint":
        print(f"Joint at path {joint_prim_path} is not a PhysicsRevoluteJoint.")
        return

    UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    drive_api.GetStiffnessAttr().Set(stiffness)
    drive_api.GetDampingAttr().Set(damping)
    drive_api.GetMaxForceAttr().Set(max_force)

    drive_api.GetTargetPositionAttr().Set(target_position)


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


def create_base_robot_model(
    position1=(0, 0, 0),
    scale1=(1, 1, 1),
    color1=(0, 0, 0),
    position2=(0, 0, 0),
    scale2=(1, 1, 1),
    color2=(0, 0, 0),
    position3=(0, 0, 0),
    scale3=(1, 1, 1),
    color3=(0, 0, 0),
    position4=(0, 0, 0),
    scale4=(1, 1, 1),
    color4=(0, 0, 0),
    position5=(0, 0, 0),
    scale5=(1, 1, 1),
    color5=(0, 0, 0),
    position6=(0, 0, 0),
    scale6=(1, 1, 1),
    color6=(0, 0, 0),
    position7=(0, 0, 0),
    scale7=(1, 1, 1),
    color7=(0, 0, 0),
    position8=(0, 0, 0),
    scale8=(1, 1, 1),
    color8=(0, 0, 0),
    position9=(0, 0, 0),
    scale9=(1, 1, 1),
    color9=(0, 0, 0),
    position10=(0, 0, 0),
    scale10=(1, 1, 1),
    color10=(0, 0, 0),
):
    gripper = DynamicCuboid(
        prim_path=GRIPPER_PATH,
        position=np.array(position1),
        scale=np.array(scale1),
        color=np.array(color1),
    )

    axis2_base = DynamicCuboid(
        prim_path=AXIS2_BASE_PATH,
        position=np.array(position2),
        scale=np.array(scale2),
        color=np.array(color2),
    )

    snake = DynamicCuboid(
        prim_path=SNAKE_PATH,
        position=np.array(position3),
        scale=np.array(scale3),
        color=np.array(color3),
    )

    base = DynamicCuboid(
        prim_path=ROBOT_BASE_CUBE_PATH,
        position=np.array(position4),
        scale=np.array(scale4),
        color=np.array(color4),
    )

    snake_base = DynamicCuboid(
        prim_path=SNAKE_BASE_PATH,
        position=np.array(position5),
        scale=np.array(scale5),
        color=np.array(color5),
    )

    force_sensor = DynamicCuboid(
        prim_path=FORCE_SENSOR_PATH,
        position=np.array(position6),
        scale=np.array(scale6),
        color=np.array(color6),
    )

    robot_base = DynamicCylinder(
        prim_path=ROBOT_BASE_PATH,
        position=np.array(position7),
        scale=np.array(scale7),
        color=np.array(color7),
    )

    axis2_tower = DynamicCuboid(
        prim_path=AXIS2_TOWER_PATH,
        position=np.array(position8),
        scale=np.array(scale8),
        color=np.array(color8),
    )

    pallet_base = DynamicCuboid(
        prim_path=PALLET_BASE_PATH,
        position=np.array(position9),
        scale=np.array(scale9),
        color=np.array(color9),
    )

    cabinet = DynamicCuboid(
        prim_path=CABINET_PATH,
        position=np.array(position10),
        scale=np.array(scale10),
        color=np.array(color10),
    )

    # Make snake_base have no collision and no volume
    stage = get_current_stage()
    collisionAPI = UsdPhysics.CollisionAPI.Get(stage, SNAKE_BASE_PATH)
    collisionAPI.GetCollisionEnabledAttr().Set(False)

    stage = get_current_stage()
    snake_base_prim = stage.GetPrimAtPath(SNAKE_BASE_PATH)

    snake_base_prim.CreateAttribute("primvars:isVolume", Sdf.ValueTypeNames.Bool).Set(
        True
    )

    setup_camera()

    # Axis2_base no volume and collision
    collisionAPI = UsdPhysics.CollisionAPI.Get(stage, AXIS2_BASE_PATH)
    collisionAPI.GetCollisionEnabledAttr().Set(False)

    stage = get_current_stage()
    axis2_base = stage.GetPrimAtPath(AXIS2_BASE_PATH)

    axis2_base.CreateAttribute("primvars:isVolume", Sdf.ValueTypeNames.Bool).Set(True)


def create_joints():
    # Axis 4 joint
    create_joint(
        AXIS4_JOINT_PATH,
        FORCE_SENSOR_PATH,
        GRIPPER_PATH,
        "PhysicsRevoluteJoint",
        "Z",
    )
    enable_angular_drive(AXIS4_JOINT_PATH)

    # Axis 3 joint
    create_joint(
        AXIS3_JOINT_PATH,
        SNAKE_PATH,
        SNAKE_BASE_PATH,
        "PhysicsPrismaticJoint",
        "Y",
    )
    
    set_prismatic_joint_limits(AXIS3_JOINT_PATH, -2.0, 0)
    enable_linear_drive(
        AXIS3_JOINT_PATH, stiffness=100, damping=100, max_force=100, target_position=0.0
    )

 
    # Axis 1 joint
    create_joint(
        AXIS1_JOINT_PATH,
        AXIS2_BASE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsRevoluteJoint",
        "Z",
    )
    enable_angular_drive(AXIS1_JOINT_PATH)

    # Base and groundplane joint
    create_joint(
        FIXED_JOINT_BASE_GROUND,
        GROUND_PLANE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )

    # Axis 2 joint
    create_joint(
        AXIS2_JOINT_PATH,
        AXIS2_BASE_PATH,
        SNAKE_BASE_PATH,
        "PhysicsPrismaticJoint",
        "Z",
    )
    set_prismatic_joint_limits(AXIS2_JOINT_PATH, 0, 1.6)
    enable_linear_drive(
        AXIS2_JOINT_PATH,
        stiffness=1000,
        damping=500,
        max_force=100,
        target_position=0.0,
    )

    # Force sensor joint
    create_joint(
        PRISMATIC_JOINT_FORCE_SENSOR,
        FORCE_SENSOR_PATH,
        SNAKE_PATH,
        "PhysicsPrismaticJoint",
        "Z",
    )
    enable_linear_drive(
        PRISMATIC_JOINT_FORCE_SENSOR,
        stiffness=10000,
        damping=10000,
        max_force=100,
        target_position=0.0,
    )

    # Robot base joint
    create_joint(
        ROBOT_BASE_JOINT_PATH,
        SNAKE_BASE_PATH,
        ROBOT_BASE_PATH,
        "PhysicsRevoluteJoint",
        "Z",
    )

    # Axis2 tower joint
    create_joint(
        AXIS2_TOWER_JOINT_PATH,
        AXIS2_TOWER_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )

    # Pallet base joint
    create_joint(
        PALLET_BASE_JOINT_PATH,
        PALLET_BASE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )

    create_joint(
        CABINET_BASE_JOINT_PATH,
        CABINET_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )
 
    # Camera snake joint
    create_joint(
        CAMERA_SNAKE_JOINT_PATH,  
        SNAKE_BASE_PATH,
        CAMERA_SENSOR_PATH,
        "PhysicsFixedJoint",
        None,
    )



def apply_articulation_root(path):
    stage = get_current_stage()
    robot_prim = stage.GetPrimAtPath(path)

    if robot_prim and robot_prim.IsValid():
        UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    else:
        print(f"Could not find prim at {path}")


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


# Values
gripper_pos = (0.0, 0.05, 0.25)
gripper_scale = (0.25, 0.15, 0.05)
gripper_color = (0.05, 0.05, 0.05)

axis2_base_pos = (0.0, 0, 1.2)
axis2_base_scale = (0.3, 0.3, 2)
axis2_base_color = (0.05, 0.05, 0.05)

snake_pos = (0.0, 0.0, 0.37)
snake_scale = (0.13, 0.3, 0.13)
snake_color = (0.05, 0.05, 0.05)

base_pos = (0, 0, 0.1)
base_scale = (0.6, 1.1, 0.2)
base_color = (0.05, 0.05, 0.05)

snake_base_pos = (0, 0, 1)
snake_base_scale = (0.5, 4.0, 0.3)
snake_base_color = (0.05, 0.05, 0.05)

force_sensor_pos = (0.0, 0.05, 0.29)
force_sensor_scale = (0.1, 0.1, 0.02)
force_sensor_color = (0.2, 0.2, 0.2)

robot_base_pos = (0.0, 0.0, 0.5)
robot_base_scale = (0.35, 0.35, 0.1)
robot_base_color = (0.05, 0.05, 0.05)

axis2_tower_pos = (0, -0.45, 1.2)
axis2_tower_scale = (0.2, 0.2, 2)
axis2_tower_color = (0.05, 0.05, 0.05)

pallet_base_pos = (0.0, 1.25, 0.25)
pallet_base_scale = (1, 1.4, 0.5)
pallet_base_color = (0.05, 0.05, 0.05)

cabinet_pos = (0.0, -0.8, 0.7)
cabinet_scale = (1, 0.5, 1.4)
cabinet_color = (0.05, 0.05, 0.05)


def setup_scene():
    stage = get_current_stage()
    UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    create_ground_plane(GROUND_PLANE_PATH)

    create_xform(ROBOT_PATH, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))

    create_xform(TOWER_PATH, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))

    create_xform(BOXCAMERA_PATH, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))


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

    create_base_robot_model(
        gripper_pos,
        gripper_scale,
        gripper_color,
        axis2_base_pos,
        axis2_base_scale,
        axis2_base_color,
        snake_pos,
        snake_scale,
        snake_color,
        base_pos,
        base_scale,
        base_color,
        snake_base_pos,
        snake_base_scale,
        snake_base_color,
        force_sensor_pos,
        force_sensor_scale,
        force_sensor_color,
        robot_base_pos,
        robot_base_scale,
        robot_base_color,
        axis2_tower_pos,
        axis2_tower_scale,
        axis2_tower_color,
        pallet_base_pos,
        pallet_base_scale,
        pallet_base_color,
        cabinet_pos,
        cabinet_scale,
        cabinet_color,
    )

    create_joints()

    create_surface_gripper(
        GRIPPER_ACTION_GRAPH_PATH,
        GRIPPER_OFFSET_PATH,
        GRIPPER_PATH,
    )

    apply_articulation_root(ROBOT_PATH)

    create_xform(
        GRIPPER_OFFSET_PATH,
        translate=(0, 0, -0.2),
        rotation=(0, 0, 0),
        scale=(1, 1, 1),
    )
