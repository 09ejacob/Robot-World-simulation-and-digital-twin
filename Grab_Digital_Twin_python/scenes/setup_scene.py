import numpy as np
from os.path import dirname, abspath, join
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, Sdf

from ..global_variables import (
    FIXED_JOINT_BASE_GROUND,
    GROUND_PLANE_PATH,
    PHYSICS_SCENE_PATH,
    ROBOT_BASE_CUBE_PATH,
)


def create_ground_plane(path):
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))


def create_joint(
    joint_prim_path,
    object1_path,
    object2_path,
    joint_type,
    hinge_axis,
):
    stage = get_current_stage()

    if hinge_axis is not None:
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


def create_additional_joints():
    # Base and groundplane joint
    create_joint(
        FIXED_JOINT_BASE_GROUND,
        GROUND_PLANE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )


def load_grab_usd():
    # Isaac Sim needs the absolute path
    current_dir = dirname(abspath(__file__))
    usd_path = abspath(
        join(current_dir, "..", "..", "Grab_Digital_Twin_python", "usd", "Grab.usd")
    )

    add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")


def setup_scene():
    stage = get_current_stage()
    UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    create_ground_plane(GROUND_PLANE_PATH)

    load_grab_usd()

    create_additional_joints()
