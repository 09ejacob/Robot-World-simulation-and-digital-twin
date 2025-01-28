import time
import numpy as np

# Core Isaac Sim / Omniverse imports
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage

# USD and PhysX imports
from pxr import UsdGeom, UsdPhysics, Gf, Sdf


def create_revolute_joint_example():
    # 1. Create a World (manages scene & simulation stepping)

    # 2. Create a parent Xform, mark it as the articulation root
    robot_xform_path = "/World/Robot"
    stage = get_current_stage()
    robot_xform = UsdGeom.Xform.Define(stage, robot_xform_path)
    # Apply an ArticulationRootAPI to the Xform's prim
    UsdPhysics.ArticulationRootAPI.Apply(robot_xform.GetPrim())

    # 3. Add two dynamic cubes under "/World/Robot"
    cube_a = DynamicCuboid(
        prim_path=f"{robot_xform_path}/CubeA",
        name="cubeA",
        position=np.array([0.0, 0.0, 0.0]),
        size=0.1,  # or scale=(0.1, 0.1, 0.1)
    )
    cube_b = DynamicCuboid(
        prim_path=f"{robot_xform_path}/CubeB",
        name="cubeB",
        position=np.array([0.3, 0.0, 0.0]),
        size=0.1,
    )

    # 4. Create a revolute joint (hinge) connecting CubeA and CubeB
    joint_prim_path = f"{robot_xform_path}/RevoluteJoint"
    create_prim(
        prim_path=joint_prim_path,
        prim_type="PhysicsRevoluteJoint",
        attributes={
            "physics:axis": "Z"  # e.g., hinge around the Z axis
        },
    )
    joint_prim = stage.GetPrimAtPath(joint_prim_path)

    # 5. Connect the joint's body0 and body1 relationships to the two cubes
    joint_prim.GetRelationship("physics:body0").SetTargets(
        [Sdf.Path(f"{robot_xform_path}/CubeA")]
    )
    joint_prim.GetRelationship("physics:body1").SetTargets(
        [Sdf.Path(f"{robot_xform_path}/CubeB")]
    )


create_revolute_joint_example()
