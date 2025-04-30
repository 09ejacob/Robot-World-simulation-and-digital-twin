import numpy as np
from os.path import dirname, abspath, join
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, Sdf, PhysxSchema, UsdLux
from isaacsim.core.prims import SingleXFormPrim
from .camera import register_existing_camera


from ..global_variables import (
    FIXED_JOINT_BASE_GROUND,
    GROUND_PLANE_PATH,
    PHYSICS_SCENE_PATH,
    ROBOT_BASE_CUBE_PATH,
    BASE_CAMERA_PATH,
    ROBOT_PATH,
    BOX_CAMERA_1,
    BOX_CAMERA_2,
    OVERVIEW_CAMERA,
    SPHERE_LIGHT,
)


def create_ground_plane(path):
    """Create a ground plane at a given prim path."""
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))


def create_joint(
    joint_prim_path,
    object1_path,
    object2_path,
    joint_type,
    hinge_axis,
):
    """
    Create and configure a joint between two given objects.

    Args:
        joint_prim_path (str): the prim path for the new joint prim.
        object1_path (str): the prim path of the first object to create the joint between.
        object2_path (str): the prim path of the second object to create the joint between.
        joint_type (str): USD joint type.
        hinge_axis (tuple or None): Axis vector for hinge joints. Can also be None.
    """
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
    """Create any extra joints required by the robot setup."""

    # Base and groundplane joint
    create_joint(
        FIXED_JOINT_BASE_GROUND,
        GROUND_PLANE_PATH,
        ROBOT_BASE_CUBE_PATH,
        "PhysicsFixedJoint",
        None,
    )


def create_camera(resolutions=None):
    """Register existing cameras for image capture or streaming."""

    # If resolutions is None, initialize with empty dictionary
    if resolutions is None:
        resolutions = {}

    # Register cameras with optional resolution changes
    register_existing_camera(
        BASE_CAMERA_PATH, resolution=resolutions.get(BASE_CAMERA_PATH)
    )
    register_existing_camera(BOX_CAMERA_1, resolution=resolutions.get(BOX_CAMERA_1))
    register_existing_camera(BOX_CAMERA_2, resolution=resolutions.get(BOX_CAMERA_2))
    register_existing_camera(
        OVERVIEW_CAMERA, resolution=resolutions.get(OVERVIEW_CAMERA)
    )


def load_grab_usd(grab_usd):
    """Load the grab USD model into the stage."""
    current_dir = dirname(abspath(__file__))
    usd_path = abspath(
        join(
            current_dir,
            "..",
            "..",
            "Grab_Digital_Twin_python",
            "usd",
            grab_usd,
        )
    )
    add_reference_to_stage(usd_path=usd_path, prim_path=ROBOT_PATH)


def _add_light():
    """Add sphere light into the scene."""
    sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path(SPHERE_LIGHT))
    sphereLight.CreateRadiusAttr(2)
    sphereLight.CreateIntensityAttr(100000)
    SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])


def setup_scene(enable_cameras=False, grab_usd="Grab.usd"):
    """
    Setup the stage with physics scene, ground plane, robot model, optional cameras, joints, and lighting.

    Args:
        enable_cameras (bool): if True, register cameras for camera capture.
        grab_usd (str): USD file to reference for the robot.
    """
    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print(f"[setup_scene] Defining physics scene at: {PHYSICS_SCENE_PATH}")
        UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    physics_scene_prim = stage.GetPrimAtPath(PHYSICS_SCENE_PATH)

    if not physics_scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
        print("[setup_scene] Applied PhysxSceneAPI to PhysicsScene prim")
    else:
        print("[setup_scene] PhysxSceneAPI already present")

    create_ground_plane(GROUND_PLANE_PATH)

    load_grab_usd(grab_usd)

    if enable_cameras:
        custom_resolutions = {  # 720x480 is safe max for UDP transmission
            BASE_CAMERA_PATH: (720, 480),
            BOX_CAMERA_1: (720, 480),
            BOX_CAMERA_2: (720, 480),
            OVERVIEW_CAMERA: (1280, 820),  # Not sent over UDP
        }
        create_camera(custom_resolutions)

    create_additional_joints()
    _add_light()
