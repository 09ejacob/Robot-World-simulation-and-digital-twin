import numpy as np
from os.path import dirname, abspath, join
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, Sdf, PhysxSchema, UsdLux
from isaacsim.core.prims import SingleXFormPrim
from .camera import register_existing_camera
from .camera import register_stereo_pair


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
    CAMERA_RESOLUTIONS,
    DEFAULT_STEREO_PAIR_ID,
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


def create_camera(
    resolutions=None, enable_3d_features=False, enable_overview_camera=False
):
    """
    Create cameras in the scene and register them with the camera capture system.
    Args:
        resolutions (dict): Dictionary of camera prim paths and their respective resolutions.
        enable_3d_features (bool): If True, enable 3D features for the cameras.
        enable_overview_camera (bool); If True, enable the overview camera.
    """

    # If resolutions is None, initialize with empty dictionary
    if resolutions is None:
        resolutions = {}

    # Register cameras with optional resolution changes and 3D features toggle
    register_existing_camera(BASE_CAMERA_PATH, resolutions.get(BASE_CAMERA_PATH))

    register_existing_camera(
        BOX_CAMERA_1, resolutions.get(BOX_CAMERA_1), add_3d_features=enable_3d_features
    )
    register_existing_camera(
        BOX_CAMERA_2, resolutions.get(BOX_CAMERA_2), add_3d_features=enable_3d_features
    )
    setup_stereo_cameras()

    if enable_overview_camera:
        register_existing_camera(
            OVERVIEW_CAMERA,
            resolutions.get(OVERVIEW_CAMERA),
        )


def setup_stereo_cameras():
    """Setup stereo camera configuration using existing box cameras."""
    # Register the stereo pair
    stereo_pair = register_stereo_pair(
        left_prim_path=BOX_CAMERA_2,
        right_prim_path=BOX_CAMERA_1,
        pair_id=DEFAULT_STEREO_PAIR_ID,
    )
    return stereo_pair


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


def setup_scene(
    grab_usd="Grab.usd",
    enable_cameras=False,
    enable_3d_features=False,
    enable_overview_camera=False,
):
    """
    Setup the stage with physics scene, ground plane, robot model, optional cameras, joints, and lighting.

    Args:
        grab_usd (str): USD file to reference for the robot.
        enable_cameras (bool): if True, register cameras for camera capture.
        enable_3d_features (bool): If True, adds depth and point cloud features to the cameras.
        enable_overview_camera (bool): If True, enables the overview camera.
    """
    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print(f"[MAIN] Defining physics scene at: {PHYSICS_SCENE_PATH}")
        UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    physics_scene_prim = stage.GetPrimAtPath(PHYSICS_SCENE_PATH)

    if not physics_scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
        print("[MAIN] Applied PhysxSceneAPI to PhysicsScene prim")
    else:
        print("[DEBUG] PhysxSceneAPI already present")

    create_ground_plane(GROUND_PLANE_PATH)

    load_grab_usd(grab_usd)

    if enable_cameras:
        create_camera(CAMERA_RESOLUTIONS, enable_3d_features, enable_overview_camera)

    create_additional_joints()
    _add_light()
