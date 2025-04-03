import numpy as np
from os.path import dirname, abspath, join
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.stage import get_current_stage
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdPhysics, Sdf, PhysxSchema, UsdLux
from .camera import setup_camera
from isaacsim.core.prims import SingleXFormPrim
from .camera import register_existing_camera
from ..camera_capture import CameraCapture


from ..global_variables import (
    FIXED_JOINT_BASE_GROUND,
    GROUND_PLANE_PATH,
    PHYSICS_SCENE_PATH,
    ROBOT_BASE_CUBE_PATH,
    BASE_CAMERA_PATH,
    ROBOT_PATH,
    BOX_CAMERA_1,
    BOX_CAMERA_2,
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


# TODO: Remove duplicate function
def create_camera2():
    # TODO: Create the camera in the USD file instead
    setup_camera(
        prim_path="/World/TestCamera",
        position=np.array([0, 0, 2]),
        euler_angles=np.array([0, 0, 0]),
        resolution=(1920, 1080),
        focal_length=20,
        clipping_range=(1, 10000),
        horizontal_aperture=20,
    )

# def create_camera(resolutions=None): 
#     # If resolutions is None, initialize with empty dictionary
#     if resolutions is None:
#         resolutions = {}
    
#     # Register cameras with optional resolution changes
#     register_existing_camera(BASE_CAMERA_PATH, 
#                             resolution=resolutions.get(BASE_CAMERA_PATH))
#     register_existing_camera(BOX_CAMERA_1, 
#                             resolution=resolutions.get(BOX_CAMERA_1))
#     register_existing_camera(BOX_CAMERA_2, 
#                             resolution=resolutions.get(BOX_CAMERA_2))

def create_camera():
    register_existing_camera(BASE_CAMERA_PATH)


def load_grab_usd():
    # Isaac Sim needs the absolute path
    current_dir = dirname(abspath(__file__))
    usd_path = abspath(
        join(current_dir, "..", "..", "Grab_Digital_Twin_python", "usd", "Grab.usd")
    )

    add_reference_to_stage(usd_path=usd_path, prim_path=ROBOT_PATH)


def _add_light():
    sphereLight = UsdLux.SphereLight.Define(
        get_current_stage(), Sdf.Path("/World/SphereLight")
    )
    sphereLight.CreateRadiusAttr(2)
    sphereLight.CreateIntensityAttr(100000)
    SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])


def setup_scene():
    stage = get_current_stage()

    # Define PhysicsScene if it doesn't exist
    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print(f"[setup_scene] Defining physics scene at: {PHYSICS_SCENE_PATH}")
        UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    # Apply PhysxSceneAPI to the PhysicsScene prim
    physics_scene_prim = stage.GetPrimAtPath(PHYSICS_SCENE_PATH)
    if not physics_scene_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        PhysxSchema.PhysxSceneAPI.Apply(physics_scene_prim)
        print("[setup_scene] Applied PhysxSceneAPI to PhysicsScene prim")
    else:
        print("[setup_scene] PhysxSceneAPI already present")

    create_ground_plane(GROUND_PLANE_PATH)
    
    load_grab_usd()

    #create_camera()
    #custom_resolutions = {
    #BOX_CAMERA_1: (1280, 720),
    #}
    #create_camera(custom_resolutions)
    create_additional_joints()
    _add_light()
