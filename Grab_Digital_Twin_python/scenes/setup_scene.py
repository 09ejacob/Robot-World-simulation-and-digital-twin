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
    
def create_camera(resolutions=None): 
    # If resolutions is None, initialize with empty dictionary
    if resolutions is None:
        resolutions = {}
    
    # Register cameras with optional resolution changes
    register_existing_camera(BASE_CAMERA_PATH, 
                            resolutions.get(BASE_CAMERA_PATH))
    register_existing_camera(BOX_CAMERA_1, 
                            resolutions.get(BOX_CAMERA_1))
    register_existing_camera(BOX_CAMERA_2, 
                            resolutions.get(BOX_CAMERA_2))


def add_sensor_asset(sensor_type: str, parent_prim_path: str = "/World", sensor_name: str = "sensor") -> str:
    """
    Adds a sensor camera asset to the stage.
    
    Args:
        sensor_type: Type of sensor (e.g. 'hawk_stereo', 'realsense', 'gemini2')
        parent_prim_path: Path where the sensor will be parented
        sensor_name: Name for the new sensor prim
        
    Returns:
        Path to the created sensor prim
    """
    sensor_paths = {
        'hawk_stereo': "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd",
    }
    
    if sensor_type not in sensor_paths:
        raise ValueError(f"Unknown sensor type: {sensor_type}. Valid options: {list(sensor_paths.keys())}")
    
    full_prim_path = f"{parent_prim_path}/{sensor_name}"
    add_reference_to_stage(
        usd_path=sensor_paths[sensor_type],
        prim_path=full_prim_path
    )
    return full_prim_path

def addstereo_camera():
    register_stereo_pair(
        left_prim_path=BOX_CAMERA_2,
        right_prim_path=BOX_CAMERA_1,
        pair_id="stereo_pair",
    )

def load_grab_usd(grab_usd):
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
    sphereLight = UsdLux.SphereLight.Define(
        get_current_stage(), Sdf.Path("/World/SphereLight")
    )
    sphereLight.CreateRadiusAttr(2)
    sphereLight.CreateIntensityAttr(100000)
    SingleXFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

def setup_scene(enable_cameras=False, grab_usd="Grab.usd"):
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
    
