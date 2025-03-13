import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera
import omni.usd
from pxr import UsdPhysics, PhysxSchema

from .global_variables import CAMERA_PATH

def setup_camera(
    prim_path="",
    position=np.array([0, 0, 0]),  
    euler_angles=np.array([0, 0, 0]),
    resolution=(1920, 1080),
    focal_length=35,  # Default focal length
    horizontal_aperture=20.955  # Default horizontal aperture (Omniverse default)
):
    stage = omni.usd.get_context().get_stage()
    camera_Xform_prim = stage.GetPrimAtPath(CAMERA_PATH)

    print(f"Initializing camera at {prim_path}")
    print(f"Original Euler angles (degrees): {euler_angles}")

    quat_xyzw = rot_utils.euler_angles_to_quats(euler_angles, extrinsic=True, degrees=True)
    print(f"Quaternion in (xyzw) format: {quat_xyzw}")

    camera = Camera(
        prim_path=prim_path,
        resolution=resolution, 
        position=position,
        orientation=np.array([1, 0, 0, 0]),
    )

    camera.set_world_pose(position, quat_xyzw, camera_axes="usd")

    # Apply physics properties
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(camera_Xform_prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(camera_Xform_prim)

    # Disable gravity for the camera
    attr = camera_Xform_prim.GetAttribute("physxRigidBody:disableGravity")

    camera.set_focal_length(focal_length)
    print(f"Camera successfully created with orientation: {quat_xyzw}")
    print(f"Set focal length: {focal_length} mm, Horizontal Aperture: {horizontal_aperture} mm")
    
    return camera
