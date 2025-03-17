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
    focal_length=35,
    clipping_range=(1,10000), 
    horizontal_aperture= 20, 

):
    stage = omni.usd.get_context().get_stage()
    camera_Xform_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_Xform_prim:
        print(f"Camera Xform not found at {CAMERA_PATH}")
        return None
    print(f"Initializing camera at {prim_path}")
    print(f"Original Euler angles (degrees): {euler_angles}")

    quat_xyzw = rot_utils.euler_angles_to_quats(euler_angles, extrinsic=True, degrees=True)
    print(f"Quaternion in (xyzw) format: {quat_xyzw}")

    camera = Camera(
        prim_path=prim_path,
        resolution=resolution, 
        position=position,
        orientation=quat_xyzw,
    )
  # Initialize the camera to ensure product_render_path is set
    camera.initialize()

    camera.set_world_pose(position, quat_xyzw, camera_axes="usd")

    # Apply physics properties
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(camera_Xform_prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(camera_Xform_prim)

    # Disable gravity for the camera
    attr = camera_Xform_prim.GetAttribute("physxRigidBody:disableGravity")

    camera.set_focal_length(focal_length/10)
    camera.set_clipping_range(clipping_range[0], clipping_range[1])
    camera.set_horizontal_aperture(horizontal_aperture/10) 
    print(f"Camera successfully created with orientation: {quat_xyzw}")
    
    return camera
