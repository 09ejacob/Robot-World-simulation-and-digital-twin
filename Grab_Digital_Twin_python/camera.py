import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera
import omni.usd
from pxr import UsdPhysics, PhysxSchema
from .global_variables import (
    CAMERA_PATH,
)

def setup_camera(
    prim_path="",
    position=np.array([0, 0, 0]), # I think the reason we need to divide pos by 2 is that camera Xform is set to half the original size
    euler_angles=np.array([0, 0, 0]),
    resolution=(1920, 1080),

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

    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(camera_Xform_prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(camera_Xform_prim)

    attr = camera_Xform_prim.GetAttribute("physxRigidBody:disableGravity")

    print(f"Camera successfully created with orientation: {quat_xyzw}")
    return camera
