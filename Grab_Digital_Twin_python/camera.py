import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera
from .global_variables import (
    CAMERA_SENSOR_PATH
)

def setup_camera(
    prim_path=CAMERA_SENSOR_PATH,
    position=np.array([0, 1, 1]),
    euler_angles=np.array([70, 0, 0]),  # Change Euler angles as needed
    resolution=(1920, 1080),

):
    """
    Sets up a camera using Euler angles converted to quaternion (w, x, y, z) for Isaac Sim,
    ensuring no automatic rotation using `camera_axes="usd"`.
    """

    print(f"Initializing camera at {prim_path}")
    print(f"Original Euler angles (degrees): {euler_angles}")

    # Convert Euler angles to quaternion (xyzw)
    quat_xyzw = rot_utils.euler_angles_to_quats(euler_angles, extrinsic=True, degrees=True)
    print(f"Quaternion in (xyzw) format: {quat_xyzw}")


    # Create camera (initial orientation doesn't matter since we'll override it)
    camera = Camera(
        prim_path=prim_path, 
        resolution=resolution, 
        position=position,
        orientation=np.array([1, 0, 0, 0])  # Temporary identity quaternion
    )

    # Set correct pose with `camera_axes="usd"` to disable auto-rotation
    camera.set_world_pose(position, quat_xyzw, camera_axes="usd")

    print(f"Camera successfully created with orientation: {quat_xyzw}")
    return camera
