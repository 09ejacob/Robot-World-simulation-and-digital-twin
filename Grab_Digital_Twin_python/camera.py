import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera

def setup_camera(
    prim_path="/World/Robot/Tower/cameraSensor",
    position=np.array([1, 2, 1]),
    euler_angles=np.array([0, 90, 0]),  # Change Euler angles as needed
    resolution=(1920, 1080),
):
    """
    Sets up a camera using Euler angles converted to quaternion (w, x, y, z) for Isaac Sim.
    """

    print(f"Initializing camera at {prim_path}")
    print(f"Original Euler angles (degrees): {euler_angles}")

    # Convert Euler angles to quaternion (xyzw)
    quat_xyzw = rot_utils.euler_angles_to_quats(euler_angles, extrinsic=False, degrees=True)
    print(f"Quaternion in (xyzw) format: {quat_xyzw}")

    # Convert to (wxyz) format for Isaac Sim
    quat_wxyz = np.roll(quat_xyzw, shift=1)
    print(f"Quaternion in (wxyz) format for Isaac Sim: {quat_wxyz}")

    # Create camera with correct quaternion format
    camera = Camera(
        prim_path=prim_path, 
        resolution=resolution, 
        position=position,
        orientation=quat_wxyz  # Use (w, x, y, z) format
    )
    
    print(f"Camera successfully created with orientation: {quat_wxyz}")
    return camera
