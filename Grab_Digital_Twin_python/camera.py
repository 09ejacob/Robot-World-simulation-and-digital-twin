import logging
from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.core.utils.numpy.rotations as rot_utils
from scipy.spatial.transform import Rotation as R
import numpy as np

# Configure logging
logging.basicConfig(level=print, format="%(asctime)s - %(levelname)s - %(message)s")

def setup_camera(
    prim_path="/World/Robot/Tower/cameraSensor",
    position=[1, 2, 1],
    euler_orientation=[0, 0, 0],
    resolution=(1920, 1080),
):
    """
    Sets up a simple camera sensor in the simulation world.

    Args:
        prim_path (str): USD path where the camera sensor is placed.
        position (list): Position of the camera [x, y, z].
        euler_orientation (list): Euler angles in degrees [roll, pitch, yaw].
        resolution (tuple): Resolution of the camera in (width, height).

    Returns:
        Camera: The Camera object added to the simulation.
    """   
    print(f"Initializing camera at {prim_path}")
    print(f"Position: {position}, Euler Orientation: {euler_orientation}, Resolution: {resolution}")

    # Convert Euler angles to quaternion
    quaternion = rot_utils.euler_angles_to_quats(
        np.array(euler_orientation),
        degrees=True
    )
    
    print(f"Converted Euler angles {euler_orientation} to Quaternion {quaternion}")

    # Alternative method for debugging quaternion conversion
    scipy_quat = R.from_euler('yxz', euler_orientation, degrees=True).as_quat()
    print(f"Scipy Quaternion for comparison: {scipy_quat}")
    

    # Create the Camera sensor
    try:
        camera = Camera(prim_path=prim_path, resolution=resolution, position=position, orientation=quaternion)
        print(f"Camera successfully created at {prim_path}")
    except Exception as e:
        logging.error(f"Failed to create Camera at {prim_path}: {e}")
        return None

    # Verify if camera is valid
    if camera is None:
        print("Camera object is None, something went wrong during creation.")
    else:
        print(f"Camera added at {prim_path} with resolution {resolution}, position {position}, and orientation {quaternion}")

    return camera
