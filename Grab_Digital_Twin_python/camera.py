from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.utils.rotations import euler_angles_to_quat
import numpy as np
def setup_camera(
    prim_path="/World/Robot/Tower/Axis2/gripper/cameraSensor",
    position=[1, 2, 1],
    orientation=[0, 90, 0, 0],
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

    # Create the Camera sensor
    camera = Camera(prim_path=prim_path, resolution=resolution, position=position, orientation=orientation)

    print(f"Camera added at {prim_path} with resolution {resolution}, position {position}, and orientation {orientation}")
    return camera
