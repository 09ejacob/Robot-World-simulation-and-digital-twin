from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, Gf


def setup_camera(
    prim_path="/World/Robot/Tower/Axis2/gripper/cameraSensor",
    position=[0, 0, 1],
    orientation=[0, 0, 0, 1],
    resolution=(1920, 1080),
):
    """
    Sets up a simple camera sensor in the simulation world.

    Args:
        prim_path (str): USD path where the camera sensor is placed.
        position (list): Position of the camera [x, y, z].
        orientation (list): Quaternion representing camera orientation [x, y, z, w].
        resolution (tuple): Resolution of the camera in (width, height).

    Returns:
        Camera: The Camera object added to the simulation.
    """
    # Create the Camera sensor at the given prim path with the specified resolution.
    camera = Camera(prim_path=prim_path, resolution=resolution)
    
    # Set the camera's position and orientation using the Camera API.
    camera.set_position(position)
    camera.set_orientation(orientation)
    
    print(f"Camera added at {prim_path} with resolution {resolution}.")
    return camera
