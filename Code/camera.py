from omni.isaac.sensor import Camera
from omni.isaac.core.simulation_context import SimulationContext


def setup_camera(
    prim_path="/World/Camera",
    position=[0, 0, 1],
    orientation=[0, 0, 0, 1],
    resolution=(1920, 1080),
):
    """
    Sets up a camera in the simulation world.

    Args:
        prim_path (str): Path in the USD hierarchy to place the camera.
        position (list): Position of the camera [x, y, z].
        orientation (list): Quaternion representing camera orientation [x, y, z, w].
        resolution (tuple): Resolution of the camera in (width, height).

    Returns:
        Camera: The Camera object added to the simulation.
    """
    camera = Camera(
        prim_path=prim_path,
        position=position,
        orientation=orientation,
        resolution=resolution,
    )

    # Add the camera to the simulation scene
    SimulationContext().scene.add(camera)
    print(f"Camera added at {prim_path} with resolution {resolution}.")
    return camera
