import carb
import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
import omni.usd
from isaacsim.sensors.camera import Camera
from pxr import UsdGeom, UsdPhysics, PhysxSchema
from ..camera_capture import CameraCapture
from ..global_variables import CAMERA_PATH

initialized_cameras = {}
camera_capture = CameraCapture()  # Global CameraCapture instance


def setup_camera(
    prim_path="",
    position=np.array([0, 0, 0]),
    euler_angles=np.array([0, 0, 0]),
    resolution=(1920, 1080),
    focal_length=35,
    clipping_range=(1, 10000),
    horizontal_aperture=20,
):
    """
    Setup a camera in the scene with the specified parameters.
    This function initializes the camera and registers it with the camera capture system.
    It also applies physics properties to the camera and sets its render product.
    Args:
        prim_path (str): Full USD prim path of the camera
        position (np.array): Camera position in world coordinates
        euler_angles (np.array): Camera orientation in Euler angles (degrees)
        resolution (tuple): Resolution of the camera in pixels
        focal_length (float): Focal length of the camera in mm
        clipping_range (tuple): Clipping range of the camera in meters
        horizontal_aperture (float): Horizontal aperture of the camera in mm
    Returns:
        Camera object or None if camera cannot be created
    """
    # Avoid re-initializing if already created
    if prim_path in initialized_cameras:
        print(
            f"[DEBUG] Camera at {prim_path} already initialized. Returning existing instance."
        )

        return initialized_cameras[prim_path]

    stage = omni.usd.get_context().get_stage()
    camera_Xform_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_Xform_prim:
        carb.log_error(f"Camera Xform not found at {CAMERA_PATH}")
        return None
    print(f"[MAIN] Initializing camera at {prim_path}")

    quat_xyzw = rot_utils.euler_angles_to_quats(
        euler_angles, extrinsic=True, degrees=True
    )

    camera = Camera(
        prim_path=prim_path,
        resolution=resolution,
        position=position,
        orientation=quat_xyzw,
        frequency=30,
    )

    camera.initialize()

    print(f"[DEBUG] Camera frequency: {camera.get_frequency()}")
    print(f"[DEBUG] Render path: {camera.get_render_product_path()}")

    camera.set_world_pose(position, quat_xyzw, camera_axes="usd")

    # Apply physics properties
    physicsAPI = UsdPhysics.RigidBodyAPI.Apply(camera_Xform_prim)
    PhysxSchema.PhysxRigidBodyAPI.Apply(camera_Xform_prim)

    # Disable gravity for the camera
    attr = camera_Xform_prim.GetAttribute("physxRigidBody:disableGravity")

    camera.set_focal_length(focal_length / 10)
    camera.set_clipping_range(clipping_range[0], clipping_range[1])
    camera.set_horizontal_aperture(horizontal_aperture / 10)
    camera.add_motion_vectors_to_frame()

    # Explicitly register camera with camera capture system
    camera_id = prim_path.split("/")[-1]
    camera_capture.register_camera(camera_id, camera)

    print(f"[MAIN] Registered cameras: {camera_capture.get_registered_cameras()}")

    initialized_cameras[prim_path] = camera
    return camera


def get_hydra_texture_safe(camera):
    rp_path = camera.get_render_product_path()
    return omni.replicator.core.get_hydra_texture(rp_path)


def register_existing_camera(prim_path, resolution=None, add_3d_features=False):
    """
    Register an existing camera from its prim path with the camera capture system

    Args:
        prim_path (str): Full USD prim path of the existing camera

    Returns:
        Camera object or None if camera cannot be created

    """

    camera_id = prim_path.split("/")[-1]

    # Check if camera is already registered
    if camera_id in camera_capture.camera_registry:
        print(f"[DEBUG] Camera '{camera_id}' is already registered.")
        return camera_capture.camera_registry[camera_id]  # Return existing camera

    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # Check if the prim exists
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        carb.log_error(f"No valid prim found at path: {prim_path}")
        return None

    try:
        # Create Camera object from existing prim
        camera = Camera(prim_path=prim_path)
        camera.set_frequency(1)
        camera.initialize()
        # Get Hydra texture directly from render product
        if add_3d_features:
            camera.add_distance_to_image_plane_to_frame()
            camera.add_pointcloud_to_frame(include_unlabelled=True)

        print(f"[DEBUG] Camera initialized at {prim_path}")

        if resolution is not None:
            camera.set_resolution(resolution)
        print(f"[DEBUG] Camera resolution updated to {resolution}")

        # Register with camera capture system
        camera_capture.register_camera(camera_id, camera)

        # Disable render products to avoid unnecessary rendering
        # if camera._render_product:
        #     hydra_texture = camera._render_product.hydra_texture
        #     hydra_texture.set_updates_enabled(False)
        # else:
        #     carb.log_error("Render product creation failed")

        print(f"[MAIN] Successfully registered camera: {camera_id}")
        return camera

    except Exception as e:
        carb.log_error(f"Error registering camera at {prim_path}: {e}")
        return None


def register_stereo_pair(left_prim_path, right_prim_path, pair_id=None):
    """
    Register a stereo camera pair from existing cameras in the scene with a known baseline distance

    Args:
        left_prim_path (str): Full USD prim path of the left camera
        right_prim_path (str): Full USD prim path of the right camera
        baseline (float): Known baseline distance between cameras in scene units
        pair_id (str, optional): ID for the stereo pair. If None, will be generated from camera names

    Returns:
        dict or None: Stereo pair information if successful, None otherwise
    """
    # Extract camera IDs from prim paths
    left_camera_id = left_prim_path.split("/")[-1]
    right_camera_id = right_prim_path.split("/")[-1]

    # Generate pair_id if not provided
    if pair_id is None:
        pair_id = f"{left_camera_id}_{right_camera_id}"

    # Check if both cameras are registered
    if left_camera_id not in camera_capture.camera_registry:
        carb.log_warn(
            f"Left camera '{left_camera_id}' not registered. Attempting to register..."
        )
        left_camera = register_existing_camera(left_prim_path, add_3d_features=True)
        if left_camera is None:
            carb.log_error(f"Failed to register left camera '{left_camera_id}'")
            return None

    if right_camera_id not in camera_capture.camera_registry:
        carb.log_warn(
            f"Right camera '{right_camera_id}' not registered. Attempting to register..."
        )
        right_camera = register_existing_camera(right_prim_path, add_3d_features=True)
        if right_camera is None:
            carb.log_error(f"Failed to register right camera '{right_camera_id}'")
            return None

    # Register the stereo pair
    stereo_pair = {
        "left": left_camera_id,
        "right": right_camera_id,
        "left_prim_path": left_prim_path,
        "right_prim_path": right_prim_path,
    }

    camera_capture.stereo_pairs[pair_id] = stereo_pair

    print(f"[MAIN] Successfully registered stereo pair '{pair_id}': {stereo_pair}")
    return stereo_pair


def get_camera_baseline(left_prim_path, right_prim_path):
    """
    Calculate the baseline distance between two camera prims in the scene

    Args:
        left_prim_path (str): USD path to the left camera
        right_prim_path (str): USD path to the right camera

    Returns:
        float: Baseline distance in scene units
    """
    stage = omni.usd.get_context().get_stage()

    # Get camera prims
    left_prim = stage.GetPrimAtPath(left_prim_path)
    right_prim = stage.GetPrimAtPath(right_prim_path)

    if not left_prim.IsValid() or not right_prim.IsValid():
        carb.log_error("One or both camera prims are invalid.")
        return None

    # Get world transforms for both cameras
    left_xform = UsdGeom.Xformable(left_prim)
    right_xform = UsdGeom.Xformable(right_prim)

    # Get world positions
    left_matrix = left_xform.ComputeLocalToWorldTransform(0)
    right_matrix = right_xform.ComputeLocalToWorldTransform(0)

    # Extract translation components (positions)
    left_position = left_matrix.ExtractTranslation()
    right_position = right_matrix.ExtractTranslation()

    # Calculate Euclidean distance between cameras
    # Note: In Isaac Sim, the position values are halved compared to the actual scene units
    import math

    baseline = (
        math.sqrt(
            (right_position[0] - left_position[0]) ** 2
            + (right_position[1] - left_position[1]) ** 2
            + (right_position[2] - left_position[2]) ** 2
        )
        * 2
    )  # Adjust for halved units

    print(f"[DEBUG] Detected baseline between cameras: {baseline:.4f} scene units")
    return baseline
