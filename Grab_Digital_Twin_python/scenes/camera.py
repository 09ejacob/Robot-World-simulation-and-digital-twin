import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
import omni.usd
from isaacsim.sensors.camera import Camera
import omni.replicator.core as rep
from pxr import UsdPhysics, PhysxSchema
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
    # Avoid re-initializing if already created
    if prim_path in initialized_cameras:
        print(
            f"Camera at {prim_path} already initialized. Returning existing instance."
        )
        return initialized_cameras[prim_path]

    stage = omni.usd.get_context().get_stage()
    camera_Xform_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_Xform_prim:
        print(f"Camera Xform not found at {CAMERA_PATH}")
        return None
    print(f"Initializing camera at {prim_path}")

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

    print(camera.get_frequency())
    print("the render path is", camera.get_render_product_path())

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

    print("✅ Registered cameras:", camera_capture.get_registered_cameras())

    initialized_cameras[prim_path] = camera
    return camera


def register_existing_camera(prim_path, resolution=None):
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
        print(f"✅ Camera '{camera_id}' is already registered.")
        return camera_capture.camera_registry[camera_id]  # Return existing camera

    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # Check if the prim exists
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"No valid prim found at path: {prim_path}")
        return None

    try:
        # Create Camera object from existing prim
        camera = Camera(prim_path=prim_path)
        # camera.set_frequency(30)
        camera.initialize()
        camera.add_distance_to_image_plane_to_frame()
        pointcloud_annotator = rep.annotators.get("pointcloud")
        pointcloud_annotator.attach(camera.get_render_product_path())
        camera.add_pointcloud_to_frame(include_unlabelled=True)

        print(f"Camera initialized at {prim_path}")

        # camera.set_frequency(30)
        print(camera.get_frequency())

        if resolution is not None:
            camera.set_resolution(resolution)
        print(f"Camera resolution updated to {resolution}")

        # Register with camera capture system
        camera_id = prim_path.split("/")[-1]
        camera_capture.register_camera(camera_id, camera)

        print(f"Successfully registered camera: {camera_id}")
        return camera

    except Exception as e:
        print(f"Error registering camera at {prim_path}: {e}")
        return None
