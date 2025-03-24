import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.camera import Camera
import omni.usd
import imageio  # For saving images
import time 
from pxr import UsdPhysics, PhysxSchema
from ..global_variables import (
    CAMERA_PATH,
)
initialized_cameras = {}

def setup_camera(
    prim_path="",
    position=np.array([0, 0, 0]),  
    euler_angles=np.array([0, 0, 0]),
    resolution=(1920, 1080),
    focal_length=35,
    clipping_range=(1,10000), 
    horizontal_aperture= 20,
    camera_capture=None
 
):
    
# Avoid re-initializing if already created
    if prim_path in initialized_cameras:
        print(f"Camera at {prim_path} already initialized. Returning existing instance.")
        return initialized_cameras[prim_path]
    
    
    stage = omni.usd.get_context().get_stage()
    camera_Xform_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_Xform_prim:
        print(f"Camera Xform not found at {CAMERA_PATH}")
        return None
    print(f"Initializing camera at {prim_path}")

    quat_xyzw = rot_utils.euler_angles_to_quats(euler_angles, extrinsic=True, degrees=True)

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
    #camera.add_motion_vectors_to_frame()

   # Get a test frame to ensure the camera is working
    try:
        frame = camera.get_current_frame()
        print(f"Camera initialized with frame data available: {frame is not None}")
        
        # Try to get RGBA data to verify camera is working
        if frame is not None:
            if isinstance(frame, dict) and 'rgba' in frame:
                rgba = frame['rgba']
                print(f"RGBA shape: {rgba.shape if rgba is not None else 'None'}")
            else:
                print(f"Frame type: {type(frame)}")
        else:
            print("No frame data available yet")
    except Exception as e:
        print(f"Warning: Could not get test frame: {e}")

    #import time
    #time.sleep(0.5)
      # Capture a frame and save it
    #try:
    #    frame = camera.get_current_frame()
    #    if frame and "rgba" in frame:
    #        rgba = frame["rgba"]  # Extract RGBA data
    #        if rgba is not None:
    #            image_filename = f"captured_frame_{int(time.time())}.png"
    #            imageio.imwrite(image_filename, rgba)  # Save image
    #            print(f"Frame saved as {image_filename}")
    #        else:
    #            print("RGBA frame data is None")
    #    else:
    #        print("Frame capture failed or RGBA key missing")
    #except Exception as e:
    #    print(f"Warning: Could not capture frame: {e}")

    # Register with camera capture system if provided
    if camera_capture is not None:
        camera_id = prim_path.split('/')[-1]  # Use the last part of the path as ID
        camera_capture.register_camera(camera_id, camera)
    
    return camera
