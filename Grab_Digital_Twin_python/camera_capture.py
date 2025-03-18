import os
import time
from datetime import datetime
from PIL import Image
import numpy as np

class CameraCapture:
    """
    A class to manage camera captures in Isaac Sim.
    Handles image capturing, storage, and scheduling for multiple cameras.
    """
    
    def __init__(self, base_save_dir="camera_captures"):
        """
        Initialize the camera capture system.
        
        Args:
            base_save_dir (str): Base directory to store all camera captures
        """
        # Create base directory for all captures
        self.base_save_dir = base_save_dir
        if not os.path.exists(base_save_dir):
            os.makedirs(base_save_dir)
            print(f"Created base capture directory: {base_save_dir}")
        
        # Track capture statistics
        self.capture_counters = {}
        self.last_capture_time = {}
        
        # Keep track of registered cameras
        self.camera_registry = {}
        print("Camera Capture System Initialized")
    
    def register_camera(self, camera_id, camera):
        """
        Register a camera for capturing.
        
        Args:
            camera_id (str): Unique identifier for the camera
            camera (Camera): Camera object from Isaac Sim
            
        Returns:
            str: Path to the camera's dedicated directory
        """
        self.camera_registry[camera_id] = camera
        self.capture_counters[camera_id] = 0
        self.last_capture_time[camera_id] = 0
        
        # Create camera-specific directory
        camera_dir = os.path.join(self.base_save_dir, camera_id)
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)
            print(f"Created directory for camera {camera_id}: {camera_dir}")
        
        return camera_dir
    
    
    def capture_image(self, camera_id, filename=None):
        """
        Capture an image from the specified camera with additional debugging.
        
        Args:
            camera_id (str): ID of the camera to capture from
            filename (str, optional): Custom filename for the image
            
        Returns:
            str: Path to the saved image file, or None if capture failed
        """
        print(f"\n📸 Attempting to capture image from {camera_id}...")

        # 🔹 Check if the camera exists
        if camera_id not in self.camera_registry:
            print(f"❌ Error: Camera with ID {camera_id} not registered.")
            return None
        print(f"✅ Camera {self.camera_registry} found in registry.")
        print(f"✅ Camera {camera_id} found in registry.")

        camera = self.camera_registry[camera_id]

        # 🔹 Ensure we have the latest frame
        print(f"🔄 Fetching latest frame from {camera_id}...")
        camera.get_current_frame()  
        print(f"✅ Latest frame fetched from {camera_id}." + str(camera.get_current_frame()))
        
        # 🔹 Capture RGB image
        rgb_img = camera.get_rgb()
        
        if rgb_img is None:
            print(f"❌ Error: Failed to capture image from camera {camera_id}" )
            return None

        # 🔹 Log image shape and type
        print(f"✅ Captured image from {camera_id} with shape: {rgb_img.shape}, dtype: {rgb_img.dtype}" + str(rgb_img))

        # 🔹 Ensure the image has 3 dimensions (H, W, C)
        if len(rgb_img.shape) != 3 or rgb_img.shape[2] != 3:
            print(f"❌ Error: Invalid image shape {rgb_img.shape} from camera {camera_id}")
            return None

        # 🔹 Convert Float Image to Uint8 Format
        if rgb_img.dtype in [np.float32, np.float64]:
            print(f"🔄 Converting float image to uint8 for {camera_id}...")
            rgb_img = (rgb_img * 255).clip(0, 255).astype(np.uint8)

        # 🔹 Ensure image is in "RGB" format
        try:
            image = Image.fromarray(rgb_img, mode="RGB")
        except Exception as e:
            print(f"❌ Error converting image to PIL format: {e}")
            return None

        # 🔹 Generate filename if not provided
        if filename is None:
            counter = self.capture_counters[camera_id]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{camera_id}_{timestamp}_{counter:04d}.png"
            self.capture_counters[camera_id] += 1

        # 🔹 Save the image
        camera_dir = os.path.join(self.base_save_dir, camera_id)
        os.makedirs(camera_dir, exist_ok=True)  # Ensure directory exists
        save_path = os.path.join(camera_dir, filename)

        try:
            print(f"💾 Saving image at: {save_path}")
            image.save(save_path)
            print(f"✅ Image saved successfully: {save_path}")
        except Exception as e:
            print(f"❌ Error saving image from {camera_id}: {e}")
            return None

        return save_path


    def capture_timed(self, camera_id, interval_seconds=1.0):
        """
        Capture an image if enough time has passed since the last capture.
        
        Args:
            camera_id (str): ID of the camera to capture from
            interval_seconds (float): Minimum time between captures
            
        Returns:
            str: Path to the saved image file, or None if no capture was made
        """
        current_time = time.time()
        if camera_id not in self.last_capture_time:
            self.last_capture_time[camera_id] = 0
            
        if current_time - self.last_capture_time[camera_id] >= interval_seconds:
            self.last_capture_time[camera_id] = current_time
            return self.capture_image(camera_id)
        
        return None

    def capture_all_cameras(self):
        """
        Capture images from all registered cameras.
        
        Returns:
            dict: Map of camera IDs to saved image paths
        """
        results = {}
        for camera_id in self.camera_registry:
            results[camera_id] = self.capture_image(camera_id)
        return results
    
    def capture_all_timed(self, interval_seconds=1.0):
        """
        Capture images from all cameras if interval has passed.
        
        Args:
            interval_seconds (float): Minimum time between captures
            
        Returns:
            dict: Map of camera IDs to saved image paths (only for cameras that captured)
        """
        results = {}
        for camera_id in self.camera_registry:
            result = self.capture_timed(camera_id, interval_seconds)
            if result:
                results[camera_id] = result
        return results

    def get_camera_registry(self):
        """Get the dictionary of registered cameras"""
        return self.camera_registry
