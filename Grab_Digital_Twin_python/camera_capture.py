import os
import time
from datetime import datetime
from PIL import Image


class CameraCapture:
    """
    A class to manage camera captures in Isaac Sim.
    Handles image capturing, storage, and scheduling for multiple cameras.
    """
    camera_registry = {}  # class-level variable
    base_save_dir = "camera_captures"
    capture_counters = {}
    last_capture_time = {}
    _instance = None

    # def __new__(cls):
    #     if cls._instance is None:
    #         cls._instance = super(CameraCapture, cls).__new__(cls)
    #         cls._instance.initialize()
    #     return cls._instance

    def initialize(self):
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
        """
        print(f"\n📸 Attempting to capture image from {camera_id}...")

        # Check if the camera exists
        if camera_id not in self.camera_registry:
            print(f"❌ Error: Camera with ID {camera_id} not registered.")
            return None
        print(
            f"✅ Camera {camera_id} found in registry.",
            list(self.camera_registry.keys()),
        )

        camera = self.camera_registry[camera_id]

        # Ensure we have the latest frame
        print(f"🔄 Fetching latest frame from {camera_id}...")
        frame = camera.get_current_frame()
        print(f"✅ Latest frame fetched from {camera_id}. Frame info: {frame}")

        # Check if frame has RGBA data
        if frame is None or "rgba" not in frame:
            print(f"❌ Error: No valid frame data from camera {camera_id}")
            return None

        # Extract RGB from RGBA
        rgba = frame["rgba"]
        if rgba is None or rgba.size == 0:
            print(f"❌ Error: Empty RGBA data from camera {camera_id}")
            return None

        # Get RGB by dropping alpha channel
        rgb_img = rgba[:, :, :3]

        if rgb_img is None or rgb_img.size == 0:
            print(f"❌ Error: Failed to extract RGB data from camera {camera_id}")
            return None

        # Log image shape and type
        print(
            f"✅ Captured image from {camera_id} with shape: {rgb_img.shape}, dtype: {rgb_img.dtype}"
        )

        # Ensure the image has 3 dimensions (H, W, C)
        if len(rgb_img.shape) != 3 or rgb_img.shape[2] != 3:
            print(
                f"❌ Error: Invalid image shape {rgb_img.shape} from camera {camera_id}"
            )
            return None

        # Ensure image is in "RGB" format
        try:
            image = Image.fromarray(rgb_img, mode="RGB")
        except Exception as e:
            print(f"❌ Error converting image to PIL format: {e}")
            return None
        
        # Generate filename if not provided
        if filename is None:
            counter = self.capture_counters[camera_id]
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            filename = f"{camera_id}_{timestamp}_{counter:04d}.jpg"
            self.capture_counters[camera_id] += 1

        # Save the image
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

    def capture_all_timed(self, interval_seconds=2.0):
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

    def get_registered_cameras(self):
        """Get list of registered camera IDs"""
        print(
            f"Registered Cameras in camera Capture: {list(self.camera_registry.keys())}"
        )
        return list(self.camera_registry.keys())

    def capture_pointcloud(self, camera_id, filename=None):
        """
        Capture a 3D point cloud from the specified camera and save it as a PLY file.

        Args:
            camera_id (str): ID of the camera to capture from
            filename (str, optional): Custom filename for the PLY file

        Returns:
            str: Path to the saved PLY file, or None if capture failed
        """
        print(f"\n📸 Attempting to capture pointcloud from {camera_id}...")

        # Check if the camera exists
        if camera_id not in self.camera_registry:
            print(f"❌ Error: Camera with ID {camera_id} not registered.")
            return None
        print(
            f"✅ Camera {camera_id} found in registry.",
            list(self.camera_registry.keys()),
        )

        camera = self.camera_registry[camera_id]

        # Get the latest frame
        print(f"🔄 Fetching latest frame with pointcloud from {camera_id}...")
        frame = camera.get_current_frame()

        # Check if frame has pointcloud data
        if (
            frame is None
            or "pointcloud" not in frame
            or "data" not in frame["pointcloud"]
        ):
            print(f"❌ Error: No valid pointcloud data from camera {camera_id}")
            return None

        # Extract pointcloud components
        pointcloud_data = frame["pointcloud"]["data"]  # XYZ coordinates
        point_colors = frame["pointcloud"]["pointRgb"]  # RGB colors
        point_normals = frame["pointcloud"]["pointNormals"]  # Normal vectors

        # Check if we have valid data
        if pointcloud_data is None or len(pointcloud_data) == 0:
            print(f"❌ Error: Empty pointcloud data from camera {camera_id}")
            return None

        print(
            f"✅ Captured pointcloud from {camera_id} with {len(pointcloud_data)} points"
        )

        # Generate filename if not provided
        if filename is None:
            counter = self.capture_counters.get(camera_id, 0)
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            filename = f"{camera_id}_pointcloud_{timestamp}_{counter:04d}.ply"
            self.capture_counters[camera_id] = counter + 1

        # Ensure file has .ply extension
        if not filename.lower().endswith(".ply"):
            filename += ".ply"

        # Create directory for pointcloud
        camera_dir = os.path.join(self.base_save_dir, camera_id, "pointclouds")
        os.makedirs(camera_dir, exist_ok=True)
        save_path = os.path.join(camera_dir, filename)

        try:
            # Write to PLY file
            print(f"💾 Saving pointcloud at: {save_path}")
            with open(save_path, "w") as f:
                # Write header
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(pointcloud_data)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")

                # Include color properties if available
                if point_colors is not None and len(point_colors) == len(
                    pointcloud_data
                ):
                    f.write("property uchar red\n")
                    f.write("property uchar green\n")
                    f.write("property uchar blue\n")

                # Include normal properties if available
                has_normals = point_normals is not None and len(point_normals) == len(
                    pointcloud_data
                )
                if has_normals:
                    f.write("property float nx\n")
                    f.write("property float ny\n")
                    f.write("property float nz\n")

                f.write("end_header\n")

                # Write vertex data
                for i in range(len(pointcloud_data)):
                    line = f"{pointcloud_data[i][0]} {pointcloud_data[i][1]} {pointcloud_data[i][2]}"

                    # Add color if available
                    if point_colors is not None and len(point_colors) == len(
                        pointcloud_data
                    ):
                        line += f" {point_colors[i][0]} {point_colors[i][1]} {point_colors[i][2]}"

                    # Add normals if available
                    if has_normals:
                        line += f" {point_normals[i][0]} {point_normals[i][1]} {point_normals[i][2]}"

                    f.write(line + "\n")

            print(f"✅ Pointcloud saved successfully: {save_path}")
            return save_path

        except Exception as e:
            print(f"❌ Error saving pointcloud from {camera_id}: {e}")
            return None
    
