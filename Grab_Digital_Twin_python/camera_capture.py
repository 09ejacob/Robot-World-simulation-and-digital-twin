import os
import subprocess
import time
from datetime import datetime
from PIL import Image
from pxr import UsdGeom  # Import missing module
import numpy as np
import omni.usd
import open3d as o3d
from pxr import Gf
import cv2



class CameraCapture:
    """
    A class to manage camera captures in Isaac Sim.
    Handles image capturing, storage, and scheduling for multiple cameras.
    """
    _instance = None

    def __new__(cls):
         if cls._instance is None:
             cls._instance = super(CameraCapture, cls).__new__(cls)
             cls._instance.initialize()
         return cls._instance

    def initialize(self):
        self.base_save_dir = "camera_captures"
        self.camera_registry = {}
        self.capture_counters = {}
        self.stereo_pairs = {}  # Stores registered stereo pairs
        self.last_capture_time = {}

        self.scenario_start = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        print("Camera Capture System Initialized at", self.scenario_start)

    def register_camera(self, camera_id, camera):
        """
        Register a camera for capturing.
        Returns the path to the camera's dedicated subfolder.
        """
        self.camera_registry[camera_id] = camera
        self.capture_counters[camera_id] = 0
        self.last_capture_time[camera_id] = 0

        # Create camera-specific directory in a subfolder named after the scenario start time
        camera_dir = os.path.join(self.base_save_dir, camera_id, self.scenario_start)
        if not os.path.exists(camera_dir):
            os.makedirs(camera_dir)
            print(f"Created directory for camera {camera_id}: {camera_dir}")

        return camera_dir

    def capture_image(self, camera_id, filename=None):
        """
        Capture an image from the specified camera with debugging.
        """

        # Check if the camera exists
        if camera_id not in self.camera_registry:
            print(f"❌ Error: Camera with ID {camera_id} not registered.")
            return None

        camera = self.camera_registry[camera_id]

        # Ensure we have the latest frame
        frame = camera.get_current_frame()
        print(f"🔄 Fetching latest frame from {camera_id}..."
              f" Frame data: {frame}")

        # Check if frame has RGBA data
        if frame is None or "rgba" not in frame:
            print(f"❌ Error: No valid frame data from camera {camera_id}")
            return None

        # Extract RGB from RGBA
        rgba = frame["rgba"]
        if rgba is None or rgba.size == 0:
            print(f"❌ Error: Empty RGBA data from camera {camera_id}")
            return None

        rgb_img = rgba[:, :, :3]
        if rgb_img is None or rgb_img.size == 0:
            print(f"❌ Error: Failed to extract RGB data from camera {camera_id}")
            return None

        if len(rgb_img.shape) != 3 or rgb_img.shape[2] != 3:
            print(
                f"❌ Error: Invalid image shape {rgb_img.shape} from camera {camera_id}"
            )
            return None

        try:
            image = Image.fromarray(rgb_img, mode="RGB")
        except Exception as e:
            print(f"❌ Error converting image to PIL format: {e}")
            return None

        if filename is None:
            counter = self.capture_counters[camera_id]
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            filename = f"{camera_id}_{timestamp}_{counter:04d}.jpg"
            self.capture_counters[camera_id] += 1

        # Use the registered camera folder (which now includes the scenario start subfolder)
        camera_dir = os.path.join(self.base_save_dir, camera_id, self.scenario_start)
        os.makedirs(camera_dir, exist_ok=True)
        save_path = os.path.join(camera_dir, filename)

        try:
            image.save(save_path)
        except Exception as e:
            print(f"❌ Error saving image from {camera_id}: {e}")
            return None

        return save_path

    def capture_timed(self, camera_id, interval_seconds=1.0):
        current_time = time.time()
        if camera_id not in self.last_capture_time:
            self.last_capture_time[camera_id] = 0

        if current_time - self.last_capture_time[camera_id] >= interval_seconds:
            self.last_capture_time[camera_id] = current_time
            return self.capture_image(camera_id)

        return None

    def capture_all_cameras(self):
        results = {}
        for camera_id in self.camera_registry:
            results[camera_id] = self.capture_image(camera_id)
        return results

    def capture_all_timed(self, interval_seconds=2.0):
        results = {}
        for camera_id in self.camera_registry:
            result = self.capture_timed(camera_id, interval_seconds)
            if result:
                results[camera_id] = result
        return results

    def get_registered_cameras(self):
        print(
            f"Registered Cameras in camera Capture: {list(self.camera_registry.keys())}"
        )
        return list(self.camera_registry.keys())

    def convert_video_from_images(self, fps):
        folder_path = os.path.join(
            self.base_save_dir, "OverviewCamera", self.scenario_start
        )
        output_file = os.path.join(
            folder_path, f"Overview-Camera-{self.scenario_start}.mp4"
        )

        command = (
            f"ffmpeg -y -framerate {fps} -pattern_type glob -i '{os.path.join(folder_path, '*.jpg')}' "
            f"-c:v libx264 -pix_fmt yuv420p {output_file}"
        )

        print("Running FFMPEG command:")
        print(command)

        subprocess.call(command, shell=True)

        print(f"Video conversion complete. Video saved at: {output_file}")

        for filename in os.listdir(folder_path):
            if filename.lower().endswith(".jpg"):
                file_path = os.path.join(folder_path, filename)
                os.remove(file_path)

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
        print("Frame data:", frame.keys())
        pc = frame["pointcloud"]
        pointcloud_data = np.array(pc["data"])  # Shape: (N, 3)
        point_colors =  np.array(pc["pointRgb"]).reshape(-1, 4)[:, :3]
        print("🔍 First 5 colors (raw):", point_colors[:5])
        print("The length of point colors:", len(point_colors))
    
        point_normals = frame["pointcloud"]["pointNormals"]  # Normal vectors

        # After extracting components
        if point_colors is not None:
            print(f"Color data shape: {np.array(point_colors).shape}")
         # If it's a 2D array, it might be an image
    
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            counter = self.capture_counters[camera_id]
            filename = os.path.join(
            self.base_save_dir,
            camera_id,
            "pointclouds",
            f"{camera_id}_pointcloud_{timestamp}_{counter:04d}.npz"
           )
            self.capture_counters[camera_id] = counter + 1

        os.makedirs(os.path.dirname(filename), exist_ok=True)

    # Save data
        try:
            np.savez_compressed(
            filename,
            points=pointcloud_data,
            colors=point_colors,
            normals=point_normals if point_normals.size > 0 else None
        )
            print(f"✅ Saved to {filename}")
            return filename
        except Exception as e:
               print(f"❌ Save failed: {e}")
        return None


    def capture_stereo_pointcloud_pair(self, pair_id, pair_name=None):
        """
        Capture synchronized stereo pair and save to timestamped directory.

        Args:
            pair_id (str): Registered stereo pair ID
            pair_name (str): Optional custom directory name

        Returns:
            dict: Paths to saved NPZ files and directory
        """
        print(f"\n🔄 Capturing stereo pair {pair_id}...")

        if pair_id not in self.stereo_pairs:
            print(f"❌ Pair {pair_id} not registered")
            return None

        # Directory setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pair_name = pair_name or f"{pair_id}_{timestamp}"
        stereo_dir = os.path.join(self.base_save_dir, "stereo_pairs", pair_name)
        os.makedirs(stereo_dir, exist_ok=True)

        # Camera IDs
        left_id = self.stereo_pairs[pair_id]["left"]
        right_id = self.stereo_pairs[pair_id]["right"]

        # Capture both cameras
        results = {
            "left_npz": self.capture_pointcloud(
                left_id,
                filename=os.path.join(stereo_dir, f"{left_id}.npz")
            ),
            "right_npz": self.capture_pointcloud(
                right_id,
                filename=os.path.join(stereo_dir, f"{right_id}.npz")
            ),
            "pair_dir": stereo_dir
        }

        if all(results.values()):
            print(f"✅ Stereo pair saved to {stereo_dir}")
        else:
            print("⚠️ Partial capture - check individual cameras")

        return results

  