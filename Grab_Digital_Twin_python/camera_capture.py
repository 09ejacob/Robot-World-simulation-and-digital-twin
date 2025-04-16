import os
import subprocess
import time
from datetime import datetime
from PIL import Image
import numpy as np
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
    

    def compute_depth_from_disparity(self, disparity_map, focal_length, baseline):
        """
        Convert disparity map to depth map

        Args:
            disparity_map: Numpy array of disparity values
            focal_length: Camera focal length in pixels
            baseline: Distance between cameras in scene units

        Returns:
            depth_map: Numpy array of depth values
        """
        # Avoid division by zero
        valid_disparities = disparity_map > 0

        # Initialize depth map with zeros
        depth_map = np.zeros_like(disparity_map, dtype=np.float32)

        # Calculate depth using the formula: depth = (baseline * focal_length) / disparity
        depth_map[valid_disparities] = (baseline * focal_length) / disparity_map[valid_disparities]
        print(f"Depth map computed with shape: {depth_map}")

        return depth_map
    
    def capture_stereo_images(self, stereo_pair):
            """Capture images from both stereo cameras"""
            # Get frames from both cameras
            left_path = self.capture_image(stereo_pair["left"])
            right_path = self.capture_image(stereo_pair["right"])

            # Load the images
            left_img = cv2.imread(left_path)
            right_img = cv2.imread(right_path)

            # Convert to grayscale for disparity calculation
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
            print(f"Captured stereo images from {stereo_pair['left']} and {stereo_pair['right']}")
            print(f"Left image shape: {left_img.shape}, Right image shape: {right_img.shape}")

            return left_gray, right_gray, left_img, right_img

    def compute_disparity(self, left_img, right_img):
            """Compute disparity map from stereo images"""
            # Create a stereo matcher
            stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*10,  # Must be divisible by 16
            blockSize=5,
         )
        
            # Compute disparity
            disparity = stereo.compute(left_img, right_img)
            print(f"Disparity map computed with shape: {disparity}")

            return disparity
    
    def run_stereo_pipeline(self, stereo_pair, baseline):
        """
        Run the stereo pipeline to compute disparity and depth maps.

        Args:
            stereo_pair (dict): Dictionary with keys "left" and "right" for stereo camera IDs.
            baseline (float): Distance between the stereo cameras in scene units.

        Returns:
            tuple: Disparity map and depth map.
        """
        # 1. Capture stereo images
        left_gray, right_gray, left_color, right_color = self.capture_stereo_images(stereo_pair)
        print(f"Captured images from {stereo_pair['left']} and {stereo_pair['right']}")

        # 2. Compute disparity map
        disparity_map = self.compute_disparity(left_gray, right_gray)
        print(f"Disparity map shape: {disparity_map.shape}")

        # 3. Get camera parameters
        left_camera = self.camera_registry[stereo_pair["left"]]
        focal_length = left_camera.get_focal_length() * 10  # Convert cm to mm
        resolution_width = left_camera.get_resolution()[0]
        horizontal_aperture_mm = left_camera.get_horizontal_aperture() * 10  # Convert cm to mm

        # Calculate focal length in pixels
        focal_length_pixels = (focal_length * resolution_width) / horizontal_aperture_mm

        print(f"   - Focal Length (cm): {focal_length:.2f}")
        print(f"   - Horizontal Aperture (cm): {horizontal_aperture_mm:.2f}")
        print(f"   - Computed Focal Length (pixels): {focal_length_pixels:.2f}")

        # 4. Compute depth map
        depth_map = self.compute_depth_from_disparity(disparity_map, focal_length_pixels, baseline)
        print(f"Depth map shape: {depth_map.shape}")

        # 5. Visualize or use the results
        norm_disparity = cv2.normalize(disparity_map, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite("disparity_map.jpg", norm_disparity)

        normalized_depth = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite("depth_map.jpg", normalized_depth)

        return disparity_map, depth_map