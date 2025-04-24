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
    
    def depth_to_point_cloud(self, depth_map, K):
        """
        Convert depth map to point cloud using camera intrinsics.
        Args:
            depth_map (np.ndarray): HxW array of depth values.
            K (np.ndarray): 3x3 camera intrinsic matrix.
        Returns:
            points (np.ndarray): Nx3 array of 3D points.
        """
        h, w = depth_map.shape
        i, j = np.indices((h, w))
        z = depth_map
        x = (j - K[0, 2]) * z / K[0, 0]
        y = (i - K[1, 2]) * z / K[1, 1]
        points = np.stack((x, y, z), axis=-1)
        mask = z > 0
        return points[mask]

    
    def run_stereo_pipeline(self, stereo_pair, baseline,filename=None):
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
        resolution_height = left_camera.get_resolution()[1]
        resolution_width = left_camera.get_resolution()[0]
        horizontal_aperture_mm = left_camera.get_horizontal_aperture() * 10  # Convert cm to mm
        vertical_aperture_mm = left_camera.get_vertical_aperture() * 10

        # Calculate focal length in pixels
        focal_length_pixels = (focal_length * resolution_width) / horizontal_aperture_mm

        print(f"   - Focal Length (cm): {focal_length:.2f}")
        print(f"   - Horizontal Aperture (cm): {horizontal_aperture_mm:.2f}")
        print(f"   - Computed Focal Length (pixels): {focal_length_pixels:.2f}")

        fx = (focal_length * resolution_width) / horizontal_aperture_mm
        fy = (focal_length * resolution_height) / vertical_aperture_mm
        cx = resolution_width / 2
        cy = resolution_height / 2

        # 4. Compute depth map
        depth_map = self.compute_depth_from_disparity(disparity_map, focal_length_pixels, baseline)
        print(f"Depth map shape: {depth_map.shape}")

        # 5. Construct intrinsic matrix
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]
        ])
        print(f"Intrinsic matrix K:\n{K}")


        point_cloud = self.depth_to_point_cloud(depth_map, K)
        print(f"Generated point cloud with {point_cloud.shape[0]} points")
    

        # 7. Save the point cloud
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"stereo_pointcloud_{timestamp}.ply"

        save_dir = os.path.join(self.base_save_dir, "stereo_pointclouds")
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, filename)

        try:
            with open(save_path, "w") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"element vertex {point_cloud.shape[0]}\n")
                f.write("property float x\nproperty float y\nproperty float z\n")
                f.write("end_header\n")
                for pt in point_cloud:
                    f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
            print(f"💾 Stereo pointcloud saved to: {save_path}")
        except Exception as e:
            print(f"❌ Failed to save stereo pointcloud: {e}")
            return None

        # 5. Visualize or use the results
        norm_disparity = cv2.normalize(disparity_map, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite("disparity_map.jpg", norm_disparity)

        normalized_depth = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite("depth_map.jpg", normalized_depth)

        return point_cloud
    
    def extract_rgb_from_rgba(rgba_image):
        """
        Extract the RGB channels from an RGBA image.
        
        Args:
            rgba_image (np.ndarray): Input image with shape (H, W, 4).
        
        Returns:
            np.ndarray: Output image with shape (H, W, 3) containing only RGB channels.
        """
        # Slice along the last dimension to ignore the alpha channel
        rgb_image = rgba_image[..., :3]
        return rgb_image
    
    def refine_with_icp(self,left_pc, right_pc_transformed, voxel_size=0.01, icp_threshold=0.05):
        """
        Refine alignment between two point clouds using ICP.

        Args:
            left_pc (np.ndarray): Left point cloud (Nx3) in left camera (reference) frame.
            right_pc_transformed (np.ndarray): Right camera point cloud, already transformed into left frame (Nx3).
            voxel_size (float): Voxel size for downsampling.
            icp_threshold (float): Maximum correspondence distance (in meters) for ICP.

        Returns:
            np.ndarray: Transformation matrix (4x4) from right to left refinement.
        """
        # Convert numpy arrays into open3d point clouds
        left_pcd = o3d.geometry.PointCloud()
        left_pcd.points = o3d.utility.Vector3dVector(left_pc)

        right_pcd = o3d.geometry.PointCloud()
        right_pcd.points = o3d.utility.Vector3dVector(right_pc_transformed)

        # Optional: Downsample for speed (adjust the voxel size as appropEngasjementriate)
        left_down = left_pcd.voxel_down_sample(voxel_size)
        right_down = right_pcd.voxel_down_sample(voxel_size)
        print(f"Downsampled point clouds to voxel size: {left_down}")

        left_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30))
        right_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30))

        # Run ICP (point-to-plane works well on organized, smooth clouds)
        # You can choose point-to-point as well.
        icp_result = o3d.pipelines.registration.registration_icp(
            right_down, left_down, icp_threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )

        print("Fitness:", icp_result.fitness)
        print("Refinement transformation:\n", icp_result.transformation)

        return icp_result.transformation

    
    def capture_combined_pointcloud(self, pair_id, filename=None, save=True):
        """
        Capture and merge point clouds from a registered stereo camera pair.

        Args:
            pair_id (str): ID of the stereo pair (as registered via register_stereo_pair)
            filename (str, optional): Custom filename to save the merged point cloud
            save (bool): Whether to save the merged point cloud to a PLY file

        Returns:
            np.ndarray: Combined point cloud (Nx3), or None if failure
        """
        if pair_id not in self.stereo_pairs:
            print(f"❌ Stereo pair '{pair_id}' not found.")
            return None

        stereo = self.stereo_pairs[pair_id]
        left_id = stereo["left"]
        right_id = stereo["right"]
        left_path = stereo["left_prim_path"]
        right_path = stereo["right_prim_path"]

        # Fetch point clouds
        left_cam = self.camera_registry[left_id]
        right_cam = self.camera_registry[right_id]

        left_frame = left_cam.get_current_frame()
        right_frame = right_cam.get_current_frame()

        if not left_frame or "pointcloud" not in left_frame or "data" not in left_frame["pointcloud"]:
            print(f"❌ No pointcloud from left camera '{left_id}'")
            return None
        if not right_frame or "pointcloud" not in right_frame or "data" not in right_frame["pointcloud"]:
            print(f"❌ No pointcloud from right camera '{right_id}'")
            return None

        left_pc = np.array(left_frame["pointcloud"]["data"])
        right_pc = np.array(right_frame["pointcloud"]["data"])
      
        stage = omni.usd.get_context().get_stage()
        left_xform = UsdGeom.Xformable(stage.GetPrimAtPath(left_path))
        right_xform = UsdGeom.Xformable(stage.GetPrimAtPath(right_path))
        left_matrix = left_xform.ComputeLocalToWorldTransform(0)
        right_matrix = right_xform.ComputeLocalToWorldTransform(0)
        # Validate transformation matrix components
        print("Left camera world matrix:\n", np.array(left_matrix))
        print("Right camera world matrix:\n", np.array(right_matrix))
        print(Gf.Transform(left_matrix)) 
        print(Gf.Transform(left_matrix).GetTranslation())


        # Compute right->left transformation
        right_to_left_matrix = left_matrix.GetInverse() * right_matrix
        print("Right-to-left transformation:\n", np.array(right_to_left_matrix))
        

        def transform_pointcloud(points, matrix):
            points_hom = np.hstack([points, np.ones((points.shape[0], 1))])
            mat_np = np.array(matrix).reshape(4, 4).T  # USD uses column-major
            transformed = np.dot(points_hom, mat_np)
            return transformed[:, :3]

        right_pc_transformed = transform_pointcloud(right_pc, right_to_left_matrix)
        icp_transformation = self.refine_with_icp(left_pc, right_pc_transformed, 1,1)
        right_pc_refined = transform_pointcloud(right_pc_transformed, icp_transformation)

        merged_pc = np.vstack([left_pc, right_pc_refined])

        # Merge
        combined_pc = merged_pc  # left_pc + right_pc_refined


        print(f"✅ Merged pointcloud has {combined_pc.shape[0]} points")

        # Save if requested
        if save:
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{pair_id}_merged_pointcloud_{timestamp}.ply"

            save_dir = os.path.join(self.base_save_dir, pair_id, "pointclouds")
            os.makedirs(save_dir, exist_ok=True)
            save_path = os.path.join(save_dir, filename)

            try:
                with open(save_path, "w") as f:
                    f.write("ply\nformat ascii 1.0\n")
                    f.write(f"element vertex {combined_pc.shape[0]}\n")
                    f.write("property float x\nproperty float y\nproperty float z\n")
                    f.write("end_header\n")
                    for pt in combined_pc:
                        f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")
                print(f"💾 Combined pointcloud saved to: {save_path}")
            except Exception as e:
                print(f"❌ Failed to save merged pointcloud: {e}")
                return None

        return merged_pc

