import os
import subprocess
import time
from datetime import datetime, timezone
from PIL import Image
import numpy as np
import cv2
import json
import struct
import carb
import omni.kit.app


class CameraController:
    """
    A class to manage camera captures in Isaac Sim.
    Handles image capturing, storage, and scheduling for multiple cameras.
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(CameraController, cls).__new__(cls)
            cls._instance.initialize()
        return cls._instance

    def initialize(self):
        self.base_save_dir = "camera_captures"
        self.camera_registry = {}
        self.capture_counters = {}
        self.stereo_pairs = {}  # Stores registered stereo pairs
        self.last_capture_time = {}

        self.scenario_start = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        print(f"[MAIN] Camera Capture System Initialized at {self.scenario_start}")

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
            print(f"[DEBUG] Created directory for camera {camera_id}: {camera_dir}")

        return camera_dir

    def _capture_image_array(self, camera_id):
        """
        Captures the current RGB image array from a registered camera.

        Parameters:
        camera_id (str): Identifier for the camera.

        Returns:
        np.ndarray or None: A uint8 NumPy array representing the RGB image
        if successful, or None if an error occurs (e.g., camera not registered,
        no frame available, or invalid frame data).
        """
        if camera_id not in self.camera_registry:
            carb.log_error(f"Camera {camera_id} not registered.")
            return None

        camera = self.camera_registry[camera_id]

        # Set high resolution just before capture
        frame = camera.get_current_frame()
        if not frame or "rgba" not in frame:
            carb.log_error(f"No 'rgba' frame for {camera_id}")
            return None

        rgba = frame["rgba"]
        if rgba is None or rgba.size == 0:
            carb.log_error(f"Empty RGBA data from {camera_id}")
            return None

        rgb_array = rgba[:, :, :3]
        if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
            carb.log_error(f"Unexpected image shape {rgb_array.shape} from {camera_id}")
            return None

        return rgb_array.astype(np.uint8)

    def _generate_capture_metadata(self, camera_id, rgb_array):
        utc_now = datetime.utcnow().replace(tzinfo=timezone.utc)
        local_now = datetime.now().astimezone()

        frame_id = self.capture_counters[camera_id]
        timestamp = utc_now.isoformat()
        filename_base = f"{camera_id}_{timestamp.replace(':', '-').replace('T', '_').replace('Z', '')}_{frame_id:04d}"

        metadata = {
            "camera_id": camera_id,
            "timestamp_utc": utc_now.isoformat(),
            "timestamp_local": local_now.isoformat(),
            "frame_id": frame_id,
            "image_format": "jpeg",
            "image_shape": rgb_array.shape,
            "scenario_id": self.scenario_start,
        }

        return filename_base, metadata

    def _save_image(self, camera_id, rgb_array):
        """
        Save the captured image to disk and return the file path.
        Parameters:
        camera_id (str): Identifier for the camera.
        rgb_array (np.ndarray): The RGB image array to save.
        Returns:
        str: The path to the saved image file.
        """
        try:
            image = Image.fromarray(rgb_array, mode="RGB")
            filename_base, metadata = self._generate_capture_metadata(
                camera_id, rgb_array
            )

            self.capture_counters[camera_id] += 1

            camera_dir = os.path.join(
                self.base_save_dir, camera_id, self.scenario_start
            )
            os.makedirs(camera_dir, exist_ok=True)

            image_path = os.path.join(camera_dir, f"{filename_base}.jpg")
            metadata_path = os.path.join(camera_dir, f"{filename_base}.json")

            image.save(image_path)

            metadata_path = os.path.join(camera_dir, f"{filename_base}.json")
            with open(metadata_path, "w") as f:
                json.dump(metadata, f, indent=4)

            return image_path

        except Exception as e:
            carb.log_error(f"Failed to save image for {camera_id}: {e}")
            return None

    def capture_image(self, camera_id):
        """
        Capture an image from the specified camera and save it to disk.
        Parameters:
        camera_id (str): Identifier for the camera.
        Returns:
        str: The path to the saved image file, or None if capture failed.
        """
        rgb_array = self._capture_image_array(camera_id)
        if rgb_array is None:
            return None
        return self._save_image(camera_id, rgb_array)

    def capture_and_stream(self, camera_id, udp_controller, host, port):
        """
        Capture an image from the specified camera, save it to disk, and stream it over UDP.
        Parameters:
        camera_id (str): Identifier for the camera.
        udp_controller (object): UDP controller for sending the image.
        host (str): Host address for UDP streaming.
        port (int): Port number for UDP streaming.
        Returns:
        str: The path to the saved image file, or None if capture failed.
        """
        rgb_array = self._capture_image_array(camera_id)
        if rgb_array is None:
            carb.log_error(f"Could not capture image from {camera_id}")
            return None

        save_path = self._save_image(camera_id, rgb_array)

        try:
            success, jpeg_data = cv2.imencode(".jpg", rgb_array)
            if not success:
                carb.log_error(f"Failed to compress image from {camera_id}")
                return save_path

            _, metadata = self._generate_capture_metadata(camera_id, rgb_array)
            metadata_bytes = json.dumps(metadata).encode("utf-8")
            metadata_length = struct.pack("!I", len(metadata_bytes))
            packet = metadata_length + metadata_bytes + jpeg_data.tobytes()

            udp_controller.send(packet, host, port)

        except Exception as e:
            carb.log_error(f"Streaming image failed for {camera_id}: {e}")

        return save_path

    def capture_timed(self, camera_id, interval_seconds=1.0):
        """
        Capture an image from the specified camera at regular intervals.
        Parameters:
        camera_id (str): Identifier for the camera.
        interval_seconds (float): Time interval between captures in seconds.
        Returns:
        str: The path to the saved image file, or None if capture failed.
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
        Capture images from all registered cameras and save them to disk.
        Returns:
        dict: A dictionary mapping camera IDs to the paths of saved images.
        """
        results = {}
        for camera_id in self.camera_registry:
            results[camera_id] = self.capture_image(camera_id)
        return results

    def capture_all_timed(self, interval_seconds=2.0):
        """
        Capture images from all registered cameras at regular intervals.
        Parameters:
        interval_seconds (float): Time interval between captures in seconds.
        Returns:
        dict: A dictionary mapping camera IDs to the paths of saved images.
        """
        results = {}
        for camera_id in self.camera_registry:
            result = self.capture_timed(camera_id, interval_seconds)
            if result:
                results[camera_id] = result
        return results

    def get_registered_cameras(self):
        """
        Returns a list of registered camera IDs.
        """
        print(f"[DEBUG] Registered Cameras: {list(self.camera_registry.keys())}")

        return list(self.camera_registry.keys())

    def convert_video_from_images(self, camera_id, fps):
        folder_path = os.path.join(self.base_save_dir, camera_id, self.scenario_start)

        # Generate next available index
        existing_videos = [
            f
            for f in os.listdir(folder_path)
            if f.startswith(f"{camera_id}_{self.scenario_start}_video")
            and f.endswith(".mp4")
        ]
        next_index = len(existing_videos) + 1
        output_filename = f"{camera_id}_{self.scenario_start}_video{next_index}.mp4"
        output_file = os.path.join(folder_path, output_filename)

        command = (
            f"ffmpeg -y -framerate {fps} -pattern_type glob -i '{os.path.join(folder_path, '*.jpg')}' "
            f"-c:v libx264 -pix_fmt yuv420p {output_file}"
        )

        print(f"[MAIN] Running FFMPEG command: {command}")

        subprocess.call(command, shell=True)

        print(f"[MAIN] Video conversion complete. Video saved at: {output_file}")

        # Delete original images and .json metadata
        for filename in os.listdir(folder_path):
            if filename.lower().endswith(".jpg"):
                file_path = os.path.join(folder_path, filename)
                os.remove(file_path)

                json_filename = filename.replace(".jpg", ".json")
                json_path = os.path.join(folder_path, json_filename)
                if os.path.exists(json_path):
                    os.remove(json_path)

    def capture_pointcloud(self, camera_id):
        """
        Capture pointcloud data from the specified camera.
        Parameters:
        camera_id (str): Identifier for the camera.
        Returns:
        tuple: A tuple containing point colors, normals, and pointcloud data.
        Returns None if capture fails.
        """
        print(f"[DEBUG] Attempting to capture pointcloud from {camera_id}")

        # Check if the camera exists
        if camera_id not in self.camera_registry:
            carb.log_error(f"Camera with ID {camera_id} not registered.")
            return None

        camera = self.camera_registry[camera_id]

        # Get the latest frame
        frame = camera.get_current_frame()

        # Check if frame has pointcloud data
        if (
            frame is None
            or "pointcloud" not in frame
            or frame["pointcloud"] is None
            or "data" not in frame["pointcloud"]
        ):
            carb.log_error(f"No valid pointcloud data from camera {camera_id}")
            return None

        # Extract pointcloud components
        pc = frame["pointcloud"]
        pointcloud_data = np.array(pc["data"])  # Shape: (N, 3)
        point_colors = np.array(pc["pointRgb"]).reshape(-1, 4)[:, :3]

        point_normals = frame["pointcloud"]["pointNormals"]  # Normal vectors

        return point_colors, point_normals, pointcloud_data

    def save_stereo_pointcloud_pair(self, pair_id, pair_name=None):
        """
        Capture both left/right, then save them together

        Returns dict with file paths or None on failure.
        """
        print(f"[MAIN] Capturing pointclouds from stereo pair {pair_id}")
        if pair_id not in self.stereo_pairs:
            carb.log_error(f"Stereo pair {pair_id} not registered.")
            return None

        left_id, right_id = (
            self.stereo_pairs[pair_id]["left"],
            self.stereo_pairs[pair_id]["right"],
        )

        # Grab raw data
        left_data = self.capture_pointcloud(left_id)
        right_data = self.capture_pointcloud(right_id)
        if left_data is None or right_data is None:
            carb.log_warn("One or both captures failed; aborting save.")
            return None

        # Timestamped folder
        ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dir_name = pair_name or f"{pair_id}_{ts}"
        out_dir = os.path.join(self.base_save_dir, "stereo_pairs", dir_name)
        os.makedirs(out_dir, exist_ok=True)

        # Save each
        left_cols, left_norms, left_pts = left_data
        right_cols, right_norms, right_pts = right_data

        left_xyzrgb = np.hstack([left_pts, left_cols])
        right_xyzrgb = np.hstack([right_pts, right_cols])

        fn_left = os.path.join(out_dir, f"{left_id}.npy")
        fn_right = os.path.join(out_dir, f"{right_id}.npy")

        try:
            np.save(fn_left, left_xyzrgb)
            np.save(fn_right, right_xyzrgb)
            print(f"[MAIN] Saved ponintclouds to {out_dir}")
            return {"left_npy": fn_left, "right_npy": fn_right, "pair_dir": out_dir}
        except Exception as e:
            carb.log_error(f"Error saving pointclouds: {e}")
            return None
