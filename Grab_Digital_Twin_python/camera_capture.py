import os
import subprocess
import time
from datetime import datetime, timezone
from PIL import Image
import numpy as np
import cv2
import json
import struct


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

    def capture_image_array(self, camera_id):
        if camera_id not in self.camera_registry:
            print(f"[ERROR] Camera {camera_id} not registered.")
            return None

        frame = self.camera_registry[camera_id].get_current_frame()
        if not frame or "rgba" not in frame:
            print(f"[ERROR] No 'rgba' frame for {camera_id}")
            return None

        rgba = frame["rgba"]
        if rgba is None or rgba.size == 0:
            print(f"[ERROR] Empty RGBA data from {camera_id}")
            return None

        rgb_array = rgba[:, :, :3]
        if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
            print(f"[ERROR] Unexpected image shape {rgb_array.shape} from {camera_id}")
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
            with open(metadata_path, "w") as f:
                json.dump(metadata, f, indent=4)

            return image_path

        except Exception as e:
            print(f"[ERROR] Failed to save image for {camera_id}: {e}")
            return None

    def capture_image(self, camera_id):
        rgb_array = self.capture_image_array(camera_id)
        if rgb_array is None:
            return None
        return self._save_image(camera_id, rgb_array)

    def capture_and_stream(self, camera_id, udp_controller, host, port):
        rgb_array = self.capture_image_array(camera_id)
        if rgb_array is None:
            print(f"[ERROR] Could not capture image from {camera_id}")
            return None

        save_path = self._save_image(camera_id, rgb_array)

        try:
            success, jpeg_data = cv2.imencode(".jpg", rgb_array)
            if not success:
                print(f"[ERROR] Failed to compress image from {camera_id}")
                return save_path

            _, metadata = self._generate_capture_metadata(camera_id, rgb_array)
            metadata_bytes = json.dumps(metadata).encode("utf-8")
            metadata_length = struct.pack("!I", len(metadata_bytes))
            packet = metadata_length + metadata_bytes + jpeg_data.tobytes()

            udp_controller.send(packet, host, port)

        except Exception as e:
            print(f"[ERROR] Streaming image failed for {camera_id}: {e}")

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

    def camera_registry(self):
        return self._camera_registry

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
