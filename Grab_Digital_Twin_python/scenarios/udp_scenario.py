import time
import queue
import threading
import omni.usd
import numpy as np
import psutil
from omni.isaac.core import World
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    PICK_BOX_1,
)
from ..networking.udp_controller import UDPController
from omni.isaac.core.objects import DynamicCuboid


class UDPScenario:
    def __init__(self, robot_controller, world=None, enable_stats=False):
        self._robot_controller = robot_controller
        self._world = world  # use world passed in (from headless_runner)
        self._did_run = False
        self.command_queue = queue.Queue()

        # Performance tracking
        self.enable_stats = enable_stats
        self.udp_message_count = 0
        self.executed_command_count = 0
        self.last_time_check = time.time()

        # Initialize the UDP server
        self.udp = UDPController()
        self.udp.callback = self._udp_callback

        self.last_box_print_time = time.time()

    def _udp_callback(self, message):
        """Receives UDP messages and stores them in a queue."""
        self.command_queue.put(message)
        self.udp_message_count += 1  #

    def reset(self):
        """Resets the scenario and stops the UDP server."""
        self._did_run = False
        self.udp.stop()
        if self._world is not None:
            self._world.reset()

    def start_udp_server(self, host="0.0.0.0", port=9999):
        """Starts the UDP server if the port is not already in use."""

        if self.port_in_use(port):
            print(f"Port {port} is already in use. Skipping new server.")
            return

        self.udp.host = host
        self.udp.port = port
        self.udp.start()

    def port_in_use(self, port):
        """Checks if the given port is already in use."""
        for conn in psutil.net_connections(kind="udp"):
            if conn.laddr.port == port:
                return True
        return False

    def parse_and_execute_command(self, message):
        """Processes and executes commands from the queue."""
        self.executed_command_count += 1  #

        if message.strip().lower() == "force_data":
            print("Read force sensor value")
            self._robot_controller.read_force_sensor_value()
            return

        if message.strip().lower() == "close_gripper":
            print("Close gripper")
            self._robot_controller.close_gripper()
            return

        if message.strip().lower() == "open_gripper":
            print("Open gripper")
            self._robot_controller.open_gripper()
            return

        if message.strip().lower() == "capture":
            print("[UDP] Capturing images from all cameras...")
            result = self._robot_controller.capture_from_all_cameras()
            for cam_id, path in result.items():
                print(f"Captured from {cam_id}: {path}")
            return

        parts = message.split(":")
        if len(parts) != 3:
            print("[ERROR] Invalid command format:", message)
            return

        command, axis_str, value_str = parts
        if command.lower() != "axis":
            print("[ERROR] Unknown command type:", command)
            return

        try:
            axis_id = int(axis_str)
            target_value = float(value_str)
        except ValueError:
            print("[ERROR] Invalid axis id or target value:", message)
            return

        if axis_id == 1:
            self._robot_controller.set_angular_drive_target(
                AXIS1_JOINT_PATH, target_value
            )
            print(f"Set angular drive target for axis 1 to {target_value}")
        elif axis_id == 2:
            self._robot_controller.set_prismatic_joint_position(
                AXIS2_JOINT_PATH, target_value
            )
            print(f"Set prismatic joint position for axis 2 to {target_value}")
        elif axis_id == 3:
            self._robot_controller.set_prismatic_joint_position(
                AXIS3_JOINT_PATH, target_value
            )
            print(f"Set prismatic joint position for axis 3 to {target_value}")
        else:
            print("[ERROR] Axis id not recognized:", axis_id)

    def random_color(self):
        return np.random.rand(3)

    def create_boxes(self, num_boxes: int):
        boxes = []
        start_x = 1.25
        x_inc = 0.3
        row_y = {0: -0.2, 1: 0.2}
        base_z = 0.3
        z_inc = 0.2

        for i in range(num_boxes):
            layer = i // 8
            index_in_layer = i % 8
            column = index_in_layer // 2
            row = index_in_layer % 2

            x = start_x + column * x_inc
            y = row_y[row]
            z = base_z + layer * z_inc

            prim_path = f"/World/Environment/box{i + 1}"
            box = DynamicCuboid(
                prim_path=prim_path,
                position=np.array((x, y, z)),
                scale=np.array((0.3, 0.4, 0.2)),
                color=self.random_color(),
            )
            boxes.append(box)

        if boxes:
            self.box = boxes[0]

        return boxes

    def setup(self):
        # Use existing world or create if not passed in (fallback for GUI/debug)
        if self._world is None:
            self._world = World()
        self._world.reset()

        self._robot_controller.refresh_handles()

        # Cache DOF indices
        self.axis1_dof = self._robot_controller.get_dof_index_for_joint(
            AXIS1_JOINT_PATH
        )
        self.axis2_dof = self._robot_controller.get_dof_index_for_joint(
            AXIS2_JOINT_PATH
        )
        self.axis3_dof = self._robot_controller.get_dof_index_for_joint(
            AXIS3_JOINT_PATH
        )

        # Environment objects
        self.pallet = DynamicCuboid(
            prim_path=f"/World/Environment/pallet",
            position=np.array((1.7, 0, 0.1)),
            scale=np.array((1.2, 0.8, 0.144)),
            color=np.array((0.2, 0.08, 0.05)),
        )

        self.create_boxes(1)
        self.start_udp_server()

    def update(self, step: float = 0.1):
        """Runs in the Isaac Sim main thread and processes queued UDP commands."""
        start_time = time.time()

        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.parse_and_execute_command(message)

        # Print performance stats every 1 second
        if self.enable_stats and (start_time - self.last_time_check >= 1.0):
            print(
                f"[STATS] UDP Received: {self.udp_message_count} msg/sec | Executed: {self.executed_command_count} cmd/sec"
            )
            self.udp_message_count = 0
            self.executed_command_count = 0
            self.last_time_check = start_time

        if hasattr(self, "box") and (start_time - self.last_box_print_time >= 1.0):
            pos, _ = self.box.get_world_pose()
            print(f"Box position: {pos}")

            self._robot_controller.print_joint_position_by_index(
                self.axis1_dof, is_angular=True
            )
            self._robot_controller.print_joint_position_by_index(
                self.axis2_dof, is_angular=False
            )
            self._robot_controller.print_joint_position_by_index(
                self.axis3_dof, is_angular=False
            )

            self.last_box_print_time = start_time


if __name__ == "__main__":
    robot_controller = ...
    scenario = UDPScenario(robot_controller)
    scenario.setup()

    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
