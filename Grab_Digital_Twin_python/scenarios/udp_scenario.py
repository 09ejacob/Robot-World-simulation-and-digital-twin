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
    def __init__(self, robot_controller, enable_stats=False):
        self._robot_controller = robot_controller
        self._world = None
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

    def setup(self):
        """Initializes the world and starts the UDP server."""
        self._world = World()
        self._world.reset()

        self.box1 = DynamicCuboid(
            prim_path=f"/World/cube",
            position=np.array((1.5, 0, 0.2)),
            scale=np.array((0.3, 0.3, 0.3)),
            color=np.array((0.1, 0.2, 0.9)),
        )

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
            self.udp_message_count = 0  # Reset counters
            self.executed_command_count = 0
            self.last_time_check = start_time  # Reset time tracking


if __name__ == "__main__":
    robot_controller = ...
    scenario = UDPScenario(robot_controller)
    scenario.setup()

    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
