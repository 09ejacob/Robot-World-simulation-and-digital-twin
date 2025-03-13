import socket
import threading
import time
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.dynamic_control import _dynamic_control
from ..robot.robot_controller import (
    get_dof_index_for_joint_prim_path,
    open_gripper,
    close_gripper,
    set_angular_drive_target,
    set_prismatic_joint_position,
    wait_for_joint_position,
)
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    PICK_BOX_1,
)


class UDPControllerScenario:
    def __init__(self):
        self._world = None
        self.last_udp_message = None
        self._did_run = False
        self._udp_thread = None

    def reset(self):
        """
        Called when the user presses the RESET button.
        Resets the simulation.
        """
        self._did_run = False
        if self._world is not None:
            self._world.reset()

    def start_udp_server(self, host="0.0.0.0", port=9999):
        if self._udp_thread is not None and self._udp_thread.is_alive():
            print("[UDP Server] Already running.")
            return

        def udp_server():
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            try:
                udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception as e:
                print("SO_REUSEPORT not available:", e)
            try:
                udp_sock.bind((host, port))
            except Exception as e:
                print(f"[UDP Server] Bind exception: {e}")
                return

            print(f"[UDP Server] Listening on {host}:{port}")
            while True:
                try:
                    data, addr = udp_sock.recvfrom(1024)
                    message = data.decode("utf-8").strip()
                    print(f"[UDP Server] Received from {addr}: {message}")
                    self.last_udp_message = message
                except Exception as e:
                    print(f"[UDP Server] Exception: {e}")

        self._udp_thread = threading.Thread(target=udp_server, daemon=True)
        self._udp_thread.start()

    def parse_and_execute_command(self, message):
        """
        Parse command in the form "axis:<axis_id>:<target_value>"
        and calls control robot function.
        """
        parts = message.split(":")
        if len(parts) != 3:
            print("Invalid command format:", message)
            return
        command, axis_str, value_str = parts
        if command.lower() != "axis":
            print("Unknown command type:", command)
            return
        try:
            axis_id = int(axis_str)
            target_value = float(value_str)
        except ValueError:
            print("Invalid axis id or target value in command:", message)
            return

        if axis_id == 1:
            set_angular_drive_target(AXIS1_JOINT_PATH, target_value)
            print(f"Set angular drive target for axis 1 to {target_value}")
        elif axis_id == 2:
            set_prismatic_joint_position(AXIS2_JOINT_PATH, target_value)
            print(f"Set prismatic joint position for axis 2 to {target_value}")
        elif axis_id == 3:
            set_prismatic_joint_position(AXIS3_JOINT_PATH, target_value)
            print(f"Set prismatic joint position for axis 3 to {target_value}")
        else:
            print("Axis id not recognized:", axis_id)

    def setup(self):
        self._world = World()
        self._world.reset()
        self.start_udp_server()

    def update(self, step: float = 0.1):
        if self.last_udp_message:
            print(f"[Scenario] UDP Message: {self.last_udp_message}")
            self.parse_and_execute_command(self.last_udp_message)
            self.last_udp_message = None
        time.sleep(step)


if __name__ == "__main__":
    scenario = UDPControllerScenario()
    scenario.setup()
    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
