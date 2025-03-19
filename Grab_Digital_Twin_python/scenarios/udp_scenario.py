import time
import omni.usd
from omni.isaac.core import World
from ..global_variables import AXIS1_JOINT_PATH, AXIS2_JOINT_PATH, AXIS3_JOINT_PATH, PICK_BOX_1
from ..networking.udp_controller import UDPController

class UDPControllerScenario:
    def __init__(self, robot_controller):
        self._robot_controller = robot_controller
        self._world = None
        self.last_udp_message = None
        self._did_run = False
        self._udp_thread = None
        self._robot_controller.refresh_handles()

        self.udp = UDPController()
        self.udp.callback = self._udp_callback

    def _udp_callback(self, message):
        self.last_udp_message = message

    def reset(self):
        self._did_run = False
        self.udp.stop()  
        if self._world is not None:
            self._world.reset()

    def start_udp_server(self, host="0.0.0.0", port=9999):
        self.udp.host = host
        self.udp.port = port
        self.udp.start()

    def parse_and_execute_command(self, message):
        if message.strip().lower() == "force_data":
            self._robot_controller.read_force_sensor_value()
            print("Read force sensor value")
            return

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
            self._robot_controller.set_angular_drive_target(AXIS1_JOINT_PATH, target_value)
            print(f"Set angular drive target for axis 1 to {target_value}")
        elif axis_id == 2:
            self._robot_controller.set_prismatic_joint_position(AXIS2_JOINT_PATH, target_value)
            print(f"Set prismatic joint position for axis 2 to {target_value}")
        elif axis_id == 3:
            self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, target_value)
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
    robot_controller = ... # dont need this probably
    scenario = UDPControllerScenario(robot_controller)
    scenario.setup()
    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
