import time
from omni.isaac.core import World
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    PICK_BOX_1)
from ..networking .UDP_controller import UDPController

class UDPControllerScenario:
    def __init__(self, robot_controller):
        self._robot_controller = robot_controller
        self._world = World()
        self._robot_controller.refresh_handles()

        # Minimal attributes to satisfy UI integration:
        self._did_run = False
        self._timeline = None
        self.ui_builder = self

        # Create the UDP controller and set its callback.
        self.udp = UDPController()
        self.udp.callback = self.parse_and_execute_command

    def reset(self):
        self._did_run = False
        if self._world:
            self._world.reset()

    def parse_and_execute_command(self, message):
        parts = message.split(":")
        if len(parts) != 3 or parts[0].lower() != "axis":
            print("Invalid command format:", message)
            return
        try:
            axis_id = int(parts[1])
            target_value = float(parts[2])
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
        self._world.reset()
        self.udp.start()

    # Minimal update method to support timeline callbacks.
    def update(self, step: float = 0.1):
        self._did_run = True
        time.sleep(step)
        return False

if __name__ == "__main__":
    # Provide your actual robot_controller instance.
    robot_controller = ...  # Replace with your robot controller initialization.
    scenario = UDPControllerScenario(robot_controller)
    scenario.setup()
    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
