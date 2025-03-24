from omni.isaac.core import World
from .scenarios.udp_scenario import UDPScenario
from .robot.robot_controller import RobotController
from .scenes.setup_scene import load_grab_usd
import carb
import sys
import omni.usd
import time
import carb


def main():
    print("Starting script.")

    print("Finished headless sim.")
