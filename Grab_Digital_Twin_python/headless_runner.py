from omni.isaac.core import World
from .scenarios.udp_scenario import UDPScenario
from .robot.robot_controller import RobotController
from .scenes.setup_scene import setup_scene
import carb
import sys
import omni.usd
import time
import carb
from isaacsim.core.utils.stage import create_new_stage


def main():
    print("Starting script.")

    create_new_stage()

    world = World(
        physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0
    )

    setup_scene()

    robot_controller = RobotController()
    scenario = UDPScenario(robot_controller)
    scenario.setup()

    try:
        while True:
            scenario.update(1.0 / 60.0)
            world.step(render=False)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")
