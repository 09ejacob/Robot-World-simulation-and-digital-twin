import time
import omni.timeline
from pxr import UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage
from omni.usd import get_context

from .scenes.setup_scene import setup_scene
from .robot.robot_controller import RobotController
from .scenarios.udp_scenario import UDPScenario

from .global_variables import (
    PHYSICS_SCENE_PATH,
    ROBOT_PATH,
)


def main():
    print("Starting UDP scenario in headless mode.")

    create_new_stage()

    setup_scene()

    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        UsdPhysics.Scene.Define(stage, PHYSICS_SCENE_PATH)

    world = World(
        physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0
    )

    timeline_iface = omni.timeline.get_timeline_interface()
    timeline_iface.set_auto_update(False)

    for _ in range(200):
        robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
        if robot_prim.IsValid():
            print("[MAIN] /Robot loaded.")
            break
        print("[MAIN] Waiting for /Robot to appear in the stage...")
        time.sleep(0.05)
    else:
        print("[MAIN] /Robot never appeared — aborting.")
        return

    timeline_iface.play()

    for _ in range(10):
        world.step(render=False)
        time.sleep(0.1)

    robot_controller = RobotController()
    robot_controller.refresh_handles()

    if not robot_controller.articulation:
        print("[MAIN] Articulation handle is STILL invalid. Something is wrong.")
        return

    scenario = UDPScenario(robot_controller=robot_controller, world=world)
    scenario.setup()

    print("[MAIN] Scenario is set up. Entering main loop...")

    try:
        while True:
            scenario.update(1.0 / 60.0)
            world.step(render=False)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")
