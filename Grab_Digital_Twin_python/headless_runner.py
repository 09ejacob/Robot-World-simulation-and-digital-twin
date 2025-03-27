import time
import omni.timeline
from pxr import UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage
from omni.usd import get_context
from omni.isaac.core.simulation_context import SimulationContext
import omni.physx as _physx

from .scenes.setup_scene import setup_scene
from .robot.robot_controller import RobotController
from .scenarios.udp_scenario import UDPScenario

from .global_variables import (
    PHYSICS_SCENE_PATH,
    ROBOT_PATH,
)


def main():
    print("Starting UDP scenario in headless mode.")

    print("Creating stage...")
    create_new_stage()

    print("Setting up Scene...")
    setup_scene()

    print("[MAIN] Forcing update loop to finalize stage...")
    for _ in range(10):
        omni.kit.app.get_app().update()
        time.sleep(0.05)

    stage = get_current_stage()
    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print("[MAIN] Physics scene not found in stage")
        return

    print("[MAIN] Giving physics a moment to settle...")
    for _ in range(20):
        omni.kit.app.get_app().update()
        time.sleep(0.05)

    physx_iface = _physx.acquire_physx_interface()
    print(f"[DEBUG] PhysX interface acquired: {physx_iface is not None}")

    print("Creating SimulationContext...")
    simulation_context = SimulationContext(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
    simulation_context.initialize_physics()
    simulation_context.play()

    for _ in range(5):
        simulation_context.step(render=False)
        time.sleep(0.1)  # optional, to give things a little breathing room

    print("[MAIN] Physics context initialized.")

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
        simulation_context.step(render=False)
        time.sleep(0.1)

    robot_controller = RobotController()
    robot_controller.refresh_handles()

    if not robot_controller.articulation:
        print("[MAIN] Articulation handle is STILL invalid. Something is wrong.")
        return

    scenario = UDPScenario(robot_controller=robot_controller, world=simulation_context)
    scenario.setup()

    print("[MAIN] Scenario is set up. Entering main loop...")

    try:
        while True:
            scenario.update(1.0 / 60.0)
            simulation_context.step(render=False)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")
