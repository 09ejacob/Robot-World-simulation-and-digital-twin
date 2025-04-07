import time
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.timeline
import omni.physx as _physx
from pxr import UsdPhysics

from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage

from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from Grab_Digital_Twin_python.robot.robot_controller import RobotController
from Grab_Digital_Twin_python.scenarios.udp_scenario import UDPScenario
from Grab_Digital_Twin_python.global_variables import PHYSICS_SCENE_PATH, ROBOT_PATH


def main():
    print("Starting UDP scenario in headless mode.")

    print("Creating stage...")
    create_new_stage()

    print("Setting up Scene...")
    setup_scene()

    print("[MAIN] Forcing update loop to finalize stage...")
    for _ in range(30):
        simulation_app.update()
        time.sleep(0.05)

    stage = get_current_stage()
    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print("[MAIN] Physics scene not found in stage")
        return

    print("[MAIN] Giving physics a moment to settle...")
    for _ in range(20):
        simulation_app.update()
        time.sleep(0.05)

    physx_iface = _physx.acquire_physx_interface()
    print(f"[DEBUG] PhysX interface acquired: {physx_iface is not None}")

    print("Creating World...")
    world = World(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
    world.reset()

    for _ in range(5):
        world.step(render=False)
        time.sleep(0.1)

    print("[MAIN] Physics context initialized.")

    timeline_iface = omni.timeline.get_timeline_interface()
    timeline_iface.set_auto_update(False)

    root = stage.GetPseudoRoot()
    print("[DEBUG] Stage children:")
    for child in root.GetChildren():
        print(f" - {child.GetPath()}")

    for i in range(200):
        robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
        if robot_prim.IsValid():
            print("[MAIN] /Robot loaded.")
            break
        if i % 10 == 0:
            print(f"[MAIN] Waiting for /Robot to appear... (attempt {i})")
        time.sleep(0.05)
    else:
        print("[MAIN] /Robot never appeared — aborting.")
        return

    timeline_iface.play()

    for _ in range(50):
        world.step(render=False)
        time.sleep(0.1)

    robot_controller = RobotController()
    robot_controller.refresh_handles()

    if not robot_controller.articulation:
        print("[MAIN] Articulation handle is still invalid. Something is wrong.")
        return

    scenario = UDPScenario(robot_controller=robot_controller, world=world, print_positions=True, print_performance_stats=True)
    scenario.setup()


    for _ in range(10):
        world.step(render=False)
        time.sleep(0.1)

    print("[MAIN] Scenario is set up. Entering main loop...")

    try:
        while True:
            #print("[LOOP] Simulation loop is running...")
            scenario.update()
            world.step(render=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")

    simulation_app.close()


if __name__ == "__main__":
    main()
