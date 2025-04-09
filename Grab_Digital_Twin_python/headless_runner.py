import time
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.timeline
import omni.physx as _physx

from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage

from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from Grab_Digital_Twin_python.robot.robot_controller import RobotController
from Grab_Digital_Twin_python.scenarios.udp_scenario import UDPScenario
from Grab_Digital_Twin_python.global_variables import PHYSICS_SCENE_PATH, ROBOT_PATH


def wait_for_condition(condition_fn, timeout=5.0, update_fn=None):
    start_time = time.time()
    while not condition_fn():
        if update_fn is not None:
            update_fn()
        if time.time() - start_time > timeout:
            break


def main():
    print("Starting UDP scenario in headless mode.")

    print("Creating stage...")
    create_new_stage()

    print("Setting up Scene...")
    setup_scene()

    wait_for_condition(
        lambda: get_current_stage().GetRootLayer() is not None,
        timeout=5.0,
        update_fn=simulation_app.update,
    )

    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print("[MAIN] Physics scene not found in stage")
        return

    wait_for_condition(lambda: False, timeout=1.0, update_fn=simulation_app.update)

    physx_iface = _physx.acquire_physx_interface()
    print(f"[DEBUG] PhysX interface acquired: {physx_iface is not None}")

    print("Creating World...")
    world = World(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
    world.reset()

    for _ in range(5):
        world.step(render=False)

    print("[MAIN] Physics context initialized.")

    timeline_iface = omni.timeline.get_timeline_interface()
    timeline_iface.set_auto_update(False)

    print("[DEBUG] Stage children:")
    for child in stage.GetPseudoRoot().GetChildren():
        print(f" - {child.GetPath()}")

    wait_for_condition(
        lambda: stage.GetPrimAtPath(ROBOT_PATH).IsValid(),
        timeout=5.0,
        update_fn=simulation_app.update,
    )
    if not stage.GetPrimAtPath(ROBOT_PATH).IsValid():
        print("[MAIN] /Robot never appeared — aborting.")
        return
    else:
        print("[MAIN] /Robot loaded.")

    timeline_iface.play()

    for _ in range(10):
        world.step(render=False)

    robot_controller = RobotController()
    robot_controller.refresh_handles()

    if not robot_controller.articulation:
        print("[MAIN] Articulation handle is still invalid. Something is wrong.")
        return

    scenario = UDPScenario(
        robot_controller=robot_controller,
        world=world,
        print_positions=True,
        print_performance_stats=True,
    )
    scenario.setup()

    for _ in range(10):
        world.step(render=False)

    print("[MAIN] Scenario is set up. Entering main loop...")

    try:
        while True:
            scenario.update()
            for _ in range(10):
                world.step(render=False)
            world.step(render=True)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")

    simulation_app.close()


if __name__ == "__main__":
    main()
