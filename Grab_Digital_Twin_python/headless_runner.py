from omni.isaac.core import World
from omni.isaac.dynamic_control import _dynamic_control
from .scenarios.udp_scenario import UDPScenario
from .robot.robot_controller import RobotController
from .scenes.setup_scene import setup_scene

import time
import omni.timeline


def main():
    print("Starting script.")

    world = World(
        physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0
    )

    setup_scene()

    from omni.isaac.core.utils.stage import get_current_stage

    stage = get_current_stage()
    for _ in range(200):
        robot_prim = stage.GetPrimAtPath("/Robot")
        if robot_prim.IsValid():
            print("[MAIN] /Robot loaded.")
            break
        print("[MAIN] Waiting for /Robot to appear in the stage...")
        time.sleep(0.05)
    else:
        print("[MAIN] /Robot never appeared — aborting.")
        return

    # Start timeline and step world multiple times for more initialization time
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    for _ in range(10):
        world.step(render=False)
        time.sleep(0.1)

    dc = _dynamic_control.acquire_dynamic_control_interface()

    articulation = None
    max_attempts = 20
    for attempt in range(max_attempts):
        articulation = dc.get_articulation("/Robot")

        if articulation is not None:
            print(f"[MAIN] Articulation handle acquired on attempt {attempt + 1}")
            break

        print(
            f"[MAIN] Attempt {attempt + 1}: Articulation not yet registered — retrying..."
        )
        time.sleep(0.5)
        world.step(render=False)

    if articulation is None:
        print(
            f"[MAIN] Failed to get articulation handle after {max_attempts} attempts. Aborting."
        )
        return

    print("[MAIN] Articulation handle is valid. Proceeding with scenario.")

    # Set up and run scenario
    robot_controller = RobotController()
    scenario = UDPScenario(robot_controller, world=world)
    scenario.setup()

    try:
        while True:
            scenario.update(1.0 / 60.0)
            world.step(render=False)
    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")
