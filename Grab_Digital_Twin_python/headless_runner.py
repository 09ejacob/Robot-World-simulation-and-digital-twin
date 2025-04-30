import time
import argparse


# Argument parsing happens before importing `SimulationApp` to allow `--help` to work.
def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rendering_fps",
        type=float,
        default=60.0,
        help="Rendering frames per second (e.g., 30, 60)",
    )
    parser.add_argument(
        "--physics_fps", type=float, default=60.0, help="Physics frames per second"
    )
    parser.add_argument(
        "--disable_cameras",
        action="store_true",
        help="Disable camera setup and UDP capture functionality",
    )
    parser.add_argument(
        "--print_positions",
        action="store_true",
        help="Print joint positions and position of robot when teleporting",
    )
    parser.add_argument(
        "--print_performance_stats",
        action="store_true",
        help="Print UDP and command execution stats",
    )
    parser.add_argument(
        "--grab_usd",
        type=str,
        default="Grab.usd",
        choices=["Grab.usd", "Grab-bottlegripper.usd"],
        help="USD file to load for the Grab robot model",
    )
    return parser.parse_args()


args = _parse_args()

# Delayed imports: SimulationApp must not start before argument parsing
from omni.isaac.kit import SimulationApp

# Must init SimulationApp before importing Isaac modules
simulation_app = SimulationApp({"headless": True})

import omni.timeline
import omni.physx as _physx

from omni.isaac.core import World
from omni.isaac.core.utils.stage import create_new_stage, get_current_stage

from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from Grab_Digital_Twin_python.robot.robot_controller import RobotController
from Grab_Digital_Twin_python.scenarios.udp_scenario import UDPScenario
from Grab_Digital_Twin_python.global_variables import PHYSICS_SCENE_PATH, ROBOT_PATH


def _wait_for_condition(condition_fn, timeout=5.0, update_fn=None):
    start_time = time.time()
    while not condition_fn():
        if update_fn is not None:
            update_fn()
        if time.time() - start_time > timeout:
            break


def _main():
    physics_dt = 1.0 / args.physics_fps
    rendering_dt = 1.0 / args.rendering_fps

    print(
        f"Starting UDP scenario in headless mode "
        f"(physics_fps={args.physics_fps}, rendering_fps={args.rendering_fps}, "
        f"physics_dt={physics_dt:.5f}, rendering_dt={rendering_dt:.5f})"
    )
    print("Creating stage...")
    create_new_stage()

    print("Setting up Scene...")
    setup_scene(
        enable_cameras=not args.disable_cameras, grab_usd=args.grab_usd
    )  # Need a flag for setting up scene with specifyed usd model.

    _wait_for_condition(
        lambda: get_current_stage().GetRootLayer() is not None,
        timeout=5.0,
        update_fn=simulation_app.update,
    )

    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        print("[MAIN] Physics scene not found in stage")
        return

    _wait_for_condition(lambda: False, timeout=1.0, update_fn=simulation_app.update)

    physx_iface = _physx.acquire_physx_interface()
    print(f"[DEBUG] PhysX interface acquired: {physx_iface is not None}")

    print("Creating World...")
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
    world.reset()

    for _ in range(5):
        world.step(render=False)

    print("[MAIN] Physics context initialized.")

    timeline_iface = omni.timeline.get_timeline_interface()
    timeline_iface.set_auto_update(False)

    print("[DEBUG] Stage children:")
    for child in stage.GetPseudoRoot().GetChildren():
        print(f" - {child.GetPath()}")

    _wait_for_condition(
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
        print_positions=args.print_positions,
        print_performance_stats=args.print_performance_stats,
        allow_udp_capture=not args.disable_cameras,
    )
    scenario.setup()

    for _ in range(10):
        world.step(render=False)

    print("[MAIN] Scenario is set up. Entering main loop...")

    try:
        while True:
            scenario.update()
            for _ in range(10):
                scenario._world.step(render=False)
            if not args.disable_cameras:
                scenario._world.step(render=True)

    except KeyboardInterrupt:
        print("Exiting headless UDP scenario.")

    simulation_app.close()


if __name__ == "__main__":
    _main()
