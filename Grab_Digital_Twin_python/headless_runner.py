import time
import argparse
import carb


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
    parser.add_argument(
        "--enable_3d_features",
        action="store_true",
        help="Enable 3D features (depth, pointcloud) on cameras if cameras are enabled",
    )
    parser.add_argument(
        "--enable_overview_camera",
        action="store_true",
        help="Enable the overview camera if cameras are enabled",
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
from isaacsim.core.utils.stage import create_new_stage, get_current_stage

from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from Grab_Digital_Twin_python.robot.robot_controller import RobotController
from Grab_Digital_Twin_python.scenarios.udp_scenario import UDPScenario
from Grab_Digital_Twin_python.global_variables import PHYSICS_SCENE_PATH, ROBOT_PATH


def _wait_for_condition(condition_fn, timeout=5.0, update_fn=None):
    """Wait until "condition_fn()" returns True or "timeout" seconds elaps."""
    start_time = time.time()
    while not condition_fn():
        if update_fn is not None:
            update_fn()
        if time.time() - start_time > timeout:
            break


def _main():
    """Entrypoint for running the UDP scenario in headless mode."""
    physics_dt = 1.0 / args.physics_fps
    rendering_dt = 1.0 / args.rendering_fps

    print(
        f"[MAIN] Starting UDP scenario in headless mode "
        f"(physics_fps={args.physics_fps}, rendering_fps={args.rendering_fps}, "
        f"physics_dt={physics_dt:.5f}, rendering_dt={rendering_dt:.5f})"
    )
    print("[MAIN] Creating stage...")
    create_new_stage()

    print("[MAIN] Setting up Scene...")
    if args.disable_cameras and (
        args.enable_3d_features or args.enable_overview_camera
    ):
        carb.log_warn(
            "[ARGS] Cameras are disabled (--disable_cameras), so '--enable_3d_features' and '--enable_overview_camera' will have no effect."
        )
    setup_scene(
        grab_usd=args.grab_usd,
        enable_cameras=not args.disable_cameras,
        enable_3d_features=args.enable_3d_features,
        enable_overview_camera=args.enable_overview_camera,
    )

    _wait_for_condition(
        lambda: get_current_stage().GetRootLayer() is not None,
        timeout=5.0,
        update_fn=simulation_app.update,
    )

    stage = get_current_stage()

    if not stage.GetPrimAtPath(PHYSICS_SCENE_PATH).IsValid():
        carb.log_error("Physics scene not found in stage")
        return

    _wait_for_condition(lambda: False, timeout=1.0, update_fn=simulation_app.update)

    physx_iface = _physx.acquire_physx_interface()
    print(f"[DEBUG] PhysX interface acquired: {physx_iface is not None}")

    print("[MAIN] Creating World...")
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt)
    world.reset()

    for _ in range(5):
        world.step(render=False)

    print("[MAIN] Physics context initialized.")

    timeline_iface = omni.timeline.get_timeline_interface()
    timeline_iface.set_auto_update(False)

    print("[DEBUG] Stage children:")
    for child in stage.GetPseudoRoot().GetChildren():
        print(f"[DEBUG]  - {child.GetPath()}")

    _wait_for_condition(
        lambda: stage.GetPrimAtPath(ROBOT_PATH).IsValid(),
        timeout=5.0,
        update_fn=simulation_app.update,
    )
    if not stage.GetPrimAtPath(ROBOT_PATH).IsValid():
        carb.log_error("/Robot never appeared — aborting.")
        return
    else:
        print("[MAIN] /Robot loaded.")

    timeline_iface.play()

    for _ in range(10):
        world.step(render=False)

    robot_controller = RobotController()
    robot_controller.refresh_handles()

    if not robot_controller.articulation:
        carb.log_error("Articulation handle is still invalid. Something is wrong.")
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
        print("[MAIN] Exiting simulation.")

    simulation_app.close()


if __name__ == "__main__":
    _main()
