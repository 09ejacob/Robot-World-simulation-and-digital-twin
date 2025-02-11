# scenario.py

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from .robot_controller import (
    open_gripper,
    close_gripper,
    set_angular_drive_target,
    set_prismatic_joint_position,
)
from .global_variables import AXIS1_JOINT_PATH, AXIS2_JOINT_PATH, PICK_BOX_PATH


class PickBoxScenario:
    """
    A minimal scenario class that runs a single sequence of robot actions.
    """

    def __init__(self):
        self._world = None
        self._did_run = False

    def setup(self):
        """
        Called once after 'Load' (or _setup_scenario in UIBuilder) to do any
        scenario-specific initialization.
        """
        self._world = World()
        self._world.reset()
        self._did_run = False

        pickBox = DynamicCuboid(
            prim_path=PICK_BOX_PATH,
            position=np.array((0, 2.3, 0.9)),
            scale=np.array((1, 1, 0.5)),
            color=np.array((2, 2, 2)),
        )

        self._scenario_generator = self._run_simulation()

    def reset(self):
        """
        Called whenever the user presses the RESET button.
        Put everything back into its default state so the simulation can be run again.
        """
        self._did_run = False
        if self._world is not None:
            self._world.reset()

    def update(self, step: float):
        try:
            next(self._scenario_generator)
            return False  # Scenario is still running
        except StopIteration:
            # Generator is exhausted; the simulation is complete.
            return True

    def _run_simulation(self):
        print("Starting robot action simulation...")
        self._world.play()

        for _ in range(60):
            self._world.step(render=True)
            yield
        print("Lowering prismatic joint...")
        set_prismatic_joint_position(AXIS2_JOINT_PATH, -1.4)
        for _ in range(60):
            self._world.step(render=True)
            yield

        print("Closing gripper...")
        close_gripper()
        for _ in range(50):
            self._world.step(render=True)
            yield

        print("Raising prismatic joint...")
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.8)
        for _ in range(60):
            self._world.step(render=True)
            yield

        print("Rotating angular joint...")
        set_angular_drive_target(AXIS1_JOINT_PATH, 180)
        for _ in range(160):
            self._world.step(render=True)
            yield

        print("Opening gripper...")
        open_gripper()
        for _ in range(60):
            self._world.step(render=True)
            yield

        print("Simulation complete. Stopping simulation.")
        self._world.stop()
