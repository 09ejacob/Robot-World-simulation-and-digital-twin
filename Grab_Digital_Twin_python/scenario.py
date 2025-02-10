# scenario.py

import numpy as np
from omni.isaac.core import World
from .robotController import (
    open_gripper,
    close_gripper,
    set_angular_drive_target,
    set_prismatic_joint_position,
)

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
        self._scenario_generator = self._run_simulation_snippet()

    def reset(self):
        """
        Called whenever the user presses the RESET button.
        Put everything back into its default state so the snippet can be run again.
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

    def _run_simulation_snippet(self):
        print("Starting robot action snippet...")
        self._world.play()

        for _ in range(60):
            self._world.step(render=True)
            yield  
        print("Lowering prismatic joint...")
        set_prismatic_joint_position("/World/Robot/Joints/PrismaticJointAxis2", -1.4)
        for _ in range(240):  # ~4 seconds
            self._world.step(render=True)
            yield 

        print("Closing gripper...")
        close_gripper()
        for _ in range(50):
            self._world.step(render=True)
            yield

        print("Raising prismatic joint...")
        set_prismatic_joint_position("/World/Robot/Joints/PrismaticJointAxis2", 0.8)
        for _ in range(240):
            self._world.step(render=True)
            yield

        print("Rotating angular joint...")
        set_angular_drive_target("/World/Robot/Joints/RevoluteJointAxis1", 180)
        for _ in range(240):
            self._world.step(render=True)
            yield

        print("Opening gripper...")
        open_gripper()
        for _ in range(240):
            self._world.step(render=True)
            yield

        print("Snippet complete. Stopping simulation.")
        self._world.stop()

