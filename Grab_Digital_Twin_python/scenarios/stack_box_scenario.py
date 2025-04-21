import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    PICK_BOX_1,
)


class StackBoxScenario:
    """
    A scenario where the robot picks up multiple boxes and stacks them.
    """

    def __init__(self, robot_controller, allow_udp_capture):
        self._robot_controller = robot_controller
        self._world = None
        self._did_run = False

    def setup(self):
        self._world = World()
        self._world.reset()
        self._did_run = False
        self._robot_controller.refresh_handles()

        self.box1 = DynamicCuboid(
            prim_path=f"{PICK_BOX_1}_1",
            position=np.array((1.5, 0, 0.5)),
            scale=np.array((0.3, 0.3, 0.3)),
            color=np.array((0.1, 0.2, 0.9)),
        )

        self.box2 = DynamicCuboid(
            prim_path=f"{PICK_BOX_1}_2",
            position=np.array((1.5, 0, 1.0)),
            scale=np.array((0.3, 0.3, 0.3)),
            color=np.array((0.9, 0.2, 0.1)),
        )

        self._scenario_generator = self._run_simulation()

    def unload(self):
        """Resets the simulation and unloads the scenario-specific prims from the stage."""
        self._did_run = False
        if self._world is not None:
            self._world.reset()

        # Remove the prims created in setup
        stage = omni.usd.get_context().get_stage()
        for prim_path in [f"{PICK_BOX_1}_1", f"{PICK_BOX_1}_2"]:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                stage.RemovePrim(prim_path)
                print(f"Removed prim at: {prim_path}")

    def update(self, step: float):
        try:
            next(self._scenario_generator)
            return False  # Scenario is still running
        except StopIteration:
            return True  # Scenario finished

    def _run_simulation(self):
        axis2_dof_index = self._robot_controller.get_dof_index_for_joint(
            AXIS2_JOINT_PATH
        )
        axis1_dof_index = self._robot_controller.get_dof_index_for_joint(
            AXIS1_JOINT_PATH
        )
        axis3_dof_index = self._robot_controller.get_dof_index_for_joint(
            AXIS3_JOINT_PATH
        )

        # Simulate stacking
        for i, _ in enumerate([self.box1, self.box2]):
            self._robot_controller.set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.6)
            yield from self._robot_controller.wait_for_joint_position(
                axis2_dof_index,
                target_position=0.6,
                pos_threshold=0.1,
            )

            # Rotate 90
            self._robot_controller.set_angular_drive_target(AXIS1_JOINT_PATH, 90)
            yield from self._robot_controller.wait_for_joint_position(
                axis1_dof_index,
                target_position=-90,
                pos_threshold=0.2,
                is_angular=True,
            )

            # Move snake out
            self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, 1.45)
            yield from self._robot_controller.wait_for_joint_position(
                axis3_dof_index,
                target_position=1.45,
                pos_threshold=0.01,
            )

            # Lower
            if i == 1:  # For box2 (second iteration)
                # Lower more for box2
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.05
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.05,
                    pos_threshold=0.04,
                )
            else:
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.34
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.34,
                    pos_threshold=0.04,
                )

            # Close gripper
            self._robot_controller.close_gripper()

            # Raise
            if i == 1:  # For box2, raise more
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 1.0
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=1.0,
                    pos_threshold=0.1,
                )
            else:
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.8
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.8,
                    pos_threshold=0.1,
                )

            # Move snake in
            self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
            yield from self._robot_controller.wait_for_joint_position(
                axis3_dof_index,
                target_position=0,
                pos_threshold=0.1,
            )

            # Rotate 90 back to 0
            self._robot_controller.set_angular_drive_target(AXIS1_JOINT_PATH, 0)
            yield from self._robot_controller.wait_for_joint_position(
                axis1_dof_index,
                target_position=0,
                pos_threshold=0.1,
                is_angular=True,
            )

            # Move snake out
            self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, 1)
            yield from self._robot_controller.wait_for_joint_position(
                axis3_dof_index,
                target_position=1,
                pos_threshold=0.01,
            )

            # Lower
            if i == 1:  # For box2, raise more
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.9
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.9,
                    pos_threshold=0.01,
                )
            else:
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.58
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.58,
                    pos_threshold=0.01,
                )

            # Open gripper
            self._robot_controller.open_gripper()

            # Raise
            if i == 1:  # For box2, raise more
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 1.1
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=1.1,
                    pos_threshold=0.1,
                )
            else:
                self._robot_controller.set_prismatic_joint_position(
                    AXIS2_JOINT_PATH, 0.8
                )
                yield from self._robot_controller.wait_for_joint_position(
                    axis2_dof_index,
                    target_position=0.8,
                    pos_threshold=0.1,
                )

            # Move snake in
            self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
            yield from self._robot_controller.wait_for_joint_position(
                axis3_dof_index,
                target_position=0,
                pos_threshold=0.1,
            )

        # Move axis 2 to resting position
        self._robot_controller.set_prismatic_joint_position(AXIS2_JOINT_PATH, 0)
        yield from self._robot_controller.wait_for_joint_position(
            axis2_dof_index,
            target_position=0,
            pos_threshold=0.0185,
        )

        for _ in range(180):
            self._world.step(render=True)
            yield

        print("Stacking scenario complete.")
        self._world.stop()
