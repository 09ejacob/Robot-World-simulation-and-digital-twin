import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.dynamic_control import _dynamic_control
from ..robot_controller import (
    get_dof_index_for_joint_prim_path,
    open_gripper,
    close_gripper,
    set_angular_drive_target,
    set_prismatic_joint_position,
    wait_for_joint_position,
)
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

    def __init__(self):
        self._world = None
        self._did_run = False

    def setup(self):
        self._world = World()
        self._world.reset()
        self._did_run = False

        self.box1 = DynamicCuboid(
            prim_path=f"{PICK_BOX_1}_1",
            position=np.array((3, 0, 0.5)),
            scale=np.array((1, 1, 1)),
            color=np.array((0.1, 0.2, 0.9)),
        )

        self.box2 = DynamicCuboid(
            prim_path=f"{PICK_BOX_1}_2",
            position=np.array((3, 0, 1.0)),
            scale=np.array((1, 1, 1)),
            color=np.array((0.9, 0.2, 0.1)),
        )

        self._scenario_generator = self._run_simulation()

    def reset(self):
        self._did_run = False
        if self._world is not None:
            self._world.reset()

    def update(self, step: float):
        try:
            next(self._scenario_generator)
            return False  # Scenario is still running
        except StopIteration:
            return True  # Scenario finished

    def _run_simulation(self):
        dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        articulation = dc_interface.get_articulation("/World/Robot")

        axis2_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS2_JOINT_PATH
        )
        axis1_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS1_JOINT_PATH
        )
        axis3_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS3_JOINT_PATH
        )

        # Simulate stacking
        for box in [self.box1, self.box2]:
            # Move robot to box position
            set_angular_drive_target(AXIS1_JOINT_PATH, 90)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis1_dof_index,
                target_position=-90,
                pos_threshold=0.1,
            )

            set_prismatic_joint_position(AXIS3_JOINT_PATH, -0.9)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis3_dof_index,
                target_position=-0.9,
                pos_threshold=0.1,
            )

            set_prismatic_joint_position(AXIS2_JOINT_PATH, -1.3)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis2_dof_index,
                target_position=-1.3,
                pos_threshold=0.1,
            )

            close_gripper()
            set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.6)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis2_dof_index,
                target_position=0.6,
                pos_threshold=0.1,
            )

            # Move robot to stacking position
            set_angular_drive_target(AXIS1_JOINT_PATH, 0)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis1_dof_index,
                target_position=0,
                pos_threshold=0.1,
            )

            set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis3_dof_index,
                target_position=0,
                pos_threshold=0.1,
            )

            open_gripper()
            yield from wait_for_joint_position(
                dc_interface,
                articulation,
                axis2_dof_index,
                target_position=1.0,
                pos_threshold=0.1,
            )

        print("Stacking scenario complete.")
        self._world.stop()
