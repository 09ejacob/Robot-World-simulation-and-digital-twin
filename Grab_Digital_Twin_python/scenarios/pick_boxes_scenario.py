import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.dynamic_control import _dynamic_control
from ..robot_controller import (
    get_dof_index_for_joint_prim_path,
    open_gripper,
    close_gripper,
    set_angular_drive_target,
    set_prismatic_joint_position,
    read_force_sensor_value,
    wait_for_joint_position,
)
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    PALLET_PATH,
    PICK_BOX_1,
    PICK_BOX_2,
    PICK_BOX_3,
)


class PickBoxesScenario:
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

        pallet = DynamicCuboid(
            prim_path=PALLET_PATH,
            position=np.array((1.7, 0, 0.1)),
            scale=np.array((1.2, 0.8, 0.144)),
            color=np.array((0.2, 0.08, 0.05)),
        )

        pickBox1 = DynamicCuboid(
            prim_path=PICK_BOX_1,
            position=np.array((1.25, -0.2, 0.25)),
            scale=np.array((0.3, 0.4, 0.2)),
            color=np.array((0.05, 0.1, 0.08)),
        )

        pickBox2 = DynamicCuboid(
            prim_path=PICK_BOX_2,
            position=np.array((1.25, 0.2, 0.25)),
            scale=np.array((0.3, 0.4, 0.2)),
            color=np.array((0.1, 0.08, 0.05)),
        )

        pickBox3 = DynamicCuboid(
            prim_path=PICK_BOX_3,
            position=np.array((1.55, 0.2, 0.25)),
            scale=np.array((0.3, 0.4, 0.2)),
            color=np.array((0.08, 0.05, 0.1)),
        )

        self._scenario_generator = self._run_simulation()

    def reset(self):
        self._did_run = False
        if self._world is not None:
            self._world.reset()
        # Remove scenario-specific prims from the stage.
        stage = omni.usd.get_context().get_stage()
        for prim_path in [PALLET_PATH, PICK_BOX_1, PICK_BOX_2, PICK_BOX_3]:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                stage.RemovePrim(prim_path)

    def update(self, step: float):
        try:
            next(self._scenario_generator)
            return False  # Scenario is still running
        except StopIteration:
            # Generator is exhausted; the simulation is complete.
            return True

    def _run_simulation(self):
        dc_interface = _dynamic_control.acquire_dynamic_control_interface()
        articulation = dc_interface.get_articulation("/World/Robot")

        # Find which DOF index corresponds to prismatic or revolute joint
        axis2_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS2_JOINT_PATH
        )
        axis1_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS1_JOINT_PATH
        )
        axis3_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS3_JOINT_PATH
        )
        axis4_dof_index = get_dof_index_for_joint_prim_path(
            dc_interface, articulation, AXIS4_JOINT_PATH
        )

        print(f"axis2_dof_index: {axis2_dof_index}, axis1_dof_index: {axis1_dof_index}")

        # Start
        for _ in range(60):
            self._world.step(render=True)
            yield

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 90)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=-90,
            pos_threshold=0.1,
            max_frames=1000,
            is_angular=True,
        )

        # Raise axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.3)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.3,
            pos_threshold=0.1,
        )

        # Move axis3 out
        set_prismatic_joint_position(AXIS3_JOINT_PATH, -1.209)  # -1.259
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=1.209,
            pos_threshold=0.1,
        )

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 83.1573)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=-83.1573,
            pos_threshold=0.01,
            max_frames=1000,
            is_angular=True,
        )

        # Rotate axis4
        set_angular_drive_target(AXIS4_JOINT_PATH, -6.8427)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis4_dof_index,
            target_position=-6.8472,
            pos_threshold=0.01,
            max_frames=1000,
            is_angular=True,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.1)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.1,
            pos_threshold=0.0185,
        )

        # # Close gripper
        close_gripper()

        # Raise axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.7)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.7,
            pos_threshold=0.1,
        )

        # Rotate axis4
        set_angular_drive_target(AXIS4_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis4_dof_index,
            target_position=0,
            pos_threshold=0.01,
            max_frames=1000,
            is_angular=True,
        )

        # Move axis3 in
        set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=0,
            pos_threshold=0.1,
        )

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=0,
            pos_threshold=0.1,
            max_frames=1000,
            is_angular=True,
        )

        # Move axis3 out
        set_prismatic_joint_position(AXIS3_JOINT_PATH, -1)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=1,
            pos_threshold=0.1,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.5)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.5,
            pos_threshold=0.0185,
        )

        open_gripper()

        # Move axis3 in
        set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=0,
            pos_threshold=0.1,
        )

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 90)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=-90,
            pos_threshold=0.1,
            max_frames=1000,
            is_angular=True,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0,
            pos_threshold=0.1,
        )

        # Raise axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.3)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.3,
            pos_threshold=0.1,
        )

        # Move axis3 out
        set_prismatic_joint_position(AXIS3_JOINT_PATH, -1.209)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=1.209,
            pos_threshold=0.1,
        )

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 96.8427)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=-96.8427,
            pos_threshold=0.1,
            max_frames=1000,
            is_angular=True,
        )

        # Rotate axis4
        set_angular_drive_target(AXIS4_JOINT_PATH, 6.8427)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis4_dof_index,
            target_position=6.8472,
            pos_threshold=0.01,
            max_frames=1000,
            is_angular=True,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.1)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.1,
            pos_threshold=0.0185,
        )

        close_gripper()

        # Raise axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.7)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.7,
            pos_threshold=0.1,
        )

        # Rotate axis4
        set_angular_drive_target(AXIS4_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis4_dof_index,
            target_position=0,
            pos_threshold=0.01,
            max_frames=1000,
            is_angular=True,
        )

        # Move axis3 in
        set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=0,
            pos_threshold=0.1,
        )

        # Rotate axis1
        set_angular_drive_target(AXIS1_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis1_dof_index,
            target_position=0,
            pos_threshold=0.1,
            max_frames=1000,
            is_angular=True,
        )

        # Raise axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.9)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.9,
            pos_threshold=0.1,
        )

        # Move axis3 out
        set_prismatic_joint_position(AXIS3_JOINT_PATH, -1)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=1,
            pos_threshold=0.1,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.8)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0.8,
            pos_threshold=0.0185,
        )

        open_gripper()

        # Move axis3 out
        set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis3_dof_index,
            target_position=0,
            pos_threshold=0.1,
        )

        # Lower axis2
        set_prismatic_joint_position(AXIS2_JOINT_PATH, 0)
        yield from wait_for_joint_position(
            dc_interface,
            articulation,
            axis2_dof_index,
            target_position=0,
            pos_threshold=0.0185,
        )

        for _ in range(100):
            self._world.step(render=True)
            yield

        # # Lower axis2
        # set_prismatic_joint_position(AXIS2_JOINT_PATH, -1.3)
        # yield from wait_for_joint_position(
        #     dc_interface,
        #     articulation,
        #     axis2_dof_index,
        #     target_position=-1.3,
        #     pos_threshold=0.1,
        # )

        # for _ in range(60):
        #     self._world.step(render=True)
        #     yield

        # # Close gripper
        # close_gripper()

        # # Raise axis2
        # set_prismatic_joint_position(AXIS2_JOINT_PATH, 0.6)
        # yield from wait_for_joint_position(
        #     dc_interface,
        #     articulation,
        #     axis2_dof_index,
        #     target_position=0.6,
        #     pos_threshold=0.1,
        # )

        # # Rotate axis1
        # set_angular_drive_target(AXIS1_JOINT_PATH, 0)
        # yield from wait_for_joint_position(
        #     dc_interface,
        #     articulation,
        #     axis1_dof_index,
        #     target_position=0,
        #     pos_threshold=0.01,
        #     max_frames=1000,
        #     is_angular=True,
        # )

        # # Move axis3 out
        # set_prismatic_joint_position(AXIS3_JOINT_PATH, 0)
        # yield from wait_for_joint_position(
        #     dc_interface,
        #     articulation,
        #     axis3_dof_index,
        #     target_position=0,
        #     pos_threshold=0.1,
        # )

        # open_gripper()

        # # Finish
        for _ in range(60):
            self._world.step(render=True)
            yield

        print("Simulation complete. Stopping simulation.")
        self._world.stop()
