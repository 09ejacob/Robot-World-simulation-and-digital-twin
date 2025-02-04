import numpy as np
import time
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.surface_gripper import SurfaceGripper
from omni.isaac.core.physics_context import PhysicsContext
import omni.graph.core as og
from pxr import UsdGeom, Gf
import omni.usd

sim = SimulationContext()


def move_axis2_up_and_down():
    stage = omni.usd.get_context().get_stage()
    axis2_prim = stage.GetPrimAtPath("/World/Robot/Tower/Axis2")

    if not axis2_prim:
        print("Axis2 not found!")
        return

    translate_op = UsdGeom.Xformable(axis2_prim).GetOrderedXformOps()[0]

    for step in range(200):
        new_z = 2.0 + 0.5 * np.sin(step * 0.1)
        translate_op.Set(Gf.Vec3d(0, 0, new_z))

        sim.step()
        time.sleep(0.01)


move_axis2_up_and_down()
