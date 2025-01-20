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

def create_groundPlane(path):
    PhysicsContext()
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))
    print("Created ground plane")

def setup_start_cubes(prim_path1, prim_path2, prim_path3,
                        position1=(0, 0, 0), scale1=(1, 1, 1), color1=(0, 0, 0),
                        position2=(0, 0, 0), scale2=(1, 1, 1), color2=(0, 0, 0),
                        position3=(0, 0, 0), scale3=(1, 1, 1), color3=(0, 0, 0)):
    #global gripper
    gripper = DynamicCuboid(
        prim_path=prim_path1,
        position=np.array(position1),
        scale=np.array(scale1),
        color=np.array(color1)
    )
    print("Created gripper")

    axis2 = DynamicCuboid(
        prim_path=prim_path2,
        position=np.array(position2),
        scale=np.array(scale2),
        color=np.array(color2)
    )
    print("Created axis2")

    snake = DynamicCuboid(
        prim_path=prim_path3,
        position=np.array(position3),
        scale=np.array(scale3),
        color=np.array(color3)
    )

def create_xform(path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xform.Define(stage, path)

    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    
    print("Created Xform")

def setup_scene():
    print("Setting up scene...")

    # Objects
    create_groundPlane("/World/groundPlane")

    create_xform("/World/Robot", translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1))
    setup_start_cubes("/World/Robot/gripper", "/World/Robot/axis2", "/World/Robot/snake", 
                        position1=(0.0, 0.0, 1.87), scale1=(0.6, 0.3, 0.1), color1=(0.2, 0.5, 0.7), # gripper
                        position2=(0.0, 2.25, 1.5), scale2=(0.8, 0.5, 3), color2=(0.7, 0.3, 0.5), # axis2
                        position3=(0.0, 1, 2), scale3=(0.15, 2, 0.15), color3=(0.2, 0.5, 0.3)) # snake

setup_scene()