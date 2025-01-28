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
from omni.isaac.sensor import ContactSensor

sim = SimulationContext()
#sim.initialize_physics()

def create_groundPlane(path):
    PhysicsContext()
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))
    print("Created ground plane")

def setup_start_cubes(prim_path1, prim_path2, position1=(0, 0, 0), scale1=(1, 1, 1), color1=(0, 0, 0),
                        position2=(0, 0, 0), scale2=(1, 1, 1), color2=(0, 0, 0)):
    global cube1
    cube1 = DynamicCuboid(
        prim_path=prim_path1,
        position=np.array(position1),
        scale=np.array(scale1),
        color=np.array(color1)
    )
    print("Created cube1")

    cube2 = DynamicCuboid(
        prim_path=prim_path2,
        position=np.array(position2),
        scale=np.array(scale2),
        color=np.array(color2)
    )
    print("Created cube2")

def create_surfaceGripper_actionGraph(graph_path, grip_position_path, parent_rigidBody_path):
    keys = og.Controller.Keys
    (graph_handle, list_of_nodes, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("surface_gripper", "omni.isaac.surface_gripper.SurfaceGripper"),
                ("close", "omni.graph.action.OnImpulseEvent"),
                ("impulse_monitor", "omni.graph.action.OnImpulseEvent"),
                ("open", "omni.graph.action.OnImpulseEvent"),
            ],
            keys.SET_VALUES: [
                ("surface_gripper.inputs:GripPosition", grip_position_path),
                ("surface_gripper.inputs:ParentRigidBody", parent_rigidBody_path),
            ],
            keys.CONNECT: [
                ("impulse_monitor.outputs:execOut", "surface_gripper.inputs:onStep"),
                ("open.outputs:execOut", "surface_gripper.inputs:Open"), # open
                ("close.outputs:execOut", "surface_gripper.inputs:Close"), # close
            ],
        },
    )
    print("Created surface gripper action graph")

def create_xform(path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xform.Define(stage, path)

    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    
    print("Created Xform")

def move_cube_up_and_down(cube, pos):
    cube.set_world_pose(pos)

def create_contact_sensor():
    sensor = ContactSensor(
        prim_path="/World/cube1/Contact_Sensor",
        frequency=60,
        translation=np.array([0, 0, 0]),
        min_threshold=0,
        max_threshold=10000000,
        radius=-1
    )

def setup_scene():
    print("Setting up scene...")

    # Objects
    create_groundPlane("/World/groundPlane")
    setup_start_cubes("/World/cube1", "/World/cube2", position1=(0.0, 0.0, 2), scale1=(1, 1, 1), color1=(0.2, 0.5, 0.7),
                        position2=(0.0, 0.0, 0.5), scale2=(1, 1, 1), color2=(0.7, 0.5, 0.2))

    # Surface gripper related
    create_surfaceGripper_actionGraph("/World/cube1/SurfaceGripperActionGraph", "/World/cube1/SurfaceGripperActionGraph/SurfaceGripperOffset", "/World/cube1")
    create_xform("/World/cube1/SurfaceGripperActionGraph/SurfaceGripperOffset", translate=(0, 0, -0.500997), rotation=(0, 0, 0), scale=(1, 1, 1))

    # Sensors
    create_contact_sensor()

setup_scene()