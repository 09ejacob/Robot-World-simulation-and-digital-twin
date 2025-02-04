import numpy as np
import time
import json
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.surface_gripper import SurfaceGripper
from omni.isaac.core.physics_context import PhysicsContext
import omni.graph.core as og
from pxr import UsdGeom, Gf
import omni.usd
from omni.isaac.core import World

world = World()
world.scene.add_default_ground_plane()

unique_id = str(time.time())


def create_groundPlane(path):
    PhysicsContext()
    GroundPlane(prim_path=path, size=10, color=np.array([0.5, 0.5, 0.5]))
    print("Created ground plane")


def setup_robot(
    prim_path1,
    prim_path2,
    prim_path3,
    position1=(0, 0, 0),
    scale1=(1, 1, 1),
    color1=(0, 0, 0),
    position2=(0, 0, 0),
    scale2=(1, 1, 1),
    color2=(0, 0, 0),
    position3=(0, 0, 0),
    scale3=(1, 1, 1),
    color3=(0, 0, 0),
):
    global gripper
    gripper = world.scene.add(
        DynamicCuboid(
            prim_path=prim_path1,
            name=f"robot-gripper-{unique_id}",
            position=np.array(position1),
            scale=np.array(scale1),
            color=np.array(color1),
        )
    )
    print("Created gripper")

    global tower
    tower = world.scene.add(
        DynamicCuboid(
            prim_path=prim_path2,
            name=f"robot-tower-{unique_id}",
            position=np.array(position2),
            scale=np.array(scale2),
            color=np.array(color2),
        )
    )
    print("Created tower")

    global snake
    snake = world.scene.add(
        DynamicCuboid(
            prim_path=prim_path3,
            name=f"robot-snake-{unique_id}",
            position=np.array(position3),
            scale=np.array(scale3),
            color=np.array(color3),
        )
    )
    print("Created snake")


def create_pickBox(prim_path, position=(0, 0, 0), scale=(1, 1, 1), color=(4, 4, 4)):
    global pickBox
    pickBox = world.scene.add(
        DynamicCuboid(
            prim_path=prim_path,
            name=f"environment-pickBox-{unique_id}",
            position=np.array(position),
            scale=np.array(scale),
            color=np.array(color),
        )
    )
    print("Created pick-box")


def create_xform(path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xform.Define(stage, path)

    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))

    print("Created Xform")


def setup_scene():
    print("Setting up scene...")

    create_groundPlane("/World/groundPlane")

    create_xform(
        "/World/Robot", translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    )
    create_xform(
        "/World/Robot/Tower", translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    )
    create_xform(
        "/World/Robot/Tower/Axis2",
        translate=(0, 0, 0),
        rotation=(0, 0, 0),
        scale=(1, 1, 1),
    )

    setup_robot(
        "/World/Robot/Tower/Axis2/gripper",
        "/World/Robot/Tower/tower",
        "/World/Robot/Tower/Axis2/snake",
        position1=(0.0, 2.25, 1.87),
        scale1=(0.6, 0.3, 0.1),
        color1=(0.2, 0.5, 0.7),  # gripper
        position2=(0.0, 0, 1.5),
        scale2=(0.8, 0.5, 3),
        color2=(0.7, 0.3, 0.5),  # tower
        position3=(0.0, 1.25, 2),
        scale3=(0.15, 2, 0.15),
        color3=(0.2, 0.5, 0.3),  # snake
    )

    create_pickBox(
        "/World/Environment/pickBox",
        position=(0, 2.3, 0.3),
        scale=(1, 1, 0.5),
        color=(2, 2, 2),
    )  # pick-box

    # world.step()

    print("Scene setup complete.")


def store_names_to_file(filename):
    data = {
        "gripper_name": gripper.name,
        "tower_name": tower.name,
        "snake_name": snake.name,
        "pickBox_name": pickBox.name,
    }

    with open(filename, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Names stored in {filename}")


setup_scene()
# store_names_to_file("object_names.json") # Fix this
