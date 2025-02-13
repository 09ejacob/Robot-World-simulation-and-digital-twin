import Grab_Digital_Twin_python.robot_controller as robot_controller
from omni.isaac.core.simulation_context import SimulationContext

sim_context = SimulationContext(physics_prim_path="/World/PhysicsScene")
sim_context.play()
sim_context.initialize_physics()

if sim_context._physics_context is None:
    raise RuntimeError("Physics context failed to initialize.")

print("Lowering prismatic joint ...")
robot_controller.set_prismatic_joint_position(
    "/World/Robot/Joints/PrismaticJointAxis2", -1.4
)
for _ in range(100):
    sim_context.step()

print("Raising prismatic joint ...")
robot_controller.set_prismatic_joint_position(
    "/World/Robot/Joints/PrismaticJointAxis2", 0.8
)
for _ in range(100):
    sim_context.step()

print("Rotating angular joint ...")
robot_controller.set_angular_drive_target("/World/Robot/Joints/RevoluteJointAxis1", 180)
for _ in range(100):
    sim_context.step()

print("Sequence complete!")
