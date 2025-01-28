# main.py
from omni.isaac.core.simulation_context import SimulationContext
from SetupScene import setup_scene
import robotController

# 1. Set up the scene once
surface_gripper = setup_scene()

# 2. Pass the gripper to robotController
robotController.init_gripper(surface_gripper)

# 3. Start or retrieve a SimulationContext
sim_context = SimulationContext()

# 4. Execute the sequence of actions
# Lower the prismatic joint to -1.4
print("Lowering prismatic joint to -1.4...")
robotController.set_prismatic_joint_position(
    "/World/Robot/Joints/PrismaticJointAxis2", -1.4
)
for _ in range(100):
    sim_context.step()

# Close the gripper
print("Closing gripper to grip the box...")
robotController.close_gripper()
for _ in range(50):  #
    sim_context.step()

# Raise the prismatic joint to 0.8
print("Raising prismatic joint to 0.8...")
robotController.set_prismatic_joint_position(
    "/World/Robot/Joints/PrismaticJointAxis2", 0.8
)
for _ in range(100):
    sim_context.step()

# Set angular drive to 180 degrees
print("Rotating angular joint to 180 degrees...")
robotController.set_angular_drive_target("/World/Robot/Joints/RevoluteJointAxis1", 180)
for _ in range(100):
    sim_context.step()

# Open the gripper
print("Opening gripper to drop the box...")
robotController.open_gripper()
for _ in range(50):
    sim_context.step()

print("Sequence complete!")
