import robotController
import time
import asyncio

pos = 10

robotController.set_prismatic_joint_position("/World/Robot/Joints/PrismaticJointAxis2", -10)
robotController.close_gripper()
asyncio.sleep(50.0)
robotController.set_prismatic_joint_position("/World/Robot/Joints/PrismaticJointAxis2", 10)