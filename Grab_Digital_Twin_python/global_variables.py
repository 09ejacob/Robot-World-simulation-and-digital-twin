# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

EXTENSION_TITLE = "Grab Digital Twin"

EXTENSION_DESCRIPTION = "solwr.grab.digital.twin"

# Robot
ROBOT_PATH = "/World/Robot"
GRIPPER_PATH = f"{ROBOT_PATH}/Tower/Axis2/gripper"

# Sensors and cameras
FORCE_SENSOR_PATH = f"{ROBOT_PATH}/Tower/Axis2/forceSensor"

# Joints
AXIS1_JOINT_PATH = f"{ROBOT_PATH}/Joints/RevoluteJointAxis1"
AXIS2_JOINT_PATH = f"{ROBOT_PATH}/Joints/PrismaticJointAxis2"
AXIS3_JOINT_PATH = f"{ROBOT_PATH}/Joints/PrismaticJointAxis3"
AXIS4_JOINT_PATH = f"{ROBOT_PATH}/Joints/RevoluteJointAxis4"

# Gripper Action Graph Paths
GRIPPER_OPEN_PATH = f"{GRIPPER_PATH}/SurfaceGripperActionGraph/open"
GRIPPER_CLOSE_PATH = f"{GRIPPER_PATH}/SurfaceGripperActionGraph/close"

# Other important paths
GROUND_PLANE_PATH = "/World/groundPlane"
PICK_BOX_PATH = "/World/Environment/pickBox"