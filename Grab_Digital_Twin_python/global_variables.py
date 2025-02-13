# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# Extension Info
EXTENSION_TITLE = "Grab Digital Twin"
EXTENSION_DESCRIPTION = "solwr.grab.digital.twin"

# Base world paths
WORLD_PATH = "/World"
GROUND_PLANE_PATH = f"{WORLD_PATH}/groundPlane"
PHYSICS_SCENE_PATH = f"{WORLD_PATH}/PhysicsScene"
PICK_BOX_PATH = f"{WORLD_PATH}/Environment/pickBox"

# Robot paths
ROBOT_PATH = f"{WORLD_PATH}/Robot"
TOWER_PATH = f"{ROBOT_PATH}/Tower"
AXIS2_PATH = f"{TOWER_PATH}/Axis2"
GRIPPER_PATH = f"{AXIS2_PATH}/gripper"

# Additional robot parts
TOWER_CUBOID_PATH = f"{TOWER_PATH}/tower"
SNAKE_PATH = f"{AXIS2_PATH}/snake"
SNAKE_BASE_PATH = f"{AXIS2_PATH}/snakeBase"
ROBOT_BASE_GROUP_PATH = f"{ROBOT_PATH}/Base"
ROBOT_BASE_CUBE_PATH = f"{ROBOT_PATH}/Base/base"

# Sensors and cameras
FORCE_SENSOR_PATH = f"{AXIS2_PATH}/forceSensor"

# Joints
AXIS1_JOINT_PATH = f"{ROBOT_PATH}/Joints/RevoluteJointAxis1"
AXIS2_JOINT_PATH = f"{ROBOT_PATH}/Joints/PrismaticJointAxis2"
AXIS3_JOINT_PATH = f"{ROBOT_PATH}/Joints/PrismaticJointAxis3"
AXIS4_JOINT_PATH = f"{ROBOT_PATH}/Joints/RevoluteJointAxis4"
FIXED_JOINT_BASE_GROUND = f"{ROBOT_PATH}/Joints/FixedJointBaseGround"
PRISMATIC_JOINT_FORCE_SENSOR = f"{ROBOT_PATH}/Joints/PrissmaticJointForceSensor"

# Gripper Action Graph Paths
GRIPPER_ACTION_GRAPH_PATH = f"{GRIPPER_PATH}/SurfaceGripperActionGraph"
GRIPPER_OPEN_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/open"
GRIPPER_CLOSE_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/close"
GRIPPER_OFFSET_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/SurfaceGripperOffset"
