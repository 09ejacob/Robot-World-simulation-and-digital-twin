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

PALLET_PATH = f"{WORLD_PATH}/Environment/pallet"
PICK_BOX_1 = f"{WORLD_PATH}/Environment/pickBox1"
PICK_BOX_2 = f"{WORLD_PATH}/Environment/pickBox2"
PICK_BOX_3 = f"{WORLD_PATH}/Environment/pickBox3"

# Robot paths
ROBOT_PATH = f"{WORLD_PATH}/Robot"
TOWER_PATH = f"{ROBOT_PATH}/Tower"
CAMERA_PATH = f"{ROBOT_PATH}/BoxCamera"
AXIS2_PATH = f"{TOWER_PATH}/Axis2"
GRIPPER_PATH = f"{AXIS2_PATH}/gripper"

# Additional robot parts
AXIS2_BASE_PATH = f"{AXIS2_PATH}/axis2_base"
SNAKE_PATH = f"{AXIS2_PATH}/snake"
SNAKE_BASE_PATH = f"{AXIS2_PATH}/snakeBase"
ROBOT_BASE_GROUP_PATH = f"{ROBOT_PATH}/Base"
ROBOT_BASE_CUBE_PATH = f"{ROBOT_PATH}/Base/base"
ROBOT_BASE_PATH = f"{AXIS2_PATH}/robot_base"
AXIS2_TOWER_PATH=f"{TOWER_PATH}/tower"
PALLET_BASE_PATH=f"{ROBOT_BASE_GROUP_PATH}/pallet_base"
CABINET_PATH=f"{ROBOT_BASE_GROUP_PATH}/cabinet"

# Sensors and cameras
FORCE_SENSOR_PATH = f"{AXIS2_PATH}/forceSensor"
BOX_CAMERA_1 = f"{CAMERA_PATH}/boxCamera1"
BOX_CAMERA_2 = f"{CAMERA_PATH}/boxCamera2"

# Joints
JOINTS_PATH = f"{ROBOT_PATH}/Joints"
AXIS1_JOINT_PATH = f"{JOINTS_PATH}/RevoluteJointAxis1"
AXIS2_JOINT_PATH = f"{JOINTS_PATH}/PrismaticJointAxis2"
AXIS3_JOINT_PATH = f"{JOINTS_PATH}/PrismaticJointAxis3"
AXIS4_JOINT_PATH = f"{JOINTS_PATH}/RevoluteJointAxis4"
BOX_CAMERA_1_SNAKE_BASE_JOINT_PATH = f"{JOINTS_PATH}/FixedJointSnakeBaseBoxCamera1"
BOX_CAMERA_2_SNAKE_BASE_JOINT_PATH = f"{JOINTS_PATH}/FixedJointSnakeBaseBoxCamera2"
FIXED_JOINT_BASE_GROUND = f"{JOINTS_PATH}/FixedJointBaseGround"
PRISMATIC_JOINT_FORCE_SENSOR = f"{JOINTS_PATH}/PrismaticJointForceSensor"
ROBOT_BASE_JOINT_PATH = f"{JOINTS_PATH}/RevoluteJointRobotBase"
AXIS2_TOWER_JOINT_PATH = f"{JOINTS_PATH}/FixedJointAxis2Tower"
PALLET_BASE_JOINT_PATH = f"{JOINTS_PATH}/FixedJointPalletBase"
CABINET_BASE_JOINT_PATH = f"{JOINTS_PATH}/FixedJointCabinetBase"

# Gripper Action Graph Paths
GRIPPER_ACTION_GRAPH_PATH = f"{GRIPPER_PATH}/SurfaceGripperActionGraph"
GRIPPER_OPEN_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/open"
GRIPPER_CLOSE_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/close"
GRIPPER_OFFSET_PATH = f"{GRIPPER_ACTION_GRAPH_PATH}/SurfaceGripperOffset"
