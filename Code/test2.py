from omni.isaac.core.utils.stage import get_current_stage

def read_force_sensor_force(joint_path="/World/Robot/Joints/FixedJointForceSensor"):
    stage = get_current_stage()
    joint = stage.GetPrimAtPath(joint_path)
    if not joint or not joint.IsValid():
        print(f"No valid joint found at {joint_path}")
        return

    # Try to get the reaction force attribute.
    # The attribute name may be "physics:reactionForce" or similar.
    force_attr = joint.GetAttribute("physics:reactionForce")
    if force_attr:
        force_value = force_attr.Get()
        print(f"Force sensor reading at {joint_path}: {force_value}")
    else:
        print(f"No reaction force attribute found on {joint_path}")

# Example usage: call this function each simulation step or on demand.
read_force_sensor_force()
