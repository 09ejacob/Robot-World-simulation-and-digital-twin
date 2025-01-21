from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import json
import os

def read_and_print_names(filename):
    if not os.path.isfile(filename):
        print(f"Error: '{filename}' does not exist.")
        return

    with open(filename, "r") as f:
        data = json.load(f)

    # print("Names: ", filename, ":")
    # print("Gripper name:", data.get("gripper_name", "N/A"))
    # print("Tower name :", data.get("tower_name", "N/A"))
    # print("Snake name:", data.get("snake_name", "N/A"))
    # print("PickBox name:", data.get("pickBox_name", "N/A"))

#read_and_print_names("C:\Skole\Dataingenior\Bachelor\Code\Robot-World-simulation-and-digital-twin---Bachelor-group-9-2025\Code\object_names.json")


world = World()

for i in range(500):
    filename = "C:\Skole\Dataingenior\Bachelor\Code\Robot-World-simulation-and-digital-twin---Bachelor-group-9-2025\Code\object_names.json"
    with open(filename, "r") as f:
        data = json.load(f)


    position, orientation = data.get("gripper_name", "N/A").get_world_pose()
    linear_velocity = gripper.get_linear_velocity()
    
    print("Cube position is : " + str(position))
    print("Cube's orientation is : " + str(orientation))
    print("Cube's linear velocity is : " + str(linear_velocity))
    
    #world.step(render=True) # execute one physics step and one rendering step

# simulation_app.close() # close Isaac Sim