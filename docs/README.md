# Usage

To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}. A new toolbar tab called `Grab Digital Twin` should appear.

1. After opening `Grab Digital Twin` from the toolbar, a new tab should appear.
2. Pressing the `LOAD` button under *World Controls* should make the robot appear with no scenario selected.
3. From here it is possible to either run a scenario, or control the robot manually.
    - Under *Scenario* it should be possible to select one of several scenarios made for the robot.
        - After selecting a scenario while the simulation is not running, pressing the `RUN` button should make the robot perform the predefined task of the chosen scenario.
    - Under *Robot Controls* it should be possible to control various links and joints of the robot manually.
        - Pressing the `Open Gripper` and `Close Gripper` buttons will make the end effector of the robot grab onto and release an object respectively.
        - To change the robot's positioning configuration, set a value for one of the joints and then press the `Set position` button underneath to make it move to the specified value.
    - Under *Camera Controls* are controls for the cameras.
        - Pressing the `Refresh Camera List` fetches the cameras that are loaded into the scene. After doing so, and the cameras are successfully fetched, buttons for capturing the individual cameras should show up under.
            - Pressing the `Capture from` buttons for the individual cameras will make the corresponding camera capture a picture.
        - Pressing the `Capture from All Cameras` button will make all the cameras capture a picture.  
4. Pressing the `UNLOAD SCENARIO` button under the `LOAD` button will reset the simulation and unload the currently loaded scenario.