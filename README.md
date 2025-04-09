# Robot World simulation and digital twin - Bachelor group 9 2025

An extension for [Isaac Sim](https://developer.nvidia.com/isaac/sim) for creating a digital twin of Solwr's [Grab](https://solwr.com/products/grab) robot. This is a Bachelor's thesis done during the [IDATA2900](https://www.ntnu.edu/studies/courses/IDATA2900#tab=omEmnet) course.

## Importing the Extension

1. In the root folder of Issac Sim: clone this repository in a new folder within the `extsUser` folder.
2. Go to the extensions window in Isaac Sim and search for the `Grab Digital Twin` extension.
3. Enable the extension. ***(It might be necessary to enable auto loading so that it loads upon launching Isaac Sim)***
4. A new tab in the toolbar called `Grab Digital Twin` should appear.

NOTE: ***Make sure that the folder for the extension does not contain any triple-dashes (`---`), since these will cause an error in Isaac Sim and make the import unsuccessful.***

## Using the Extension

1. After opening `Grab Digital Twin` from the toolbar, a new tab should appear.
2. Pressing the `LOAD` button under *World Controls* should make the robot appear with the default scenario.
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
