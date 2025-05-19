# Usage

To enable this extension, run Isaac Sim with the flags --ext-folder {path_to_ext_folder} --enable {ext_directory_name}. A new toolbar tab called `Grab Digital Twin` should appear.

1. After opening `Grab Digital Twin` from the toolbar, a new tab should appear.
2. At the top, there should be various options for loading in the robot under the *World Controls* frame.
    - Toggling the `Enable Cameras` option decides whether or not the attached cameras should load in when loading in the robot. This is useful due to camera initialization potentially being slow.
    - If the cameras are enabled, toggling the additional `Enable 3D Features` option decides wether or not the box cameras should be capable of taking point cloud images.
    - Different variations of the robot can be selected from the `Select Grab USD` dropdown. This changes the type of gripper that the robot is fitted with. There are two types: surface gripper and bottle gripper.
    - There are a couple of inputs for specifying the refresh rate for the physics and rendering.
        - Changing the `Physics Rate (Hz)` changes the update frequency of the physics simulation. Changing this value, changes the accuracy of the physics calculations. However, higher values are more resource intensive.
        - Changing the `Rendering Rate (Hz)` changes the update frequency of the rendering inside Isaac Sim. Lowering this value can be used for speeding up the simulation.
    - Pressing the `LOAD` button should make the robot appear with no scenario selected.
3. From here it is possible to either run a scenario, or control the robot manually.
    - Under the *Scenario* frame, it is possible to select and load a one of several scenarios made for the robot.
        - A specific scenario can be selected from the `Select Scenario` dropdown.
        - After selecting a scenario while the simulation is not running, pressing the `RUN` button should make the robot perform the predefined task of the chosen scenario.
        - Pressing the `UNLOAD SCENARIO` button will reset the simulation and unload the currently loaded scenario.
    - Under the *Robot Controls* frame, it is possible to control various links and joints of the robot manually.
        - Pressing the `Open Gripper` and `Close Gripper` buttons will make the end effector of the robot grab onto and release an object respectively.
        - The `Set bottlegripper to idle` button sets the bottle gripper to an idle state. This makes the it neither opened or closed, it gets put in a between state.
        - To change the robot's positioning configuration, set a value for one of the joints and then press the `Set position` button underneath to make it move to the specified value.
4. Additionally, there are extra controls for the cameras under the *Camera Controls* frame if the cameras are enabled.
    - Pressing the `Refresh Camera List` fetches the cameras that are loaded into the scene. After doing so, and the cameras are successfully fetched, buttons for capturing the individual cameras should show up under.
        - Pressing the `Capture from` buttons for the individual cameras will make the corresponding camera capture a picture.
    - Pressing the `Capture from All Cameras` button will make all the cameras capture a picture.
    - Pressing the `Capture pointclouds` button will take pointclouds from from both box cameras.

## Running in Headless Mode

To run the extension in Headless Mode (without a GUI), use the following command:

```bash
GRAB_RUN_HEADLESS=1 /path/to/isaac_sim/python.sh -m Grab_Digital_Twin_python.headless_runner
```
This automatically initializes the world and starts the UDP scenario.

To view all available flags and their descriptions, append the `--help` flag to the run command:

```bash
GRAB_RUN_HEADLESS=1 /path/to/isaac_sim/python.sh -m Grab_Digital_Twin_python.headless_runner --help
```

## UDP Controller Example
An example of sending and receiving UDP data and commands can be found in the following repository: https://github.com/09ejacob/UDP-Controller-Grab
