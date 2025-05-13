import carb
import time
import os
import queue
import threading
import psutil
import numpy as np
from os.path import dirname, abspath, join
from pxr import UsdGeom, Gf
import omni.usd
from omni.isaac.core import World
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from isaacsim.core.api.objects.cuboid import DynamicCuboid
from pxr.UsdGeom import Tokens as UsdGeomTokens

from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    ENVIRONMENT_PATH,
    SHELF_PATH,
    BROADCAST_RATE,
    LISTEN_HOST,
    LISTEN_PORT,
    SEND_HOST,
    SEND_PORT,
    DEFAULT_STEREO_PAIR_ID,
)
from ..networking.udp_controller import UDPController


class UDPScenario:
    def __init__(
        self,
        robot_controller,
        world=None,
        print_positions=False,
        print_performance_stats=False,
        allow_udp_capture=True,
        allow_pointcloud_capture=False,
    ):
        self._robot_controller = robot_controller

        self._world = world
        self._did_run = False

        self.command_queue = queue.Queue()
        self.udp = UDPController()
        self.udp.callback = self._udp_callback

        self.broadcast_rate = BROADCAST_RATE
        self.broadcast_target_host = SEND_HOST
        self.broadcast_target_port = SEND_PORT
        self.broadcast_stop_event = threading.Event()
        self.broadcast_thread = None

        self.print_performance_stats = print_performance_stats
        self.print_positions = print_positions
        self.udp_message_count = 0
        self.executed_command_count = 0
        self.last_time_check = time.time()
        self.last_position_print_time = time.time()

        self.allow_udp_capture = allow_udp_capture
        self.allow_pointcloud_capture = allow_pointcloud_capture
        self.overview_camera_active = False
        self.last_overview_capture_time = 0
        self.overview_capture_interval = 0.2  # Deafult, can be set with UDP command

        self.axis_config = [
            ("axis1", AXIS1_JOINT_PATH, True),
            ("axis2", AXIS2_JOINT_PATH, False),
            ("axis3", AXIS3_JOINT_PATH, False),
            ("axis4", AXIS4_JOINT_PATH, True),
        ]
        self.axis_dofs = []

        self.boxes_paths = []
        self.pallets_paths = []

    def _udp_callback(self, message):
        """Receive UDP messages and store them in a queue."""
        self.command_queue.put(message)
        self.udp_message_count += 1

    def _parse_and_execute_command(self, message):
        """Parse an incoming command and execute the corresponding handler.

        Messages must be in a form like this:
            "command:arg1:arg2:..."

        This method will:
            1. Split messages on ":", strip trailing colons, and drop empty parts.
            2. Look up the command in the handler map.
            3. Call the handler with the parts list if the command is found.
            4. If command starts with "axis", route to the "_handle_axis_command".
            5. If command is not recognized, print out an error.
        """
        self.executed_command_count += 1
        parts = [p for p in message.rstrip(":").strip().split(":") if p]
        if not parts:
            carb.log_error("Empty command.")
            return

        command = parts[0].lower()
        handlers = self._get_command_handlers()

        if command in handlers:
            handlers[command](parts)

        elif command.startswith("axis"):
            self._handle_axis_command(parts)

        else:
            carb.log_error(f"Command not recognized: {message}")

    def _get_command_handlers(self):
        """Return a dictionary that maps command strings to handlers."""
        return {
            "tp_robot": self._handle_tp_robot,
            "close_gripper": lambda p: self._robot_controller.close_gripper(),
            "open_gripper": lambda p: self._robot_controller.open_gripper(),
            "bottlegripper_idle": lambda p: self._robot_controller.set_bottlegripper_to_idle_pos(),
            "capture": lambda p: self._handle_capture_command(p),
            "reload": lambda p: self._reload_scene(),
            "start_overview_camera": lambda p: self._toggle_overview_camera(True, p),
            "stop_overview_camera": lambda p: self._toggle_overview_camera(False),
            "add_colliding_item": lambda p: self._robot_controller.add_colliding_item(),
            "capture_pointcloud": lambda p: self._handle_capture_pointcloud_command(p),
        }

    def _toggle_overview_camera(self, start, parts=None):
        """
        Enable or disable overview-camera captures.

        If "start" is True, it begins capturing at the current "overview_capture_interval" interval (in seconds).
        You can override the interval by passing a new value as the second element of "parts".

        If "start" becomes False, it stops capturing and then generates a video at 3 / interval fps.
        """
        if self.allow_udp_capture:
            if start:
                # Parse interval from command if provided
                if parts and len(parts) > 1:
                    try:
                        interval = float(parts[1])
                        if interval > 0:
                            self.overview_capture_interval = interval
                            print(
                                f"[MAIN] Set overview camera interval to {interval:.3f} seconds"
                            )
                        else:
                            carb.log_warn(
                                f"Interval must be > 0 to activate overview camera. Got: {interval}"
                            )
                            return
                    except ValueError:
                        carb.log_warn(f"Invalid overview camera interval: {parts[1]}")
                        return

            self.overview_camera_active = start
            state = "started" if start else "stopped"
            print(f"[DEBUG] Overview-camera capturing {state}.")

            if not start:
                self._robot_controller.generate_video(
                    3
                    / self.overview_capture_interval,  # Dividing by 3 to regulate the speed of the generated video.
                    "OverviewCamera",
                )
        else:
            print("[MAIN] Overview camera is disabled.")

    def _handle_tp_robot(self, parts):
        """Parse and execute a "tp_robot:x:y:z" -command"""
        if len(parts) != 4:
            carb.log_error("Invalid tp_robot format. Use: tp_robot:x:y:z")
            return

        try:
            pos = list(map(float, parts[1:4]))
            self._robot_controller.teleport_robot(pos)
            if self.print_positions:
                print(f"[DEBUG] Teleported robot to position: {pos}")

        except ValueError:
            carb.log_error("tp_robot values must be floats.")

    def _handle_axis_command(self, parts):
        """Parse and execute a "axisX:value" -command."""
        if len(parts) != 2:
            carb.log_error("axis command format must be: axisX:value")
            return

        try:
            axis_id = int(parts[0].replace("axis", ""))
            value = float(parts[1])

        except ValueError:
            carb.log_error("Invalid axis id or value")
            return

        axis_map = {
            1: lambda v: self._robot_controller.set_angular_drive_target(
                AXIS1_JOINT_PATH, v
            ),
            2: lambda v: self._robot_controller.set_prismatic_joint_position(
                AXIS2_JOINT_PATH, v
            ),
            3: lambda v: self._robot_controller.set_prismatic_joint_position(
                AXIS3_JOINT_PATH, v
            ),
            4: lambda v: self._robot_controller.set_angular_drive_target(
                AXIS4_JOINT_PATH, v
            ),
        }

        handler = axis_map.get(axis_id)
        if handler:
            handler(value)

        else:
            carb.log_error(f"Axis {axis_id} not supported.")

    def _handle_capture_command(self, parts):
        """
        Parse and execute a "capture" UDP command.

        The expected format is:
            capture:cam1:cam2[:...][:stream=true|false]

        - The first part is which camera IDs to capture images from.
        - If the last part is "stream=True" or "stream=False", it toggles
          the UDP streaming on/off. Deafult is False.
        """
        print(f"[DEBUG] Received capture command with parts: {parts}")

        if not self.allow_udp_capture:
            carb.log_warn("Camera capture is disabled.")
            return

        if len(parts) < 2:
            carb.log_error(
                "No cameras specified. Use format: capture:cam1:cam2[:...][:stream=true|false]"
            )
            return

        # Check if the last part is a stream directive
        stream = False
        if "stream=" in parts[-1].lower():
            stream_arg = parts[-1].lower()
            if stream_arg == "stream=true":
                stream = True
            elif stream_arg == "stream=false":
                stream = False
            else:
                carb.log_error(
                    f"Invalid stream value: {parts[-1]}. Use stream=true or stream=false."
                )
                return
            cameras = parts[1:-1]  # exclude the stream parameter
        else:
            cameras = parts[1:]

        if not cameras or not all(cameras):
            carb.log_error(f"Invalid or missing camera names in command: {parts}")
            return

        print(f"[MAIN] Capturing from cameras: {cameras} | stream={stream}")

        self._robot_controller.capture_cameras(
            cameras=cameras,
            udp_controller=self.udp,
            host=self.broadcast_target_host,
            port=self.broadcast_target_port,
            stream=stream,
        )

    def _handle_capture_pointcloud_command(self, parts):
        """
        Handle the 'capture_pointcloud' UDP command.

        Format:
            capture_pointcloud[:pair_name]

        If pair_name is not provided, the default pair ID from global_variables.DEFAULT_STEREO_PAIR_ID is used.
        """
        if not self.allow_pointcloud_capture:
            carb.log_warn(
                "Cannot capture pointcloud because 3D features are not enabled."
            )
            return

        pair_name = parts[1] if len(parts) > 1 else DEFAULT_STEREO_PAIR_ID
        print(f"[MAIN] Attempting stereo pointcloud capture for pair: {pair_name}")

        result = self._robot_controller.camera_controller.save_stereo_pointcloud_pair(
            pair_id=DEFAULT_STEREO_PAIR_ID,
            pair_name=pair_name,
        )

        if result is None:
            carb.log_error(
                f"Failed to capture or save stereo pointcloud for {pair_name}"
            )
        else:
            print(f"[MAIN] Stereo pointcloud saved: {result}")

    def _create_xform(
        self, path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    ):
        """Create a xform at given path based on given translation, rotation, and scale"""
        stage = omni.usd.get_context().get_stage()
        xform_prim = stage.GetPrimAtPath(path)

        if not xform_prim.IsValid():
            xform = UsdGeom.Xform.Define(stage, path)
        else:
            xform = UsdGeom.Xform(xform_prim)

        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.ClearXformOpOrder()

        xformable.AddTranslateOp().Set(Gf.Vec3d(*translate))
        xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        xformable.AddScaleOp().Set(Gf.Vec3f(*scale))

    def _random_color(self):
        return np.random.rand(3)

    def _create_boxes(
        self,
        path,
        num_boxes,
        position=(1, 1, 1),
        stack_id=1,
        reverse=False,
        box_size: tuple = (0.3, 0.4, 0.2),
        mass=1,
    ):
        """
        Create a stack of simple DynamicCuboid boxes.

        Args:
            path (str): USD parent path to create the boxes.
            num_boxes (int): how many boxes to create in the stack.
            position (tuple): base (x, y, z) position for the stack.
            stack_id (int): identifier which is used in each box's prim name.
            reverse (bool): if this is True, flip the x-ordering of the stack.
        """
        dx, dy, dz = box_size

        pallet_length = 1.2
        pallet_width = 0.8
        pallet_height = 0.144

        base_x, base_y, pallet_center_z = position

        num_cols = max(1, int(pallet_length // dx))
        num_rows = max(1, int(pallet_width // dy))
        layer_capacity = num_cols * num_rows

        start_x = base_x - (pallet_length / 2) + (dx / 2)
        start_y = base_y - (pallet_width / 2) + (dy / 2)

        base_z = pallet_center_z + (pallet_height) + (dz / 2)

        boxes = []
        for i in range(num_boxes):
            layer = i // layer_capacity
            index_in_layer = i % layer_capacity
            col = index_in_layer // num_rows
            row = index_in_layer % num_rows

            if reverse:
                x = start_x + col * dx
            else:
                x = start_x + (num_cols - 1 - col) * dx
            y = start_y + row * dy
            z = base_z + layer * dz

            prim_path = f"{path}/box_{stack_id}_{i + 1}"
            box = DynamicCuboid(
                prim_path=prim_path,
                position=np.array((x, y, z)),
                scale=np.array((dx, dy, dz)),
                color=self._random_color(),
                mass=mass,
            )
            boxes.append(box)
            self.boxes_paths.append(prim_path)

        return boxes

    def _create_bottles(
        self,
        path,
        num_bottles,
        position=(2.0, 0.0, 0.072),
        stack_id=1,
        reverse=False,
        mass=1,
    ):
        """
        Create a stack of simple with BottlePack.usd objects.

        Args:
            path (str): USD parent path to create the bottles.
            num_boxes (int): how many bottles to create in the stack.
            position (tuple): base (x, y, z) position for the stack.
            stack_id (int): identifier which is used in each bottles prim name.
            reverse (bool): if this is True, flip the x-ordering of the stack.
        """
        stack_path = f"{path}/stack{stack_id}"
        self._create_xform(stack_path, translate=(0, 0, 0))

        module_dir = dirname(abspath(__file__))
        usd_path = abspath(
            join(
                module_dir,
                "..",
                "..",
                "Grab_Digital_Twin_python",
                "usd",
                "BottlePack.usd",
            )
        )
        if not os.path.exists(usd_path):
            carb.log_error(f"Box USD asset not found at {usd_path}")
            return

        bx, by, bz = position

        start_x = bx + 0.45
        x_inc = 0.3
        x_positions = [start_x - i * x_inc for i in range(4)]

        if reverse:
            x_positions = x_positions[::-1]

        y_positions = [by + 0.3 - j * 0.2 for j in range(4)]
        z_base = bz + 0.073 * 2
        z_inc = 0.320

        stage = omni.usd.get_context().get_stage()

        for i in range(num_bottles):
            layer = i // 16
            idx = i % 16
            col = idx // 4
            row = idx % 4

            x = x_positions[col]
            y = y_positions[row]
            z = z_base + layer * z_inc

            prim_path = f"{stack_path}/box_{stack_id}_{i + 1}"

            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                carb.log_warn(f"Failed to reference box at {prim_path}")
                continue

            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

            color = Gf.Vec3f(*self._random_color())
            gprim = UsdGeom.Gprim(prim)
            pv = gprim.CreateDisplayColorPrimvar(UsdGeomTokens.constant, 3)
            pv.Set([color])

            self.boxes_paths.append(prim_path)

    def _create_pick_stack(
        self,
        path,
        pallet_position=(0, 0, 0),
        number_of_items=1,
        stack_id=1,
        reverse=False,
        isBottles=False,
        box_size=(1, 1, 1),
        mass=1,
    ):
        """
        Create a full pick stack. Use Euro_Pallet.usd as the pallet and fill the stack with boxes or bottles.

        Args:
            path (str): USD parent path.
            pallet_position (tuple): the (x, y, z) position of the pallet.
            number_of_items (int): the number of boxes or bottles to create on the stack.
            stack_id (int): this is used to generate unique prim names.
            reverse (bool): reverse the layout direction if True.
            isBottles (bool): if True, make stack of bottles, else make it with boxes.
        """
        stack_path = f"{path}/stack{stack_id}"
        self._create_xform(stack_path, (0, 0, 0), (0, 0, 0), (1, 1, 1))

        module_dir = dirname(abspath(__file__))
        pallet_usd = abspath(
            join(
                module_dir,
                "..",
                "..",
                "Grab_Digital_Twin_python",
                "usd",
                "Euro_Pallet.usd",
            )
        )
        pallet_prim_path = f"{stack_path}/pallet{stack_id}"
        add_reference_to_stage(usd_path=pallet_usd, prim_path=pallet_prim_path)

        stage = omni.usd.get_context().get_stage()
        pallet_prim = stage.GetPrimAtPath(pallet_prim_path)
        if pallet_prim.IsValid():
            xform = UsdGeom.Xformable(pallet_prim)
            xform.ClearXformOpOrder()

            xform.AddTranslateOp().Set(Gf.Vec3d(*pallet_position))

            self.pallets_paths.append(pallet_prim_path)
        else:
            carb.log_error(f"Failed to reference pallet at {pallet_prim_path}")

        if isBottles:
            self._create_bottles(
                stack_path,
                number_of_items,
                pallet_position,
                stack_id,
                reverse,
                mass,
            )
        else:
            self._create_boxes(
                stack_path,
                number_of_items,
                pallet_position,
                stack_id,
                reverse,
                box_size,
                mass,
            )

    def _load_shelf_usd(self, position=(0, 0, 0), scale=(1, 1, 1)):
        """Load the shelf USD at a specific position at a specific scale."""
        current_dir = dirname(abspath(__file__))
        usd_path = abspath(
            join(
                current_dir, "..", "..", "Grab_Digital_Twin_python", "usd", "shelf.usd"
            )
        )

        add_reference_to_stage(usd_path=usd_path, prim_path=SHELF_PATH)
        stage = omni.usd.get_context().get_stage()
        shelf_prim = stage.GetPrimAtPath(SHELF_PATH)

        if shelf_prim.IsValid():
            xformable = UsdGeom.Xformable(shelf_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
            xformable.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
                Gf.Vec3d(*scale)
            )

            print(f"[MAIN] Shelf loaded at position: {position}")
        else:
            carb.log_error(f"Failed to load shelf at prim path: {SHELF_PATH}")

    def _start_udp_server(self, host=LISTEN_HOST, port=LISTEN_PORT):
        """
        Start the UDPController listening on the given host/port unless it is already in use.

        Args:
            host (str): IP to bind to.
            port (str): UDP port to listen on.
        """
        if self._port_in_use(port):
            print(f"[MAIN] Port {port} is already in use. Skipping server start.")
            return

        self.udp.host = host
        self.udp.port = port
        self.udp.start()

    def _port_in_use(self, port):
        """Check if the given UDP port is already in use locally."""
        for conn in psutil.net_connections(kind="udp"):
            if conn.laddr.port == port:
                return True

        return False

    def update(self, step: float = 0.1):
        """Process the queued commands, broadcast joint and force data, and log performance and print positions, capture overview camera if enabled."""
        start_time = time.time()
        self._process_command_queue()

        self._broadcast_data()

        self._log_performance_stats(start_time)
        self._print_joint_positions(start_time)
        self._capture_overview_camera(start_time)

    def _process_command_queue(self):
        """Drain the UDP command queue, parsing and executing each message."""
        while not self.command_queue.empty():
            message = self.command_queue.get()
            self._parse_and_execute_command(message)

    def _broadcast_data(self):
        """Get current joint and force data, format it and send over UDP."""
        if not self.broadcast_stop_event.is_set():
            data = []

            for name, dof_index, is_angular in self.axis_dofs:
                pos = self._robot_controller.get_joint_position_by_index(
                    dof_index, is_angular
                )
                if pos is not None:
                    data.append(f"{name}:{pos:.4f}")

            try:
                reading = self._robot_controller.get_force_sensor_data()
                force_n = reading.get("force", 0)
                data.append(f"force_N:{force_n:.2f}")
            except Exception as e:
                carb.log_warn(f"Could not read force sensor: {e}")

            if data:
                self.udp.send(
                    ";".join(data),
                    self.broadcast_target_host,
                    self.broadcast_target_port,
                )
            else:
                carb.log_warn("No joint data to broadcast.")

    def _log_performance_stats(self, current_time):
        """If this is enabled, print out how many UDP commands processed every second."""
        if self.print_performance_stats and (
            current_time - self.last_time_check >= 1.0
        ):
            print(
                f"[DEBUG] UDP Received: {self.udp_message_count} msg/sec | Executed: {self.executed_command_count} cmd/sec"
            )

            self.udp_message_count = 0
            self.executed_command_count = 0
            self.last_time_check = current_time

    def _print_joint_positions(self, current_time):
        """Print joint positions once per second if enabled."""
        if self.print_positions and (
            current_time - self.last_position_print_time >= 1.0
        ):
            print(
                "[DEBUG] --------------------------------------------------------------"
            )
            for name, dof_index, is_angular in self.axis_dofs:
                self._robot_controller.print_joint_position_by_index(
                    dof_index, is_angular
                )

            self.last_position_print_time = current_time

    def _capture_overview_camera(self, current_time):
        """ "If overview camera is enabled and time is gone by, capture image."""
        if self.overview_camera_active and (
            current_time - self.last_overview_capture_time
            >= self.overview_capture_interval
        ):
            self._robot_controller.camera_controller.capture_image("OverviewCamera")
            self.last_overview_capture_time = current_time

    def _print_box_position(self, box_path):
        """Print the world position of a box at a given prim path."""
        stage = omni.usd.get_context().get_stage()
        box_prim = stage.GetPrimAtPath(box_path)
        if not box_prim.IsValid():
            carb.log_warn(f"Box prim not found at {box_path}")
            return
        xformable = UsdGeom.Xformable(box_prim)
        for op in xformable.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                pos = op.Get()
                print(f"[DEBUG] Box position for {box_path}: {pos}")
                return
        print(f"[MAIN] No translation op found for box at {box_path}")

    def _stop_broadcasting(self):
        """Stop broadcasting."""
        self.broadcast_stop_event.set()
        if self.broadcast_thread:
            self.broadcast_thread.join()
            self.broadcast_thread = None
        print("[MAIN] Broadcast thread stopped.")

    def setup(self):
        """Set up the world, refresh robot controller handles, create scene objects, and start UDP server."""
        self._world = World()
        self._world.reset()
        self._robot_controller.refresh_handles()
        self.axis_dofs = []

        for name, path, is_angular in self.axis_config:
            dof_index = self._robot_controller.get_dof_index_for_joint(path)
            if dof_index == -1:
                carb.log_error(f"Could not resolve DOF index for {name} ({path})")
            else:
                self.axis_dofs.append((name, dof_index, is_angular))

        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.0, 0),
            number_of_items=19,
            stack_id=1,
            reverse=True,
            isBottles=False,
            box_size=(0.3, 0.4, 0.2),
            mass=14,
        )
        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, -0.9, 0),
            number_of_items=20,
            stack_id=2,
            reverse=True,
            isBottles=False,
            box_size=(0.3, 0.4, 0.2),
            mass=14,
        )
        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.9, 0),
            number_of_items=15,
            stack_id=3,
            reverse=True,
            isBottles=True,
            mass=9,
        )
        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.0, 1.8),
            number_of_items=30,
            stack_id=4,
            reverse=True,
            isBottles=False,
            box_size=(0.3, 0.4, 0.2),
            mass=14,
        )
        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, -0.9, 1.8),
            number_of_items=16,
            stack_id=5,
            reverse=True,
            isBottles=True,
            mass=9,
        )
        self._create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.9, 1.8),
            number_of_items=16,
            stack_id=6,
            reverse=True,
            isBottles=True,
            mass=9,
        )

        self._load_shelf_usd(position=(-1.3, -1.4, 0), scale=(1, 0.7, 1))
        self._start_udp_server()

    def unload(self):
        """Tear down the current simulation: stop UDP, reset world, and remove scene objects."""
        self._did_run = False

        self.udp.stop()

        self._stop_broadcasting()

        if self._world is not None:
            self._world.reset()

        self._remove_scenario_specific_prims()

    def _remove_scenario_specific_prims(self):
        """Remove objects created by this scenario from the stage."""
        stage = omni.usd.get_context().get_stage()

        prim_paths = self.boxes_paths + self.pallets_paths + [SHELF_PATH]

        for prim_path in prim_paths:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                stage.RemovePrim(prim_path)

    def _reload_scene(self):
        """Reload the entire scene to mirror the headless runner startup process."""
        print("[MAIN] Reloading scene with UDP scenario...")

        self.unload()

        create_new_stage()
        setup_scene(enable_cameras=self.allow_udp_capture)

        self.setup()
        print("[MAIN] Scene reloaded and UDP scenario started.")


if __name__ == "__main__":
    robot_controller = ...
    scenario = UDPScenario(
        robot_controller, print_positions=True, print_performance_stats=True
    )
    scenario.setup()

    try:
        while True:
            scenario.update()
            scenario._world.step(render=False)
            scenario._world.step(render=True)

            time.sleep(0.1)

    except KeyboardInterrupt:
        scenario._stop_broadcasting()
        scenario.udp.stop()
        print("[MAIN] Exiting UDP scenario.")
