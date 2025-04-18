import time
import os
import queue
import threading
import psutil
import numpy as np
from os.path import dirname, abspath, join
from pxr import UsdGeom, Gf, UsdPhysics
import omni.usd
from omni.isaac.core import World
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage
from Grab_Digital_Twin_python.scenes.setup_scene import setup_scene
from omni.isaac.core.objects import DynamicCuboid

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
        self.overview_camera_active = False
        self.last_overview_capture_time = 0
        self.overview_capture_interval = 0.2

        self.axis_config = [
            ("axis1", AXIS1_JOINT_PATH, True),
            ("axis2", AXIS2_JOINT_PATH, False),
            ("axis3", AXIS3_JOINT_PATH, False),
            ("axis4", AXIS4_JOINT_PATH, True),
        ]
        self.axis_dofs = []

        self.boxes = []
        self.pallets = []

    def _udp_callback(self, message):
        """Receive UDP messages and store them in a queue."""
        self.command_queue.put(message)
        self.udp_message_count += 1

    def parse_and_execute_command(self, message):
        """Parse an incoming command and execute the corresponding handler."""
        self.executed_command_count += 1
        parts = [p for p in message.rstrip(":").strip().split(":") if p]
        if not parts:
            print("[ERROR] Empty command.")
            return

        command = parts[0].lower()
        handlers = self._get_command_handlers()

        if command in handlers:
            handlers[command](parts)

        elif command.startswith("axis"):
            self._handle_axis_command(parts)

        else:
            print("[ERROR] Command not recognized:", message)

    def _get_command_handlers(self):
        """Return a dictionary that maps command strings to handlers."""
        return {
            "tp_robot": self._handle_tp_robot,
            "nudge_box": self._handle_nudge_box,
            "force_data": lambda p: self._robot_controller.read_force_sensor_value(),
            "close_gripper": lambda p: self._robot_controller.close_gripper(),
            "open_gripper": lambda p: self._robot_controller.open_gripper(),
            "bottlegripper_idle": lambda p: self._robot_controller.set_bottlegripper_to_idle_pos(),
            "capture": lambda p: self._handle_capture_command(p),
            "reload": lambda p: self._reload_scene_with_model(p),
            "start_overview_camera": lambda p: self._toggle_overview_camera(True),
            "stop_overview_camera": lambda p: self._toggle_overview_camera(False),
        }

    def _toggle_overview_camera(self, start):
        if self.allow_udp_capture:
            self.overview_camera_active = start
            state = "started" if start else "stopped"

            print(f"Overview-camera capturing {state}.")

            if not start:
                self._robot_controller.generate_video(
                    3 / self.overview_capture_interval
                )

        else:
            print("[INFO] Overview camera is disabled.")

    def _handle_tp_robot(self, parts):
        if len(parts) != 4:
            print("[ERROR] Invalid tp_robot format. Use: tp_robot:x:y:z")
            return

        try:
            pos = list(map(float, parts[1:4]))
            self._robot_controller.teleport_robot(pos)
            if self.print_positions:
                print(f"Teleported robot to position: {pos}")

        except ValueError:
            print("[ERROR] tp_robot values must be floats.")

    def _handle_nudge_box(self, parts):
        if len(parts) != 5:
            print("[ERROR] Invalid nudge_box format. Use: nudge_box:/path:x:y:z")
            return

        try:
            prim_path = parts[1]
            offset = tuple(map(float, parts[2:5]))
            self.nudge_box(prim_path, offset)

        except ValueError:
            print("[ERROR] nudge_box values must be floats.")

    def _handle_axis_command(self, parts):
        if len(parts) != 2:
            print("[ERROR] axis command format must be: axisX:value")
            return

        try:
            axis_id = int(parts[0].replace("axis", ""))
            value = float(parts[1])

        except ValueError:
            print("[ERROR] Invalid axis id or value")
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
            print(f"[ERROR] Axis {axis_id} not supported.")

    def _handle_capture_command(self, parts):
        if self.allow_udp_capture:
            self._robot_controller.capture_from_all_cameras()

        else:
            print("[INFO] Camera is disabled.")

    def nudge_box(self, prim_path, offset):
        stage = omni.usd.get_context().get_stage()
        box_prim = stage.GetPrimAtPath(prim_path)

        if not box_prim.IsValid():
            print(f"Box prim not found at {prim_path}")
            return

        xformable = UsdGeom.Xformable(box_prim)
        translate_op = next(
            (
                op
                for op in xformable.GetOrderedXformOps()
                if "translate" in op.GetOpName()
            ),
            None,
        )
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()

        current_translation = translate_op.Get()
        new_translation = current_translation + Gf.Vec3d(*offset)
        translate_op.Set(new_translation)

        if self.print_positions:
            print(
                f"Nudged box at {prim_path} by {offset}. New position: {new_translation}"
            )

    def create_xform(
        self, path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)
    ):
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

    def random_color(self):
        return np.random.rand(3)

    def create_boxes(
        self, path, num_boxes, position=(1, 1, 1), stack_id=1, reverse=False
    ):
        boxes = []

        base_x, base_y, base_z = position
        start_x = base_x - 0.45
        x_inc = 0.3
        row_y = {0: base_y - 0.2, 1: base_y + 0.2}
        base_z = base_z + 0.072 + 0.1
        z_inc = 0.2
        max_col = 3

        for i in range(num_boxes):
            layer = i // 8
            index_in_layer = i % 8

            column = index_in_layer // 2
            row = index_in_layer % 2

            x = start_x + (column * x_inc if reverse else (max_col - column) * x_inc)
            y = row_y[row]
            z = base_z + layer * z_inc

            prim_path = f"{path}/box_{stack_id}_{i + 1}"

            box = DynamicCuboid(
                prim_path=prim_path,
                position=np.array((x, y, z)),
                scale=np.array((0.3, 0.4, 0.2)),
                color=self.random_color(),
                mass=14.0,
            )

            boxes.append(box)
            self.boxes.append(box)

        return boxes
    
    def create_bottles(
        self,
        path,
        num_boxes,
        position=(2.0, 0.0, 0.072),
        stack_id=1,
        reverse=False,
    ):
        stack_path = f"{path}/stack{stack_id}"
        self.create_xform(stack_path, translate=(0, 0, 0))

        module_dir = dirname(abspath(__file__))
        usd_path = abspath(
            join(
                module_dir, "..", "..",
                "Grab_Digital_Twin_python", "usd", "BottlePack.usd",
            )
        )
        if not os.path.exists(usd_path):
            print(f"[ERROR] Box USD asset not found at {usd_path}")
            return

        bx, by, bz = position

        start_x = bx + 0.45
        x_inc = 0.3
        x_positions = [start_x - i * x_inc for i in range(4)] 
        if reverse:
            x_positions = x_positions[::-1]

        y_positions = [by + 0.3 - j * 0.2 for j in range(4)]

        z_base = bz + 0.073
        z_inc = 0.320

        stage = omni.usd.get_context().get_stage()
        self.boxes = []

        for i in range(num_boxes):
            layer = i // 16
            idx   = i % 16
            col   = idx // 4
            row   = idx % 4

            x = x_positions[col]
            y = y_positions[row]
            z = z_base + layer * z_inc

            prim_path = f"{stack_path}/box_{stack_id}_{i+1}"

            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                print(f"[WARN] failed to reference box at {prim_path}")
                continue

            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.CollisionAPI.Apply(prim)

            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_attr = mass_api.GetMassAttr() or mass_api.CreateMassAttr()
            mass_attr.Set(9.0)

            self.boxes.append(prim_path)

    def create_pick_stack(
        self,
        path,
        pallet_position=(0, 0, 0),
        number_of_boxes=1,
        stack_id=1,
        reverse=False,
        isBottles=False,
    ):
        self.create_xform(f"{path}/stack{stack_id}", (0, 0, 0), (0, 0, 0), (1, 1, 1))
        pallet = DynamicCuboid(
            prim_path=f"{path}/stack{stack_id}/pallet{stack_id}",
            position=pallet_position,
            scale=np.array((1.2, 0.8, 0.144)),
            color=np.array((0.2, 0.08, 0.05)),
            mass=25.0,
        )

        self.pallets.append(pallet)

        if (isBottles):
            self.create_bottles(
                f"{path}/stack{stack_id}",
                number_of_boxes,
                pallet_position,
                stack_id,
                reverse,
            )
        else:
            self.create_boxes(
                f"{path}/stack{stack_id}",
                number_of_boxes,
                pallet_position,
                stack_id,
                reverse,
            )

    def load_shelf_usd(self, position=(0, 0, 0), scale=(1, 1, 1)):
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

            print(f"Shelf loaded at position: {position}")
        else:
            print(f"Failed to load shelf at prim path: {SHELF_PATH}")

    def start_udp_server(self, host=LISTEN_HOST, port=LISTEN_PORT):
        if self.port_in_use(port):
            print(f"Port {port} is already in use. Skipping server start.")
            return

        self.udp.host = host
        self.udp.port = port
        self.udp.start()

    def port_in_use(self, port):
        for conn in psutil.net_connections(kind="udp"):
            if conn.laddr.port == port:
                return True

        return False

    def update(self, step: float = 0.1):
        """Process the queued commands, broadcast joint data, and log performance."""
        start_time = time.time()
        self._process_command_queue()

        self._broadcast_joint_data()

        self._log_performance_stats(start_time)
        self._print_joint_positions(start_time)
        self._capture_overview_camera(start_time)

    def _process_command_queue(self):
        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.parse_and_execute_command(message)

    def _broadcast_joint_data(self):
        if not self.broadcast_stop_event.is_set():
            data = []

            for name, dof_index, is_angular in self.axis_dofs:
                pos = self._robot_controller.get_joint_position_by_index(
                    dof_index, is_angular
                )
                if pos is not None:
                    data.append(f"{name}:{pos:.4f}")

            if data:
                self.udp.send(
                    ";".join(data),
                    self.broadcast_target_host,
                    self.broadcast_target_port,
                )
            else:
                print("[WARN] No joint data to broadcast.")

    def _log_performance_stats(self, current_time):
        if self.print_performance_stats and (
            current_time - self.last_time_check >= 1.0
        ):
            print(
                f"[STATS] UDP Received: {self.udp_message_count} msg/sec | Executed: {self.executed_command_count} cmd/sec"
            )

            self.udp_message_count = 0
            self.executed_command_count = 0
            self.last_time_check = current_time

    def _print_joint_positions(self, current_time):
        if self.print_positions and (
            current_time - self.last_position_print_time >= 1.0
        ):
            print("-----------------------------------------------------------------")
            for name, dof_index, is_angular in self.axis_dofs:
                self._robot_controller.print_joint_position_by_index(
                    dof_index, is_angular
                )

            self.print_box_position("/World/Environment/stack1/box_1_19")
            self.print_box_position("/World/Environment/stack4/box_4_30")
            self.print_box_position("/World/Environment/stack2/box_2_19")

            self.last_position_print_time = current_time

    def _capture_overview_camera(self, current_time):
        if self.overview_camera_active and (
            current_time - self.last_overview_capture_time
            >= self.overview_capture_interval
        ):
            self._robot_controller.camera_capture.capture_image("OverviewCamera")
            self.last_overview_capture_time = current_time

    def print_box_position(self, box_path):
        stage = omni.usd.get_context().get_stage()
        box_prim = stage.GetPrimAtPath(box_path)
        if not box_prim.IsValid():
            print(f"Box prim not found at {box_path}")
            return
        xformable = UsdGeom.Xformable(box_prim)
        for op in xformable.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                pos = op.Get()
                print(f"Box position for {box_path}: {pos}")
                return
        print(f"No translation op found for box at {box_path}")

    def stop_broadcasting(self):
        self.broadcast_stop_event.set()
        if self.broadcast_thread:
            self.broadcast_thread.join()
            self.broadcast_thread = None
        print("[UDPScenario] Broadcast thread stopped.")

    def setup(self):
        """Set up the world, refresh robot controller handles, create scene objects, and start UDP server."""
        self._world = World()
        self._world.reset()
        self._robot_controller.refresh_handles()
        self.axis_dofs = []

        for name, path, is_angular in self.axis_config:
            dof_index = self._robot_controller.get_dof_index_for_joint(path)
            if dof_index == -1:
                print(f"[ERROR] Could not resolve DOF index for {name} ({path})")
            else:
                self.axis_dofs.append((name, dof_index, is_angular))

        self.pallets = []
        self.boxes = []

        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.0, 0.072),
            number_of_boxes=19,
            stack_id=1,
            reverse=True,
            isBottles=False,
        )
        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, -0.9, 0.072),
            number_of_boxes=20,
            stack_id=2,
            reverse=True,
            isBottles=False,
        )
        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.9, 0.072),
            number_of_boxes=35,
            stack_id=3,
            reverse=True,
            isBottles=False,
        )
        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.0, 1.872),
            number_of_boxes=30,
            stack_id=4,
            reverse=True,
            isBottles=True,
        )
        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, -0.9, 1.872),
            number_of_boxes=27,
            stack_id=5,
            reverse=True,
            isBottles=True,
        )
        self.create_pick_stack(
            ENVIRONMENT_PATH,
            pallet_position=(-1.4, 0.9, 1.872),
            number_of_boxes=12,
            stack_id=6,
            reverse=True,
            isBottles=False,
        )

        self.load_shelf_usd(position=(-1.3, -1.4, 0), scale=(1, 0.7, 1))
        self.start_udp_server()

    def unload(self):
        """Tear down the current simulation: stop UDP, reset world, and remove scene objects."""
        self._did_run = False

        self.udp.stop()

        self.stop_broadcasting()

        if self._world is not None:
            self._world.reset()

        self.remove_scenario_specific_prims()

    def remove_scenario_specific_prims(self):
        """Remove objects created by this scenario from the stage."""
        stage = omni.usd.get_context().get_stage()

        prim_paths = (
            [box.prim_path for box in self.boxes]
            + [pallet.prim_path for pallet in self.pallets]
            + [SHELF_PATH]
        )

        for prim_path in prim_paths:
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                stage.RemovePrim(prim_path)

    def _reload_scene_with_model(self, parts):
        if len(parts) < 2:
            print("[ERROR] reload command must be: reload:<grab_usd_filename>")
            return
        
        grab_usd = parts[1]

        self.unload()

        create_new_stage()
        setup_scene(enable_cameras=self.allow_udp_capture, grab_usd=grab_usd)
        
        self.setup()

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
        scenario.stop_broadcasting()
        scenario.udp.stop()
        print("Exiting UDP scenario.")
