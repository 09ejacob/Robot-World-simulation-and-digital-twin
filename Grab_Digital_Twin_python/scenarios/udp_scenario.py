import time
import queue
import threading
import omni.usd
import numpy as np
import psutil
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
from omni.isaac.core import World
from ..global_variables import (
    AXIS1_JOINT_PATH,
    AXIS2_JOINT_PATH,
    AXIS3_JOINT_PATH,
    AXIS4_JOINT_PATH,
    ENVIRONMENT_PATH,
)
from ..networking.udp_controller import UDPController
from omni.isaac.core.objects import DynamicCuboid


class UDPScenario:
    def __init__(self, robot_controller, enable_stats=False):
        self._robot_controller = robot_controller
        self._world = None
        self._did_run = False

        self.command_queue = queue.Queue()

        # Performance tracking
        self.enable_stats = enable_stats
        self.udp_message_count = 0
        self.executed_command_count = 0
        self.last_time_check = time.time()

        # Initialize the UDP server
        self.udp = UDPController()
        self.udp.callback = self._udp_callback

    def _udp_callback(self, message):
        """Receives UDP messages and stores them in a queue."""
        self.command_queue.put(message)
        self.udp_message_count += 1 

    def reset(self):
        """Resets the scenario and stops the UDP server."""
        self._did_run = False
        self.udp.stop()
        if self._world is not None:
            self._world.reset()

    def start_udp_server(self, host="0.0.0.0", port=9999):
        """Starts the UDP server if the port is not already in use."""
        if self.port_in_use(port):
            print(f"Port {port} is already in use. Skipping new server.")
            return

        self.udp.host = host
        self.udp.port = port
        self.udp.start()

    def port_in_use(self, port):
        """Checks if the given port is already in use."""
        for conn in psutil.net_connections(kind="udp"):
            if conn.laddr.port == port:
                return True
        return False

    def parse_and_execute_command(self, message):
        self.executed_command_count += 1
        message = message.rstrip(":").strip()
        parts = [p for p in message.split(":") if p != ""]
        if not parts:
            print("[ERROR] Empty command.")
            return

        command_keyword = parts[0].lower()

        if command_keyword == "tp_robot":
            if len(parts) != 4:
                print(f"[ERROR] Invalid teleport command format: {message}. Expects: tp_robot:x:y:z")
                return
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
            except ValueError:
                print(f"[ERROR] Teleport command must contain only numbers: {message}")
                return
            self._robot_controller.teleport_robot([x, y, z])
            print(f"Teleported robot to: {[x, y, z]}")
            return

        if command_keyword == "nudge_box":
            if len(parts) != 5:
                print(f"[ERROR] Invalid nudge_box command format: {message}. Expects: nudge_box:/path/to/box:x:y:z")
                return
            prim_path = parts[1]
            try:
                dx = float(parts[2])
                dy = float(parts[3])
                dz = float(parts[4])
            except ValueError:
                print(f"[ERROR] nudge_box command must contain only numbers: {message}")
                return
            self.nudge_box(prim_path, (dx, dy, dz))
            print(f"Nudged box at {prim_path} by offset: {[dx, dy, dz]}")
            return

        if command_keyword == "force_data":
            print("Read force sensor value")
            self._robot_controller.read_force_sensor_value()
            return

        if message.lower() == "close_gripper":
            print("Close gripper")
            self._robot_controller.close_gripper()
            return

        if message.lower() == "open_gripper":
            print("Open gripper")
            self._robot_controller.open_gripper()
            return

        if command_keyword.startswith("axis"):
            if len(parts) != 2:
                print("[ERROR] Invalid axis command format:", message, "Expected format: axisX:position")
                return
            try:
                axis_id = int(command_keyword.replace("axis", ""))
                target_value = float(parts[1])
            except ValueError:
                print("[ERROR] Invalid axis id or target value:", message)
                return

            if axis_id == 1:
                self._robot_controller.set_angular_drive_target(AXIS1_JOINT_PATH, target_value)
                print(f"Set angular drive target for axis 1 to {target_value}")
            elif axis_id == 2:
                self._robot_controller.set_prismatic_joint_position(AXIS2_JOINT_PATH, target_value)
                print(f"Set prismatic joint position for axis 2 to {target_value}")
            elif axis_id == 3:
                self._robot_controller.set_prismatic_joint_position(AXIS3_JOINT_PATH, target_value)
                print(f"Set prismatic joint position for axis 3 to {target_value}")
            elif axis_id == 4:
                self._robot_controller.set_angular_drive_target(AXIS4_JOINT_PATH, target_value)
                print(f"Set angular drive target for axis 4 to {target_value}")
            else:
                print("[ERROR] Axis id not recognized:", axis_id)
            return

        print("[ERROR] Command not recognized:", message)

    def nudge_box(self, prim_path, offset):
        stage = omni.usd.get_context().get_stage()
        box_prim = stage.GetPrimAtPath(prim_path)
        if not box_prim.IsValid():
            print(f"Box prim not found at {prim_path}")
            return
        xformable = UsdGeom.Xformable(box_prim)
        translate_op = None
        for op in xformable.GetOrderedXformOps():
            if "translate" in op.GetOpName():
                translate_op = op
                break
        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
        current_translation = translate_op.Get()
        new_translation = current_translation + Gf.Vec3d(*offset)
        translate_op.Set(new_translation)
        print(f"Nudged box at {prim_path} by offset {offset}. New position: {new_translation}")

    def create_xform(self, path, translate=(0, 0, 0), rotation=(0, 0, 0), scale=(1, 1, 1)):
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

    def create_boxes(self, path, num_boxes: int, position=(1, 1, 1), stack_id=1):
        boxes = []
        base_x_pos, base_y_pos, base_z_pos = position
        start_x = base_x_pos - 0.45
        x_inc = 0.3
        row_y = {0: base_y_pos - 0.2, 1: base_y_pos + 0.2}
        base_z = base_z_pos + 0.072 + 0.1
        z_inc = 0.2
        max_col = 3

        for i in range(num_boxes):
            layer = i // 8
            index_in_layer = i % 8
            column = index_in_layer // 2
            row = index_in_layer % 2

            x = start_x + (max_col - column) * x_inc
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
        return boxes

    def create_pick_stack(self, path, pallet_position=(0, 0, 0), number_of_boxes=1, stack_id=1):
        self.create_xform(f"{path}/stack{stack_id}", (0, 0, 0), (0, 0, 0), (1, 1, 1))

        self.pallet = DynamicCuboid(
            prim_path=f"{path}/stack{stack_id}/pallet{stack_id}",
            position=pallet_position,
            scale=np.array((1.2, 0.8, 0.144)),
            color=np.array((0.2, 0.08, 0.05)),
            mass=25.0,
        )

        self.create_boxes(f"{path}/stack{stack_id}", number_of_boxes, pallet_position, stack_id)

    def setup(self):
        self._world = World()
        self._world.reset()

        self.create_pick_stack(ENVIRONMENT_PATH, pallet_position=(1.7, 0.0, 0.072), number_of_boxes=15, stack_id=1)
        self.create_pick_stack(ENVIRONMENT_PATH, pallet_position=(1.7, -1.0, 0.072), number_of_boxes=20, stack_id=2)
        #self.create_pick_stack(ENVIRONMENT_PATH, pallet_position=(1.7, -1.0, 0.072), number_of_boxes=45, stack_id=3)

        self.start_udp_server()

    def update(self, step: float = 0.1):
        """Runs in the Isaac Sim main thread and processes queued UDP commands."""
        start_time = time.time()

        while not self.command_queue.empty():
            message = self.command_queue.get()
            self.parse_and_execute_command(message)

        # Print performance stats every 1 second
        if self.enable_stats and (start_time - self.last_time_check >= 1.0):
            print(
                f"[STATS] UDP Received: {self.udp_message_count} msg/sec | Executed: {self.executed_command_count} cmd/sec"
            )
            self.udp_message_count = 0  # Reset counters
            self.executed_command_count = 0
            self.last_time_check = start_time  # Reset time tracking


if __name__ == "__main__":
    robot_controller = ...
    scenario = UDPScenario(robot_controller)
    scenario.setup()

    try:
        while True:
            scenario.update()
    except KeyboardInterrupt:
        print("Exiting UDP scenario.")
