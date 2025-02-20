import socket
import threading
import time
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.dynamic_control import _dynamic_control

print("START")

def udp_server(host='0.0.0.0', port=9999):
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.bind((host, port))
    print(f"[UDP Server] Listening on {host}:{port}")

    while True:
        try:
            data, addr = udp_sock.recvfrom(1024)  # Buffer size is 1024 bytes
            message = data.decode('utf-8').strip()
            print(f"[UDP Server] Received from {addr}: {message}")
        except Exception as e:
            print(f"[UDP Server] Exception: {e}")

if __name__ == "__main__":
    udp_thread = threading.Thread(target=udp_server, daemon=True)
    udp_thread.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting UDP control.")

