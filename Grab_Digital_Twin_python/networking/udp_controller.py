import socket
import threading

class UDPController:
    def __init__(self, host="0.0.0.0", port=9999):
        self.host = host
        self.port = port
        self.callback = None
        self._thread = None

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            print("[UDP Controller] Already running.")
            return

        def udp_server():
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception as e:
                print("SO_REUSEPORT not available:", e)
            try:
                udp_sock.bind((self.host, self.port))
            except Exception as e:
                print(f"[UDP Controller] Bind exception: {e}")
                return
            print(f"[UDP Controller] Listening on {self.host}:{self.port}")
            while True:
                try:
                    data, addr = udp_sock.recvfrom(1024)
                    message = data.decode("utf-8").strip()
                    print(f"[UDP Controller] Received from {addr}: {message}")
                    if self.callback:
                        self.callback(message)
                except Exception as e:
                    print(f"[UDP Controller] Exception: {e}")

        self._thread = threading.Thread(target=udp_server, daemon=True)
        self._thread.start()
