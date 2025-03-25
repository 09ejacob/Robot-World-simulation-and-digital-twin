import socket
import threading


class UDPController:
    def __init__(self, host="0.0.0.0", port=9999):
        self.host = host
        self.port = port
        self.callback = None
        self._thread = None
        self._stop_event = threading.Event()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            print("[UDP Controller] Already running.")
            return

        self._stop_event.clear()

        def udp_server():
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                udp_sock.bind((self.host, self.port))
            except Exception as e:
                print(f"[UDP Controller] Bind exception: {e}")
                return

            print(f"[UDP Controller] Listening on {self.host}:{self.port}")
            udp_sock.settimeout(0.5)
            while not self._stop_event.is_set():
                try:
                    data, addr = udp_sock.recvfrom(1024)
                    message = data.decode("utf-8").strip()
                    # print(f"[UDP Controller] Received from {addr}: {message}")
                    if self.callback:
                        self.callback(message)
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[UDP Controller] Exception: {e}")
            udp_sock.close()

        self._thread = threading.Thread(target=udp_server, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
        self._thread = None
        print("[UDP Controller] Stopped.")
