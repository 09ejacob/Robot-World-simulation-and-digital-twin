import carb
import socket
import threading
import uuid
import struct

MAX_UDP_SIZE = 60000


class UDPController:
    def __init__(self, host="0.0.0.0", port=9999):
        self.host = host
        self.port = port
        self.callback = None
        self._thread = None
        self._stop_event = threading.Event()
        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message, target_host, target_port):
        """
        Send UDP message to a specified target.

        Args:
            message (str or bytes): the message/payload to send.
            target_host (str): destination IP address.
            target_port (int): destination port number.
        """
        try:
            if isinstance(message, str):
                data = message.encode("utf-8")
            elif isinstance(message, bytes):
                data = message
            else:
                raise TypeError(f"Unsupported message type: {type(message)}")

            if len(data) <= MAX_UDP_SIZE:
                self._send_sock.sendto(data, (target_host, target_port))
                return

            # Fragmenting
            msg_id = uuid.uuid4().bytes[:8]  # 8-byte unique ID
            total_chunks = (len(data) + MAX_UDP_SIZE - 1) // MAX_UDP_SIZE

            for i in range(total_chunks):
                start = i * MAX_UDP_SIZE
                end = min(start + MAX_UDP_SIZE, len(data))
                chunk = data[start:end]

                # Fragment header: [msg_id][total_chunks][chunk_index]
                header = msg_id + struct.pack("!HH", total_chunks, i)
                packet = header + chunk
                self._send_sock.sendto(packet, (target_host, target_port))

        except Exception as e:
            carb.log_error(f"[UDP Controller] Send error: {e}")

    def start(self):
        """Start a background thread that listens for incoming UDP messages."""
        if self._thread is not None and self._thread.is_alive():
            print("[MAIN] Already running.")
            return

        self._stop_event.clear()

        def _udp_server():
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                udp_sock.bind((self.host, self.port))
            except Exception as e:
                carb.log_error(f"[UDP Controller] Bind exception: {e}")
                return

            print(f"[UDP Controller] Listening on {self.host}:{self.port}")
            udp_sock.settimeout(0.5)
            while not self._stop_event.is_set():
                try:
                    data, addr = udp_sock.recvfrom(1024)
                    message = data.decode("utf-8").strip()
                    if self.callback:
                        self.callback(message)
                except socket.timeout:
                    continue
                except Exception as e:
                    carb.log_error(f"[UDP Controller] Exception: {e}")
            udp_sock.close()

        self._thread = threading.Thread(target=_udp_server, daemon=True)
        self._thread.start()

    def stop(self):
        """Signal the listening thread to stop and wait for it to finish."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
        self._thread = None
        print("[UDP Controller] Stopped.")
