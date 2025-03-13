import socket
import time

def send_udp_command(message, host='127.0.0.1', port=9999):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        udp_socket.sendto(message.encode('utf-8'), (host, port))
        print(f"Sent command: {message}")

if __name__ == '__main__':
    while True:
        send_udp_command("axis:2:1.0")
        time.sleep(2)
