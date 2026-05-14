import socket

ARDUINO_IP = "172.20.10.10"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"STOP", (ARDUINO_IP, UDP_PORT))