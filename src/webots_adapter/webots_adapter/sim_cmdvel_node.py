import socket
import struct

class SimCmdVel:
    def __init__(self, logger, cmd_host, cmd_port, proto='tcp'):
        self.logger = logger
        self.proto = proto
        self.host = cmd_host
        self.port = cmd_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_cmd(self, linear, angular):
        packet = struct.pack("<2f", linear, angular)
        self.sock.sendto(packet, (self.host, self.port))

