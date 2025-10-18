import socket
import struct
import time

class SimTelemetry:
    def __init__(self, logger, tel_host, tel_port, proto='tcp'):
        self.logger = logger
        self.proto = proto
        self.tel_host = tel_host
        self.tel_port = tel_port

        if proto == "udp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.tel_host, self.tel_port))
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.bind((tel_host, tel_port))
            self.sock.listen(1)
            self.logger.info(f"Waiting for telemetry TCP on {tel_host}:{tel_port}...")
            conn, _ = self.sock.accept()
            self.sock = conn
            self.logger.info("Connected to udp_diff telemetry")

    def recv_all(self, size):
        buf = b""
        while len(buf) < size:
            chunk = self.sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def recv_tel(self):
        latest_data = None
        try:
            self.sock.setblocking(False)
            while True:
                if self.proto == "udp":
                    try:
                        data, _ = self.sock.recvfrom(65535)
                        latest_data = data
                    except BlockingIOError:
                        break
                else:
                    try:
                        size_bytes = self.sock.recv(4)
                        if not size_bytes:
                            break
                        size = struct.unpack("<I", size_bytes)[0]
                        data = self.recv_all(size)
                        if data:
                            latest_data = data
                    except BlockingIOError:
                        break
                    
            self.sock.setblocking(True)

            if latest_data is None:
                return None

            if not latest_data.startswith(b"WBTG"):
                print('WBT2 -> WBTG')
                return None

            header_size = 4 + 9 * 4
            odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack(
                "<9f", latest_data[4:header_size]
            )
            n = struct.unpack("<I", latest_data[header_size:header_size + 4])[0]
            ranges = []
            if n > 0:
                ranges = struct.unpack(
                    f"<{n}f", latest_data[header_size + 4:header_size + 4 + 4 * n]
                )

            return odom_x, odom_y, odom_th, (vx, vy, vth), (wx, wy, wz), ranges

        except Exception as e:
            self.logger.warn(f"Telemetry receive error: {e}")
            # Вернем сокет в блокирующий режим на случай ошибки
            self.sock.setblocking(True)
            return None


    def sleep(self, dt):
        time.sleep(dt)

