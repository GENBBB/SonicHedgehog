import os
import serial
import numpy as np
from lidar_parser import CalcLidarData
from lidar_filter import filter_lidar_points, binarize_by_degree
from lidar_stabilizer import MedianStabilizer
from lidar_visualizer import LidarVisualizer

# Параметры
COM_PORT = "/dev/ttyUSB0"
CONF_THRESHOLD = 40
PACKETS_PER_FRAME = 40

def read_lidar_packet(ser):
    tmp_str = ""
    flag_54 = False
    while True:
        b = ser.read()
        if not b:
            return None

        val = int.from_bytes(b, 'big')

        if val == 0x54:
            tmp_str += b.hex() + " "
            flag_54 = True
        elif val == 0x2C and flag_54:
            tmp_str += b.hex()
            if len(tmp_str[0:-5].replace(" ", "")) == 90:
                return tmp_str[0:-5]
            else:
                tmp_str = ""
                flag_54 = False
        else:
            tmp_str += b.hex() + " "
            flag_54 = False

def main():
    ser = serial.Serial(port=COM_PORT, baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
    print("Start")
    visualizer = LidarVisualizer()
    stabilizer = MedianStabilizer(spatial_kernel=3)

    angles_buf, dist_buf, conf_buf = [], [], []
    packet_count = 0

    try:
        while True:
            packet = read_lidar_packet(ser)
            if packet is None:
                continue

            lidar_data = CalcLidarData(packet)
            filtered = filter_lidar_points(lidar_data, CONF_THRESHOLD)

            angles_buf.extend(filtered["Degree_angle"])
            dist_buf.extend(filtered["Distance"])
            conf_buf.extend(filtered["Confidence"])
            packet_count += 1

            if packet_count >= PACKETS_PER_FRAME:
                dist_bins, conf_bins = binarize_by_degree(angles_buf, dist_buf, conf_buf)
                stable_bins = stabilizer.update(dist_bins)
                visualizer.plot_frame(stable_bins)

                angles_buf.clear()
                dist_buf.clear()
                conf_buf.clear()
                packet_count = 0

    finally:
        ser.close()

if __name__ == "__main__":
    main()
