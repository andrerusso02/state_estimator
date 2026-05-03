import socket
import struct
from typing import Callable

import numpy as np

from publisher import start_tf_node_in_thread
from marker_helper.canva import Canva
from marker_helper import items, presets
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# --- Configuration Constants ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
PAYLOAD_SIZE = 72  # 18 integers * 4 bytes
UNPACK_FORMAT = '<18i'

def sensor_value_to_float(val1: int, val2: int) -> float:
    """Converts Zephyr's int32_t val1 and val2 back to a standard float."""
    return val1 + (val2 / 1000000.0)

def parse_imu_packet(data: bytes) -> dict:
    """
    Unpacks the 72-byte raw payload and reconstructs the IMU data.
    """
    unpacked = struct.unpack(UNPACK_FORMAT, data)
    
    return {
        'accel': (
            sensor_value_to_float(unpacked[0], unpacked[1]),
            sensor_value_to_float(unpacked[2], unpacked[3]),
            sensor_value_to_float(unpacked[4], unpacked[5])
        ),
        'gyro': (
            sensor_value_to_float(unpacked[6], unpacked[7]),
            sensor_value_to_float(unpacked[8], unpacked[9]),
            sensor_value_to_float(unpacked[10], unpacked[11])
        ),
        'magn': (
            sensor_value_to_float(unpacked[12], unpacked[13]),
            sensor_value_to_float(unpacked[14], unpacked[15]),
            sensor_value_to_float(unpacked[16], unpacked[17])
        )
    }

def run_udp_receiver(ip: str, port: int, callback: Callable[[dict], None]):
    """
    Starts the UDP server, listens for packets, and triggers the 
    provided callback function whenever valid IMU data arrives.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    
    print(f"Listening for IMU data on {ip}:{port}...")
    print("-" * 60)
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            
            if len(data) == PAYLOAD_SIZE:
                parsed_data = parse_imu_packet(data)
                # Trigger the callback with the parsed dictionary
                callback(parsed_data)
            else:
                print(f"Ignored packet of incorrect size: {len(data)} bytes")
                
    except KeyboardInterrupt:
        print("\nReceiver stopped by user. Closing socket.")
    finally:
        sock.close()


tf_node, tf_thread = start_tf_node_in_thread()

# Initialize marker visualization (enable=True)
canva = Canva(tf_node, enable=True)

# Store marker items for updating
arrow_accel = None
arrow_magn = None

def on_receive(imu_data: dict):
    global arrow_accel, arrow_magn
    accel = np.array(imu_data['accel'])
    gyro = np.array(imu_data['gyro'])
    magn = np.array(imu_data['magn'])

    # print accel with formatting
    print(f"Accel: {accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}")

    accel_normalized = accel / np.linalg.norm(accel)
    magn_normalized = magn / np.linalg.norm(magn)

    # x_vector = cross product of accel and magn
    x_vector = np.cross(accel_normalized, magn_normalized)

    # # compute quaternion from orientation vector (assuming small angles)
    # norm = np.linalg.norm(orientation)
    # if norm > 0:
    #     orientation /= norm  # Normalize the orientation vector
    # # Convert orientation vector to quaternion (simple approximation)
    # qx = orientation[0] * np.sin(norm / 2)
    # qy = orientation[1] * np.sin(norm / 2)
    # qz = orientation[2] * np.sin(norm / 2)
    # qw = np.cos(norm / 2)

    # tf_node.set_orientation(qx, qy, qz, qw)
    # tf_node.set_imu([qx, qy, qz, qw], gyro.tolist(), accel.tolist())
    # tf_node.set_magnetic_field(magn.tolist())

    # Draw 3D arrows for magn and accel using marker_helper Arrows
    canva.clear()
    arrow_accel = items.Arrow(((0, 0, 0), tuple(accel_normalized)), size=0.5, color=presets.GOLD)
    arrow_magn = items.Arrow(((0, 0, 0), tuple(magn_normalized)), size=0.5, color=presets.MAGENTA)
    x_vector_arrow = items.Arrow(((0, 0, 0), tuple(x_vector)), size=0.5, color=presets.GREEN)
    canva.add(arrow_accel)
    canva.add(arrow_magn)
    canva.add(x_vector_arrow)

    canva.draw()

    # print orientation in degrees
    # orientation_degrees = np.degrees(orientation)
    # print(f"Orientation (degrees): {orientation_degrees}")

if __name__ == "__main__":
    run_udp_receiver(UDP_IP, UDP_PORT, callback=on_receive)