import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time

from sensor_msgs.msg import Imu, MagneticField

class PublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.br = TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.magnetic_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion x, y, z, w
        self.translation = [0.0, 0.0, 0.0]
        self.parent_frame = 'world'
        self.child_frame = 'base_link'
        self._imu_data = {
            'orientation': [0.0, 0.0, 0.0, 1.0],
            'angular_velocity': [0.0, 0.0, 0.0],
            'linear_acceleration': [0.0, 0.0, 0.0]
        }
        self._magnetic_field = [0.0, 0.0, 0.0]

    def set_orientation(self, x, y, z, w):
        self.orientation = [x, y, z, w]

    def set_translation(self, x, y, z):
        self.translation = [x, y, z]

    def set_imu(self, orientation, angular_velocity, linear_acceleration):
        """
        Set IMU data for publication.
        orientation: [x, y, z, w] quaternion
        angular_velocity: [x, y, z]
        linear_acceleration: [x, y, z]
        """
        self._imu_data = {
            'orientation': orientation,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }

    def set_magnetic_field(self, magnetic_field):
        """
        Set magnetic field data for publication.
        magnetic_field: [x, y, z] in Tesla
        """
        self._magnetic_field = magnetic_field

    def timer_callback(self):
        # TF broadcast
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        self.br.sendTransform(t)

        # IMU publish
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.child_frame
        o = self._imu_data['orientation']
        imu_msg.orientation.x = o[0]
        imu_msg.orientation.y = o[1]
        imu_msg.orientation.z = o[2]
        imu_msg.orientation.w = o[3]
        av = self._imu_data['angular_velocity']
        imu_msg.angular_velocity.x = av[0]
        imu_msg.angular_velocity.y = av[1]
        imu_msg.angular_velocity.z = av[2]
        la = self._imu_data['linear_acceleration']
        imu_msg.linear_acceleration.x = la[0]
        imu_msg.linear_acceleration.y = la[1]
        imu_msg.linear_acceleration.z = la[2]
        self.imu_pub.publish(imu_msg)

        # MagneticField publish
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.child_frame
        mag_msg.magnetic_field.x = self._magnetic_field[0]
        mag_msg.magnetic_field.y = self._magnetic_field[1]
        mag_msg.magnetic_field.z = self._magnetic_field[2]
        self.magnetic_pub.publish(mag_msg)

def start_tf_node_in_thread():
    import threading
    rclpy.init(args=None)
    node = PublisherNode()

    def spin_node():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    return node, thread

# For standalone testing
if __name__ == '__main__':
    node, thread = start_tf_node_in_thread()
    try:
        while thread.is_alive():
            thread.join(1)
    except KeyboardInterrupt:
        pass
